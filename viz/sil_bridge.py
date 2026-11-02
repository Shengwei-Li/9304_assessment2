"""Relay between hydrofoil_sim.slx (telemetry over TCP) and the 3D view (browser).

    python viz/sil_bridge.py            then open http://localhost:8000

- Listens on port 5502 for SilTelemetry.m (one model run at a time).
- Serves the viewer on http://localhost:8000 and forwards every telemetry
  packet to it over a WebSocket (/ws).
- Records every run to viz/recordings/run_<date>_<time>.bin, which the viewer
  can replay at any speed (for recording a video).
- Saves still images of the 3D view (key S in the viewer) to viz/snapshots.

Packets are big-endian doubles, each preceded by its length n (also a double):
    setup   [0, ...]  sample  [1, ...]   (see matlab/SilTelemetry.m)
Only the Python standard library is used.
"""
import asyncio
import base64
import datetime
import hashlib
import json
import mimetypes
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REC_DIR = os.path.join(HERE, 'recordings')
SNAP_DIR = os.path.join(HERE, 'snapshots')
TEL_PORT = 5502
HTTP_PORT = 8000

clients = set()          # open WebSocket writers
last_setup = None        # newest setup packet, sent to viewers that join late


# ---------------------------------------------------------------- WebSocket
def ws_frame(payload: bytes, opcode=0x2) -> bytes:
    n = len(payload)
    if n < 126:
        head = struct.pack('!BB', 0x80 | opcode, n)
    elif n < 65536:
        head = struct.pack('!BBH', 0x80 | opcode, 126, n)
    else:
        head = struct.pack('!BBQ', 0x80 | opcode, 127, n)
    return head + payload


async def broadcast(payload: bytes):
    frame = ws_frame(payload)
    for w in list(clients):
        try:
            w.write(frame)
        except Exception:
            clients.discard(w)


async def ws_session(reader, writer, key):
    accept = base64.b64encode(hashlib.sha1(
        (key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
    writer.write(('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n'
                  'Connection: Upgrade\r\nSec-WebSocket-Accept: %s\r\n\r\n' % accept).encode())
    if last_setup is not None:
        writer.write(ws_frame(last_setup))
    await writer.drain()
    clients.add(writer)
    try:
        while True:                       # read (and ignore) client frames until close
            b = await reader.readexactly(2)
            op, n = b[0] & 0x0F, b[1] & 0x7F
            if n == 126:
                n = struct.unpack('!H', await reader.readexactly(2))[0]
            elif n == 127:
                n = struct.unpack('!Q', await reader.readexactly(8))[0]
            if b[1] & 0x80:
                await reader.readexactly(4)
            await reader.readexactly(n)
            if op == 0x8:
                break
    except (asyncio.IncompleteReadError, ConnectionError):
        pass
    finally:
        clients.discard(writer)
        writer.close()


# ---------------------------------------------------------------- HTTP
async def http_handler(reader, writer):
    try:
        head = await reader.readuntil(b'\r\n\r\n')
    except (asyncio.IncompleteReadError, asyncio.LimitOverrunError, ConnectionError):
        writer.close()
        return
    lines = head.decode('latin-1').split('\r\n')
    method, path, _ = (lines[0].split(' ') + ['', '', ''])[:3]
    hdr = {}
    for ln in lines[1:]:
        if ':' in ln:
            k, v = ln.split(':', 1)
            hdr[k.strip().lower()] = v.strip()
    if path == '/ws' and hdr.get('upgrade', '').lower() == 'websocket':
        await ws_session(reader, writer, hdr['sec-websocket-key'])
        return

    path = path.split('?')[0]
    if method == 'POST' and path == '/snapshot':           # PNG data URL from the viewer
        n = int(hdr.get('content-length', '0'))
        data = (await reader.readexactly(n)).decode('ascii')
        os.makedirs(SNAP_DIR, exist_ok=True)
        name = datetime.datetime.now().strftime('snap_%Y%m%d_%H%M%S.png')
        with open(os.path.join(SNAP_DIR, name), 'wb') as f:
            f.write(base64.b64decode(data.split(',', 1)[1]))
        body, ctype, code = ('snapshots/' + name).encode(), 'text/plain', 200
    elif path == '/recordings/':                    # list of recorded runs
        files = sorted(f for f in os.listdir(REC_DIR) if f.endswith('.bin')) if os.path.isdir(REC_DIR) else []
        body, ctype, code = json.dumps(files[::-1]).encode(), 'application/json', 200
    else:
        rel = 'index.html' if path in ('', '/') else path.lstrip('/')
        full = os.path.normpath(os.path.join(HERE, rel))
        if not full.startswith(HERE) or not os.path.isfile(full):
            body, ctype, code = b'not found', 'text/plain', 404
        else:
            with open(full, 'rb') as f:
                body = f.read()
            ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
            if full.endswith('.js'):
                ctype = 'text/javascript'
            code = 200
    writer.write(('HTTP/1.1 %d OK\r\nContent-Type: %s\r\nContent-Length: %d\r\n'
                  'Cache-Control: no-store\r\nConnection: close\r\n\r\n' % (code, ctype, len(body))).encode() + body)
    await writer.drain()
    writer.close()


# ---------------------------------------------------------------- telemetry from Simulink
async def telemetry_handler(reader, writer):
    global last_setup
    os.makedirs(REC_DIR, exist_ok=True)
    name = datetime.datetime.now().strftime('run_%Y%m%d_%H%M%S.bin')
    rec = open(os.path.join(REC_DIR, name), 'wb')
    n_samples = 0
    print('Simulink connected, recording to recordings/%s' % name, flush=True)
    try:
        while True:
            nb = await reader.readexactly(8)
            n = int(struct.unpack('>d', nb)[0])
            payload = await reader.readexactly(8 * n)
            rec.write(nb + payload)
            if payload[:8] == struct.pack('>d', 0.0):
                last_setup = payload
            else:
                n_samples += 1
            await broadcast(payload)
    except (asyncio.IncompleteReadError, ConnectionError):
        pass
    finally:
        rec.close()
        writer.close()
        print('Run ended: %d samples (%.1f s), %d viewer(s) connected.'
              % (n_samples, n_samples * 0.02, len(clients)), flush=True)


async def main():
    tel = await asyncio.start_server(telemetry_handler, '127.0.0.1', TEL_PORT)
    web = await asyncio.start_server(http_handler, '127.0.0.1', HTTP_PORT)
    print('Telemetry port %d, viewer at http://localhost:%d  (Ctrl+C to stop)' % (TEL_PORT, HTTP_PORT), flush=True)
    async with tel, web:
        await asyncio.gather(tel.serve_forever(), web.serve_forever())


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        sys.exit(0)
