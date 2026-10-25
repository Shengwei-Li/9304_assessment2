function ctrl_server(port, extra_ms, jitter_ms)
%CTRL_SERVER  The boat's controller as a separate process (software in the loop).
%   Run it in its own MATLAB session (current folder: matlab/), then run
%   hydrofoil_sim with ctrl_ext = 1. From a terminal in the repository root:
%       matlab -batch "cd('matlab'); ctrl_server"      (or double-click start_sil.bat)
%   It runs digital_controller.m, the same code as the in-model controller,
%   and knows nothing about the boat except what arrives over TCP.
%
%   ctrl_server(port, extra_ms, jitter_ms)
%     port       TCP port to listen on (default 5501)
%     extra_ms   extra computing time added to every sample [ms] (default 0),
%                to emulate a slower processor
%     jitter_ms  plus a random extra time, uniform in [0, jitter_ms] (default 0)
%   A run can add to both through its reset message (sil_extra_ms and
%   sil_jitter_ms in the model), so delay tests need no restart.
%
%   Messages are big-endian doubles (Java's byte order):
%     plant -> controller  [type seq y(3) delta(2) r]      type 1 = sample
%                          [0 0 extra_ms jitter_ms 0 0 0 0] type 0 = reset (new run)
%     controller -> plant  [seq du(2) xh(5) proc_ms]       reply (see below)
%   du is the command computed from sample seq; the plant applies it at
%   sample seq+1, which is the one-sample delay the controller is designed for.
%   If computing takes longer than a sample, readings queue up. The older ones
%   then only update the filter and only the newest one is answered, as a
%   real-time controller would; the plant holds its last command until a
%   reply comes.
%   The server keeps running and accepts the next run when one ends (Ctrl+C stops it).
%
%   MATLAB's own tcpserver needs the Instrument Control Toolbox, so the server
%   socket is Java's (part of every MATLAB installation).

if nargin < 1 || isempty(port),      port = 5501;   end
if nargin < 2 || isempty(extra_ms),  extra_ms = 0;  end
if nargin < 3 || isempty(jitter_ms), jitter_ms = 0; end

p  = params();
[~, ~, ~, ~, ~, u0] = hydrofoil_lin(p);
dc = design_digital(p, p.Ts_ctrl, 1);       % same design as the in-model controller
NIN = 8;

srv = java.net.ServerSocket(port);
cleanup = onCleanup(@() srv.close());
fprintf('Controller listening on port %d (extra %g ms, jitter %g ms). Ctrl+C to stop.\n', ...
        port, extra_ms, jitter_ms);
while true
    sock = srv.accept();                     % wait for a run of the model
    sock.setTcpNoDelay(true);                % send each reply at once
    in  = java.io.DataInputStream(java.io.BufferedInputStream(sock.getInputStream()));
    out = java.io.DataOutputStream(java.io.BufferedOutputStream(sock.getOutputStream()));
    fprintf('Plant connected.\n');
    n = 0;  tproc = zeros(10000, 1);  ex = extra_ms;  ji = jitter_ms;  skipped = 0;
    msg = zeros(1, NIN);
    try
        while true
            for j = 1:NIN, msg(j) = in.readDouble(); end    % blocks until a message arrives
            t0 = tic;
            % if the controller fell behind, newer readings are already waiting:
            % pass the older ones through the filter (it must see every sample,
            % it predicts Ts ahead each call) and reply only to the newest one,
            % instead of answering a growing queue of out-of-date readings
            while msg(1) == 1 && in.available() >= 8*NIN
                digital_controller(msg(3:5).', msg(6:7).', msg(8), dc, p, u0);
                for j = 1:NIN, msg(j) = in.readDouble(); end
                skipped = skipped + 1;
            end
            if msg(1) == 0                                  % new run
                clear digital_controller
                n = 0;  skipped = 0;  ex = extra_ms + msg(3);  ji = jitter_ms + msg(4);
                fprintf('New run: extra %g ms, jitter %g ms.\n', ex, ji);
                continue
            end
            y = msg(3:5).';  delta = msg(6:7).';  r = msg(8);
            [~, xh, du] = digital_controller(y, delta, r, dc, p, u0);
            wait_s = (ex + ji*rand)/1e3;
            while toc(t0) < wait_s, end                     % emulated slower processor
            n = n + 1;  tproc(n) = toc(t0);
            reply = [msg(2), du(:).', xh(:).', 1e3*tproc(n)];
            for j = 1:numel(reply), out.writeDouble(reply(j)); end
            out.flush();
        end
    catch                                                   % plant closed the connection
    end
    sock.close();
    fprintf('Plant disconnected after %d samples, computing time mean %.3f ms, max %.3f ms.\n', ...
            n, 1e3*mean(tproc(1:n)), 1e3*max(tproc(1:n)));
end
end
