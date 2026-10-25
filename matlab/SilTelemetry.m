classdef SilTelemetry < matlab.System
    %SILTELEMETRY  Sends the simulation state to sil_bridge.py for the 3D view.
    %   Used by the Telemetry block of hydrofoil_sim.slx when viz_on = 1. It only
    %   reads signals; nothing it does affects the simulation. If the bridge is
    %   not running it prints a warning once and does nothing.
    %
    %   Packets are big-endian doubles, each preceded by its length:
    %     setup  [0, Ts, U, ls, x(2), z(2), d0, c, u0(2), u_min, u_max,
    %             wave_on, noise_on, ctrl_ext, lockstep, n, a(n), om(n), k(n), ph(n)]
    %     sample [1, t, wall, x(4), delta(2), y(3), r, xh(5), st(3), u_fb(2)]
    %   wall is the wall-clock time since the start [s], so the viewer can show
    %   whether the simulation keeps up with real time.

    properties (Nontunable)
        Host = '127.0.0.1'
        Port = 5502
        Setup = 0                 % setup packet body, built by hydrofoil_sim_init
        Enabled = false           % viz_on
    end

    properties (Access = private)
        c                 % output stream of the socket
        sock
        t0
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.c = [];  obj.sock = [];  obj.t0 = tic;
            if ~obj.Enabled, return; end
            try
                sock = java.net.Socket();                  % Java socket, see SilLink.m
                sock.connect(java.net.InetSocketAddress(obj.Host, obj.Port), 1000);
                sock.setTcpNoDelay(true);
                obj.c = java.io.DataOutputStream(java.io.BufferedOutputStream(sock.getOutputStream()));
                obj.sock = sock;
                obj.send(obj.Setup);
            catch
                obj.c = [];
                warning('SilTelemetry:nobridge', ...
                        'sil_bridge.py is not running on port %d; no 3D view.', obj.Port);
            end
            obj.t0 = tic;                  % wall clock starts with the simulation
        end

        function stepImpl(obj, t, x, delta, y, r, xh, st, u_fb)
            if isempty(obj.c), return; end
            obj.send([1, t, toc(obj.t0), x(:).', delta(:).', y(:).', r, xh(:).', st(:).', u_fb(:).']);
        end

        function releaseImpl(obj)
            if ~isempty(obj.sock), obj.sock.close(); end
            obj.c = [];  obj.sock = [];
        end

        function send(obj, v)
            try
                obj.c.writeDouble(numel(v));
                for j = 1:numel(v), obj.c.writeDouble(v(j)); end
                obj.c.flush();
            catch
                obj.c = [];                % bridge went away: stop sending
            end
        end

        function n = getNumInputsImpl(~),  n = 8; end
        function n = getNumOutputsImpl(~), n = 0; end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl, simMode = 'Interpreted execution'; end
        function flag = showSimulateUsingImpl, flag = false; end
    end
end
