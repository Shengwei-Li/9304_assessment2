classdef SilLink < matlab.System
    %SILLINK  Simulink side of the software-in-the-loop link to ctrl_server.m.
    %   Used by the "External (TCP)" choice of the Controller block in
    %   hydrofoil_sim.slx. Every sample (p.Ts_ctrl) it sends the sensor readings
    %   to the controller process and outputs the command that process computed
    %   from the PREVIOUS sample, i.e. the link is the one-sample computation
    %   delay the controller was designed for.
    %
    %   Lockstep = true   wait for the reply before the simulation continues:
    %                     deterministic, must give the same result as the
    %                     in-model controller (checked in run_sil.m)
    %   Lockstep = false  real time: use whatever reply has arrived by the next
    %                     sample; a reply that is too late is counted and the
    %                     previous command is held (model with pacing on)
    %
    %   Outputs: u_fb [2]   foil-angle correction [rad]
    %            xh   [5]   controller's estimate (telemetry only)
    %            st   [3]   [late replies so far; controller computing time [ms];
    %                        round-trip time [ms] (lockstep only, else 0)]

    properties (Nontunable)
        Host = '127.0.0.1'
        Port = 5501
        Lockstep = true
        ExtraMs = 0       % test only: extra computing time the controller adds [ms]
        JitterMs = 0      % test only: plus uniform random time in [0, JitterMs] [ms]
    end

    properties (Access = private)
        sock              % java.net.Socket and its streams
        in
        out
        k                 % sample counter
        u                 % command applied in this sample
        xh
        late
        proc
        rtt
        next              % reply waiting to be applied at the next sample (lockstep)
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Java sockets: MATLAB's tcpclient waits for the 15.6 ms Windows
            % timer on every write and read (measured 31 ms round trip), Java
            % sends at once (0.3-0.4 ms median)
            obj.sock = java.net.Socket(obj.Host, obj.Port);
            obj.sock.setTcpNoDelay(true);
            obj.sock.setSoTimeout(5000);                   % a blocking read fails after 5 s
            obj.in  = java.io.DataInputStream(java.io.BufferedInputStream(obj.sock.getInputStream()));
            obj.out = java.io.DataOutputStream(java.io.BufferedOutputStream(obj.sock.getOutputStream()));
            obj.send([0, 0, obj.ExtraMs, obj.JitterMs, 0, 0, 0, 0]);   % reset
        end

        function resetImpl(obj)
            obj.k = 0;  obj.u = zeros(2, 1);  obj.xh = zeros(5, 1);
            obj.late = 0;  obj.proc = 0;  obj.rtt = 0;  obj.next = [];
        end

        function [u_fb, xh, st] = stepImpl(obj, y, delta, r)
            % 1. the command computed from the previous sample
            if obj.Lockstep
                if ~isempty(obj.next), obj.take(obj.next); end
            else
                got = [];
                while obj.in.available() >= 72
                    got = obj.receive();                   % keep the newest reply
                end
                if ~isempty(got)
                    obj.take(got);
                    if got(1) < obj.k - 1, obj.late = obj.late + 1; end
                elseif obj.k > 0
                    obj.late = obj.late + 1;               % nothing arrived: hold the old command
                end
            end
            u_fb = obj.u;  xh = obj.xh;

            % 2. send this sample's readings
            t0 = tic;
            obj.send([1, obj.k, y(:).', delta(:).', r]);
            if obj.Lockstep
                obj.next = obj.receive();                  % wait for the reply
                obj.rtt  = 1e3*toc(t0);
            end
            obj.k = obj.k + 1;
            st = [obj.late; obj.proc; obj.rtt];
        end

        function releaseImpl(obj)
            if ~isempty(obj.sock), obj.sock.close(); end
            obj.sock = [];
        end

        function take(obj, msg)
            obj.u = msg(2:3).';  obj.xh = msg(4:8).';  obj.proc = msg(9);
        end

        function send(obj, v)
            for j = 1:numel(v), obj.out.writeDouble(v(j)); end
            obj.out.flush();
        end

        function m = receive(obj)
            m = zeros(1, 9);
            for j = 1:9, m(j) = obj.in.readDouble(); end
        end

        function n = getNumInputsImpl(~),  n = 3; end
        function n = getNumOutputsImpl(~), n = 3; end
        function [a, b, c] = getOutputSizeImpl(~), a = [2 1]; b = [5 1]; c = [3 1]; end
        function [a, b, c] = getOutputDataTypeImpl(~), a = 'double'; b = 'double'; c = 'double'; end
        function [a, b, c] = isOutputComplexImpl(~), a = false; b = false; c = false; end
        function [a, b, c] = isOutputFixedSizeImpl(~), a = true; b = true; c = true; end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl, simMode = 'Interpreted execution'; end
        function flag = showSimulateUsingImpl, flag = false; end
    end
end
