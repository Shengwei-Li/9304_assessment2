%HYDROFOIL_SIM_INIT  Default workspace variables for hydrofoil_sim.slx.
%   Called automatically when the model is opened and before each run.
%   Only variables that do not exist yet are created, so values set by
%   run_simulink.m (or by hand) are kept.
%
%   Default run: 50 Hz digital controller on (one-sample computation delay), 5 cm height step at t = 1 s, regular head
%   waves, sensor noise and actuator limits all on, 10 s, controller inside the
%   model, no telemetry.

if ~exist('p', 'var'),  p = params();  end
if ~exist('A', 'var') || ~exist('B', 'var') || ~exist('E', 'var') || ~exist('u0', 'var')
    [A, B, E, C, D, u0] = hydrofoil_lin(p);
end
if ~exist('dc', 'var'),    dc = design_digital(p, p.Ts_ctrl, 1);  end   % 50 Hz controller, one-sample delay
if ~exist('wv', 'var'),    wv   = wave_params(p); end   % 5 cm amplitude, 8 m long
if ~exist('Tstop', 'var'),     Tstop = 10;    end   % simulation time [s]
if ~exist('ctrl_on', 'var'),   ctrl_on = 1;   end   % 1 = controller on
if ~exist('act_on', 'var'),    act_on = 1;    end   % 1 = actuator limits on
if ~exist('wave_on', 'var'),   wave_on = 1;   end   % 1 = waves on
if ~exist('noise_on', 'var'),  noise_on = 1;  end   % 1 = sensor noise on
if ~exist('ctrl_ext', 'var'),  ctrl_ext = 0;  end   % 1 = controller in ctrl_server.m over TCP
if ~exist('sil_lockstep', 'var'), sil_lockstep = 1; end   % 1 = wait for every reply, 0 = real time
if ~exist('sil_extra_ms', 'var'),  sil_extra_ms = 0;  end   % test: slower controller [ms]
if ~exist('sil_jitter_ms', 'var'), sil_jitter_ms = 0; end   % test: random extra time [ms]
if ~exist('viz_on', 'var'),    viz_on = 0;    end   % 1 = telemetry to sil_bridge.py (3D view)
% setup packet for the 3D view (see SilTelemetry.m), rebuilt every run
sil_setup = [0, dc.Ts, p.U, p.ls, p.x(:).', p.z(:).', p.d0, p.c, u0(:).', p.u_min(1), p.u_max(1), ...
             wave_on, noise_on, ctrl_ext, sil_lockstep, numel(wv.a), ...
             wv.a(:).', wv.om(:).', wv.k(:).', wv.ph(:).'];
if ~exist('r_ts', 'var')                            % height reference: +5 cm at t = 1 s
    tg_ = (0:0.005:Tstop)';
    r_ts = timeseries(0.05*(tg_ >= 1), tg_);
    clear tg_
end
if ~exist('u_cmd_ts', 'var')                        % no extra open-loop foil command
    u_cmd_ts = timeseries(zeros(2, 2), [0; Tstop]);
end
