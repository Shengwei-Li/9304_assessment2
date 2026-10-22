%HYDROFOIL_SIM_INIT  Default workspace variables for hydrofoil_sim.slx.
%   Called automatically when the model is opened and before each run.
%   Only variables that do not exist yet are created, so values set by
%   run_simulink.m (or by hand) are kept.
%
%   Default run: 50 Hz digital controller on (one-sample computation delay), 5 cm height step at t = 1 s, regular head
%   waves, sensor noise and actuator limits all on, 10 s.

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
if ~exist('r_ts', 'var')                            % height reference: +5 cm at t = 1 s
    tg_ = (0:0.005:Tstop)';
    r_ts = timeseries(0.05*(tg_ >= 1), tg_);
    clear tg_
end
if ~exist('u_cmd_ts', 'var')                        % no extra open-loop foil command
    u_cmd_ts = timeseries(zeros(2, 2), [0; Tstop]);
end
