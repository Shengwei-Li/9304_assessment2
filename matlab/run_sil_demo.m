%% RUN_SIL_DEMO  30 s real-time run for the 3D view (software in the loop).
%   Before running:
%     1. ctrl_server running in a second MATLAB (current folder matlab/)
%     2. python viz/sil_bridge.py running, http://localhost:8000 open
%   (start_sil.bat in the repository root starts both.)
%   Irregular head sea S2 (Hs 0.15 m, Tp 3 s), 5 cm height step at t = 5 s,
%   controller in the separate process, real time. The run is also recorded
%   in viz/recordings for replay.

clear; close all; clc;
mdl = 'hydrofoil_sim';
p  = params();
wv = wave_irregular(p, 0.15, 3.0, 1);
Tstop = 30;
tg_ = (0:0.005:Tstop)';  r_ts = timeseries(0.05*(tg_ >= 5), tg_);  clear tg_
hydrofoil_sim_init;                          % everything else at its default
ctrl_ext = 1;  sil_lockstep = 0;  viz_on = 1;

load_system(mdl);
set_param(mdl, 'EnablePacing', 'on', 'PacingRate', 1);
out = sim(mdl);
set_param(mdl, 'EnablePacing', 'off');

lk = reshape(permute(out.link.Data, [3 1 2]), numel(out.link.Time), []);
fprintf('Done: %d late replies out of %d samples, controller computing %.2f ms on average.\n', ...
        lk(end,1), size(lk,1) - 1, mean(lk(2:end,2)));
