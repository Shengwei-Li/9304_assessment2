%% RUN_OBSERVER_COMPARE  Pole-placement observers against the Kalman filter.
%   The 50 Hz controller (design_digital, one-sample delay) is kept and only the
%   observer gain M is changed. Pole-placement observers put the error dynamics
%   e(k+1) = Ade*(I - M*Ce)*e(k) at chosen poles:
%     PP-3x   the textbook choice, three times faster than the controller
%     PP-1x   as fast as the controller
%     PP-KF   at the poles of the Kalman filter (same speed, different gain)
%   Tests (nonlinear model, actuators, one-sample delay):
%     waves   5 cm step in regular 5 cm / 8 m head waves with sensor noise
%     sea S2  irregular sea Hs 0.15 m, Tp 3 s, with noise (3 seeds)
%     speed   calm water, true speed 6 m/s instead of 7 from t = 0: deviation
%             from the nominal response (5 cm step at 1 s), peak and the time
%             after which it stays below 5 mm
%   Results are printed and saved to figs/observer_compare.mat.

clear; close all; clc;
p  = params();
[~, ~, ~, ~, ~, u0] = hydrofoil_lin(p);
dc = design_digital(p, p.Ts_ctrl, 1);
Ts = dc.Ts;  d2r = pi/180;
ref = @(t) 0.05*(t >= 1);

% observer gains
sK = log(dc.polesK(abs(dc.polesK) > 1e-6))/Ts;        % controller poles (continuous equivalent)
[~, i] = sort(abs(sK));  sK = sK(i);
sK5 = sK(1:5);                                        % five slowest controller poles
sKF = log(dc.polesL)/Ts;
cases = {'Kalman', [], ...
         'PP-3x',  3*sK5, ...
         'PP-1x',  sK5, ...
         'PP-KF',  sKF};
M = cell(1, numel(cases)/2);  names = cases(1:2:end);
for j = 1:numel(names)
    s = cases{2*j};
    if isempty(s)
        M{j} = dc.M;
    else
        L = place(dc.Ade', dc.Ce', exp(s*Ts)).';      % eig(Ade - L*Ce) at the chosen poles
        M{j} = dc.Ade \ L;                            % current-estimator form: L = Ade*M
    end
    zo = eig(dc.Ade - dc.Ade*M{j}*dc.Ce);
    fprintf('%-7s slowest observer pole %6.2f rad/s, fastest %7.1f rad/s, |M| = %6.2f\n', ...
            names{j}, -max(real(log(zo)/Ts)), -min(real(log(zo)/Ts)), norm(M{j}));
end

wvr = wave_params(p);
nom = sim_digital(p, p, dc, ref, 12, struct('delay', 1));          % nominal calm-water step
pslow = p;  pslow.U = 6;
R = struct();
fprintf('\n%-7s | waves: h err  est err  foil rate rms | S2: h err | U = 6 m/s: peak dev, back within 5 mm\n', 'obs');
for j = 1:numel(names)
    dj = dc;  dj.M = M{j};
    o = sim_digital(p, p, dj, ref, 10, struct('delay', 1, 'wave', wvr, 'noise', true, 'seed', 1));
    k = o.t > 3;  kk = o.tk > 3 & o.tk < o.t(end);
    R(j).name   = names{j};
    R(j).h_rms  = 100*rms(o.x(k,1) - 0.05);
    R(j).e_rms  = 100*rms(interp1(o.t, o.x(:,1), o.tk(kk)) - o.xhat(kk,1));
    R(j).rate   = rms(diff(o.delta(k,1))./diff(o.t(k)))/d2r;
    R(j).wave   = o;
    hs = zeros(1,3);
    for sd = 1:3
        wv = wave_irregular(p, 0.15, 3.0, sd);
        os = sim_digital(p, p, dj, @(t) 0, 30, struct('delay', 1, 'wave', wv, 'noise', true, 'seed', sd));
        hs(sd) = 100*rms(os.x(os.t > 5, 1));
    end
    R(j).S2 = hs;
    os = sim_digital(pslow, p, dj, ref, 12, struct('delay', 1));
    R(j).speed = os;
    R(j).diverged = os.stopped;                       % foils left the water: simulation stopped
    if os.stopped
        R(j).peak = NaN;  R(j).back = NaN;
        sp = sprintf('diverged at t = %.1f s', os.t(end));
    else
        dev = interp1(os.t, os.x(:,1), nom.t) - nom.x(:,1);
        R(j).peak = 100*max(abs(dev));
        last = find(abs(dev) > 0.005, 1, 'last');
        R(j).back = nom.t(min(last + 1, numel(nom.t)));   % from t = 0, when the error starts
        sp = sprintf('%5.1f cm, %4.1f s', R(j).peak, R(j).back);
    end
    fprintf('%-7s | %5.2f cm  %5.2f cm  %5.1f deg/s     | %4.2f-%4.2f cm | %s\n', ...
            names{j}, R(j).h_rms, R(j).e_rms, R(j).rate, min(hs), max(hs), sp);
end
save(fullfile(fileparts(mfilename('fullpath')), 'figs', 'observer_compare.mat'), 'R', 'nom', 'names');
