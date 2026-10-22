%% RUN_SIMULINK  Build, run and check the Simulink model hydrofoil_sim.
%   1. Builds the model if it does not exist
%   2. Open loop (controller off): checks against ode45 and lsim, and an
%      open-loop demonstration (fig6)
%   3. Closed loop (50 Hz digital controller): checks against sim_digital.m, and a closed-loop
%      demonstration with waves and sensor noise (fig14)
%   Figures are saved in ./figs

clear; close all; clc;
here = fileparts(mfilename('fullpath'));
mdl  = 'hydrofoil_sim';
if ~exist(fullfile(here, [mdl '.slx']), 'file'), build_hydrofoil_sim(); end
load_system(mdl);
set(groot, 'defaultLineLineWidth', 1.3, 'defaultAxesFontSize', 10);
d2r = pi/180;

%% Workspace variables used by the model
p = params();
[A, B, E, C, D, u0] = hydrofoil_lin(p);

dc   = design_digital(p, p.Ts_ctrl, 1);        % 50 Hz controller, one-sample delay
wv   = wave_params(p);                          % 5 cm amplitude, 8 m wavelength

% open-loop runs: controller off, no height reference
ctrl_on = 0;
r_ts = timeseries([0; 0], [0; 100]);

%% Check 1: ideal actuators, calm water, small front-foil step
Tstop = 4;  act_on = 0;  wave_on = 0;  noise_on = 0;
tg = (0:0.005:Tstop)';
du = [0.1*d2r*(tg >= 1), 0*tg];
u_cmd_ts = timeseries(du, tg);
out = sim(mdl);

% independent reference: ode45 on hydrofoil_nl with the same interpolated input
ufun = @(t) u0(:) + interp1(tg, du, t, 'linear', 'extrap')';
[~, xr] = ode45(@(t, x) hydrofoil_nl(x, ufun(t), [0 0], p), tg, zeros(4,1), ...
                odeset('MaxStep', 0.005, 'RelTol', 1e-8, 'AbsTol', 1e-10));
xs = interp1(out.x_nl.Time, tsd(out.x_nl), tg);
xl = interp1(out.x_lin.Time, tsd(out.x_lin), tg);
fprintf('Check 1 (step, ideal actuators):\n');
fprintf('  Simulink nonlinear vs ode45:  max |dh| = %.2e m, max |dtheta| = %.2e rad\n', ...
        max(abs(xs(:,1)-xr(:,1))), max(abs(xs(:,3)-xr(:,3))));
fprintf('  Simulink linear vs nonlinear: max |dh| = %.2e m (h reaches %.3f m)\n', ...
        max(abs(xl(:,1)-xs(:,1))), xs(end,1));

%% Check 2: waves on, ideal actuators, no command
Tstop = 4;  act_on = 0;  wave_on = 1;  noise_on = 0;
u_cmd_ts = timeseries(0*du, tg);
out = sim(mdl);
wfun = @(t) wave_eval(wv, t, p);
[~, xr] = ode45(@(t, x) hydrofoil_nl(x, u0, wfun(t), p), tg, zeros(4,1), ...
                odeset('MaxStep', 0.005, 'RelTol', 1e-8, 'AbsTol', 1e-10));
xs = interp1(out.x_nl.Time, tsd(out.x_nl), tg);
fprintf('Check 2 (waves):\n');
fprintf('  Simulink nonlinear vs ode45:  max |dh| = %.2e m, max |dtheta| = %.2e rad\n', ...
        max(abs(xs(:,1)-xr(:,1))), max(abs(xs(:,3)-xr(:,3))));
xl  = interp1(out.x_lin.Time, tsd(out.x_lin), tg);
xlr = lsim(ss(A, E, eye(4), 0), wfun(tg), tg);          % linear reference
fprintf('  Simulink linear vs lsim:      max |dh| = %.2e m, max |dtheta| = %.2e rad\n', ...
        max(abs(xl(:,1)-xlr(:,1))), max(abs(xl(:,3)-xlr(:,3))));

%% Demonstration: real actuators, waves and sensor noise, front-foil pulse
Tstop = 3;  act_on = 1;  wave_on = 1;  noise_on = 1;  % controller still off
tg = (0:0.005:Tstop)';
du = [-1*d2r*(tg >= 1 & tg < 1.2), 0*tg];        % 1 deg down for 0.2 s
u_cmd_ts = timeseries(du, tg);
out = sim(mdl);

xn = tsd(out.x_nl);  xl = tsd(out.x_lin);  dl = tsd(out.delta);
figure('Position', [50 50 1100 300]);
subplot(1,3,1);
plot(out.delta.Time, dl(:,1)/d2r, tg, (u0(1) + du(:,1))/d2r, '--'); grid on;
xlabel('time [s]'); ylabel('front incidence \delta_f [deg]');
legend('applied', 'commanded', 'Location', 'southeast');
subplot(1,3,2);
plot(out.x_nl.Time, 100*xn(:,1), out.x_lin.Time, 100*xl(:,1), '--'); grid on;
xlabel('time [s]'); ylabel('heave h [cm]'); legend('nonlinear', 'linear', 'Location', 'southwest');
subplot(1,3,3);
y_true = xn(:,1) + p.ls*sin(xn(:,3));
ym = tsd(out.y_meas);                            % columns: bow height, pitch angle, pitch rate
plot(out.y_meas.Time, 100*ym(:,1), out.x_nl.Time, 100*y_true); grid on;
xlabel('time [s]'); ylabel('bow height y [cm]');
legend('measured', 'true', 'Location', 'southwest');
saveas(gcf, fullfile(here, 'figs', 'fig6_simulink_demo.png'));

%% Check 3: closed loop, waves on, no noise; compare with sim_digital.m
ctrl_on = 1;  act_on = 1;  wave_on = 1;  noise_on = 0;  Tstop = 6;
tg = (0:0.005:Tstop)';
r_ts = timeseries(0.05*(tg >= 1), tg);
u_cmd_ts = timeseries(zeros(2, 2), [0; Tstop]);
out = sim(mdl);
ref = @(t) interp1(tg, 0.05*(tg >= 1), t, 'linear', 'extrap');
o = sim_digital(p, p, dc, ref, Tstop, struct('delay', 1, 'wave', wv, 'noise', false));
tc = o.t(o.t <= Tstop - 0.05);
xs = interp1(out.x_nl.Time, tsd(out.x_nl), tc);   xo = interp1(o.t, o.x, tc);
ds = interp1(out.delta.Time, tsd(out.delta), tc); do = interp1(o.t, o.delta, tc);
fprintf('Check 3 (closed loop, waves):\n');
fprintf('  Simulink vs sim_digital:      max |dh| = %.2e m, max |dtheta| = %.2e rad, max |d delta| = %.2e rad\n', ...
        max(abs(xs(:,1)-xo(:,1))), max(abs(xs(:,3)-xo(:,3))), max(abs(ds(:)-do(:))));

%% Closed-loop demonstration: 5 cm step, waves and sensor noise
noise_on = 1;  Tstop = 10;
tg = (0:0.005:Tstop)';
r_ts = timeseries(0.05*(tg >= 1), tg);
out = sim(mdl);
xn = tsd(out.x_nl);  dl = tsd(out.delta);  ym = tsd(out.y_meas);
figure('Position', [50 50 1100 300]);
subplot(1,3,1);
plot(out.y_meas.Time, 100*ym(:,1), 'Color', [0.75 0.75 0.75]); hold on;
plot(out.x_nl.Time, 100*xn(:,1)); plot(tg, 100*0.05*(tg >= 1), 'k:'); grid on;
xlabel('time [s]'); ylabel('height [cm]');
legend('measured bow height', 'true heave h', 'reference', 'Location', 'southeast');
subplot(1,3,2); plot(out.x_nl.Time, xn(:,3)/d2r); grid on;
xlabel('time [s]'); ylabel('pitch \theta [deg]');
subplot(1,3,3); plot(out.delta.Time, (dl - u0(:)')/d2r); grid on;
xlabel('time [s]'); ylabel('foil deflection from trim [deg]'); legend('front', 'rear');
saveas(gcf, fullfile(here, 'figs', 'fig14_simulink_closed_loop.png'));
k = out.x_nl.Time > 3;
fprintf('Closed-loop demo after 3 s: h - 5 cm %.2f cm rms, theta %.3f deg rms\n', ...
        100*rms(xn(k,1) - 0.05), rms(xn(k,3))/d2r);

fprintf('Done.\n');

%% ------------------------------------------------------------------------
function d = tsd(ts)
%TSD  Logged data as an N-by-n matrix (matrix signals are logged as n-by-1-by-N).
d = squeeze(ts.Data);
if size(d, 1) ~= numel(ts.Time), d = d.'; end
end
