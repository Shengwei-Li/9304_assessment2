%% RUN_DIGITAL  The controller as it would run on the boat's microcontroller.
%   1. Why the continuous design cannot simply be run at 50 Hz (emulation)
%   2. Direct digital design: poles compared with the continuous design
%   3. Continuous vs digital, with and without a one-sample computation delay
%   4. Sample-rate sweep
%   5. Robustness of the digital loop to speed and mass errors
%   Figures fig15 to fig17 are saved in ./figs

clear; close all; clc;
p = params();
[A, B, E, C] = hydrofoil_lin(p);
[u0, ~] = hydrofoil_trim(p);
ctrl = design_lqr(p);
obs  = design_kf(p);
figdir = fullfile(fileparts(mfilename('fullpath')), 'figs');
set(groot, 'defaultLineLineWidth', 1.2, 'defaultAxesFontSize', 10);
d2r = pi/180;
Ts  = p.Ts_ctrl;
wv  = wave_params(p);
ref = @(t) 0.05*(t >= 1);

%% 1. Emulation of the continuous design
ze = eig(eye(5) + Ts*(obs.Ae - obs.L*obs.Ce));
fprintf('Fastest continuous observer pole %.0f rad/s; Nyquist at %.0f Hz is %.0f rad/s\n', ...
        min(real(obs.poles)), 1/Ts, pi/Ts);
fprintf('Continuous observer run with forward Euler at %.0f Hz: max |z| = %.2f (unstable if > 1)\n', ...
        1/Ts, max(abs(ze)));

%% 2. Direct digital design
dc0 = design_digital(p, Ts, 0);            % no computation delay assumed
dc1 = design_digital(p, Ts, 1);            % one-sample delay in the design model
eqs = @(z) sort(log(z(abs(z) > 1e-6))/Ts);
fprintf('Digital LQR, equivalent s-plane poles: %s\n', mat2str(eqs(dc0.polesK).', 3));
fprintf('Continuous LQR poles:                  %s\n', mat2str(sort(ctrl.poles).', 3));
fprintf('Digital Kalman filter, equivalent poles: %s\n', mat2str(eqs(dc0.polesL).', 3));
disp('Digital K ='); disp(dc0.K);  disp('Digital Ki ='); disp(dc0.Ki);
disp('Delay-compensated design, gain on the previous command Kd ='); disp(dc1.Kd);

%% 3. Continuous vs digital (5 cm step, waves and noise)
Tend = 10;  t = (0:0.005:Tend)';
oc = sim_closed_loop(p, p, ctrl, obs, ref, t, struct('observer', true, 'wave', wv, 'noise', true));
runs = { 'digital, no delay',                         dc0, 0; ...
         'digital, 1-sample delay (not compensated)', dc0, 1; ...
         'digital, 1-sample delay (compensated)',     dc1, 1};
res = cell(size(runs,1), 1);
fprintf('\n%-44s | h err rms | theta rms | foil peak | foil rate rms\n', 'Case (5 cm step, waves, noise)');
report = @(name, tt, x, dl) fprintf('%-44s | %6.2f cm | %6.3f deg | %6.2f deg | %6.1f deg/s\n', name, ...
    100*rms(x(tt>3,1) - 0.05), rms(x(tt>3,3))/d2r, max(abs(dl(:,1) - u0(1)))/d2r, ...
    rms(diff(dl(tt>3,1))./diff(tt(tt>3)))/d2r);
report('continuous (reference)', oc.t, oc.x, oc.delta);
for i = 1:size(runs,1)
    res{i} = sim_digital(p, p, runs{i,2}, ref, Tend, struct('delay', runs{i,3}, 'wave', wv, 'noise', true));
    report(runs{i,1}, res{i}.t, res{i}.x, res{i}.delta);
end

% calm water: step response metrics
fprintf('\nCalm-water 5 cm step:\n');
oc0 = sim_closed_loop(p, p, ctrl, obs, ref, t, struct('observer', true));
stepm = @(tt, h) stepinfo(interp1(tt, h, (1:0.005:Tend-0.05)')/0.05, (0:0.005:Tend-1.05)', 'SettlingTimeThreshold', 0.02);
si = stepm(oc0.t, oc0.x(:,1));
fprintf('  %-44s settling %.2f s, overshoot %.1f %%\n', 'continuous', si.SettlingTime, si.Overshoot);
for i = 1:size(runs,1)
    o = sim_digital(p, p, runs{i,2}, ref, Tend, struct('delay', runs{i,3}));
    si = stepm(o.t, o.x(:,1));
    fprintf('  %-44s settling %.2f s, overshoot %.1f %%\n', runs{i,1}, si.SettlingTime, si.Overshoot);
end

figure('Position', [50 50 1100 330]);
subplot(1,3,1); hold on; grid on;
plot(oc.t, 100*oc.x(:,1), 'k', 'DisplayName', 'continuous');
for i = [1 3], plot(res{i}.t, 100*res{i}.x(:,1), 'DisplayName', runs{i,1}); end
xlabel('time [s]'); ylabel('heave h [cm]'); legend('Location', 'southeast'); title('Height');
subplot(1,3,2); hold on; grid on;
k = res{3}.t >= 4 & res{3}.t <= 4.5;  kk = res{3}.tk >= 4 & res{3}.tk <= 4.5;
plot(res{3}.t(k), res{3}.delta(k,1)/d2r, 'DisplayName', 'applied (servo)');
stairs(res{3}.tk(kk), res{3}.ucmd(kk,1)/d2r, 'DisplayName', 'command (50 Hz)');
xlabel('time [s]'); ylabel('front foil angle [deg]'); legend('Location', 'best');
title('Zoom: 50 Hz command and servo');
subplot(1,3,3); hold on; grid on;
k = res{3}.tk >= 4 & res{3}.tk <= 6;
plot(res{3}.tk(k), 100*res{3}.y(k,1), '.', 'Color', [0.7 0.7 0.7], 'DisplayName', 'bow height samples');
kc = res{3}.t >= 4 & res{3}.t <= 6;
plot(res{3}.t(kc), 100*res{3}.x(kc,1), 'DisplayName', 'true h');
stairs(res{3}.tk(k), 100*res{3}.xhat(k,1), 'DisplayName', 'estimate (50 Hz)');
xlabel('time [s]'); ylabel('height [cm]'); legend('Location', 'best'); title('Zoom: samples and estimate');
saveas(gcf, fullfile(figdir, 'fig15_digital_vs_continuous.png'));

%% 4. Sample-rate sweep (delay-compensated design, one-sample delay)
rates = [100 50 25 20 15 10];
fprintf('\nSample-rate sweep (5 cm step, waves, noise, one-sample delay, compensated design):\n');
fprintf('%6s | %8s | %8s | %9s | %9s | %s\n', 'rate', 'h err', 'theta', 'foil pk', 'max|z|', 'note');
hrms = nan(size(rates));  fpk = hrms;
for i = 1:numel(rates)
    pr = p;  pr.Ts_sens = 1/rates(i);  pr.Ts_ctrl = 1/rates(i);
    dcr = design_digital(pr, 1/rates(i), 1);
    o = sim_digital(pr, pr, dcr, ref, Tend, struct('delay', 1, 'wave', wv, 'noise', true));
    zmax = max(abs([dcr.polesK; dcr.polesL]));
    if o.stopped, note = 'left valid range'; else, note = ''; end
    hrms(i) = 100*rms(o.x(o.t>3,1) - 0.05);  fpk(i) = max(abs(o.delta(:,1) - u0(1)))/d2r;
    fprintf('%4d Hz | %5.2f cm | %5.3f deg | %6.2f deg | %8.3f | %s\n', rates(i), hrms(i), ...
            rms(o.x(o.t>3,3))/d2r, fpk(i), zmax, note);
end
figure('Position', [50 50 800 300]);
subplot(1,2,1); plot(rates, hrms, 'o-'); grid on; set(gca, 'XDir', 'reverse');
xlabel('controller rate [Hz]'); ylabel('height error after 3 s [cm rms]');
subplot(1,2,2); plot(rates, fpk, 'o-'); grid on; set(gca, 'XDir', 'reverse'); hold on;
yline((p.u_max(1) - u0(1))/d2r, ':', 'upward margin');
xlabel('controller rate [Hz]'); ylabel('peak front foil deflection [deg]');
saveas(gcf, fullfile(figdir, 'fig16_sample_rate.png'));

%% 5. Robustness of the digital loop (50 Hz, delay-compensated)
% (a) linear: closed loop of the sampled true plant with the nominal digital controller
worst = 0;
for U = 5.9:0.1:9
    for m = 190:5:240
        pt = p; pt.U = U; pt.m = m;
        [At, Bt] = hydrofoil_lin(pt);
        sd = c2d(ss(At, Bt, C, 0), Ts, 'zoh');
        worst = max(worst, max(abs(eig(cl_digital(sd.A, sd.B, C, dc1)))));
    end
end
fprintf('\nLinear check, 50 Hz with delay, U = 5.9-9 m/s, m = 190-240 kg: max |z| = %.4f (stable if < 1)\n', worst);
% (b) nonlinear: speed and mass errors
figure('Position', [50 50 800 300]);
subplot(1,2,1); hold on; grid on;
for U = [6 7 8]
    pt = p; pt.U = U;
    o = sim_digital(pt, p, dc1, ref, 12, struct('delay', 1));
    plot(o.t, 100*o.x(:,1), 'DisplayName', sprintf('U = %d m/s', U));
    fprintf('U = %d m/s: final h %.3f cm, theta %.4f deg\n', U, 100*o.x(end,1), o.x(end,3)/d2r);
end
xlabel('time [s]'); ylabel('heave h [cm]'); title('Speed error, digital controller'); legend('Location', 'southeast');
subplot(1,2,2); hold on; grid on;
for m = [195 215 235]
    pt = p; pt.m = m;
    o = sim_digital(pt, p, dc1, ref, 12, struct('delay', 1));
    plot(o.t, 100*o.x(:,1), 'DisplayName', sprintf('m = %d kg', m));
    fprintf('m = %d kg: final h %.3f cm, theta %.4f deg\n', m, 100*o.x(end,1), o.x(end,3)/d2r);
end
xlabel('time [s]'); ylabel('heave h [cm]'); title('Mass error, digital controller'); legend('Location', 'southeast');
saveas(gcf, fullfile(figdir, 'fig17_digital_robustness.png'));
fprintf('Done.\n');

%% ------------------------------------------------------------------------
function Acl = cl_digital(Ad, Bd, C, dc)
%CL_DIGITAL  Linear closed loop: sampled plant + digital controller with delay.
%   State z = [x (4); xp (5, predicted estimate); xi (2); up (2, previous command)],
%   reference zero. The encoder reading is approximated by the held command.
n  = 5;  Sx = [eye(4), zeros(4,1)];  e1 = [1 0 0 0 0];
Ix = eye(n) - dc.M*dc.Ce;           % xh = Ix*xp + M*C*x
Acl = [Ad,                    zeros(4,n),          zeros(4,2),  Bd;
       dc.Ade*dc.M*C,         dc.Ade*Ix,           zeros(n,2),  dc.Bde;
       dc.Ts*[e1*dc.M*C; C(2,:)], dc.Ts*[e1*Ix; zeros(1,n)], eye(2), zeros(2,2);
       -dc.K*Sx*dc.M*C,       -dc.K*Sx*Ix,         -dc.Ki,      -dc.Kd];
end
