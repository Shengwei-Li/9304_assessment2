%% RUN_OBSERVER  Kalman filter + LQR (output feedback) on the nonlinear boat.
%   1. Observer design: gain, poles, expected accuracy, separation principle
%   2. Estimation quality in closed loop, with waves and sensor noise
%   3. Output feedback vs full-state feedback
%   4. Robustness: speed and mass different from the design values
%   Figures fig11 to fig13 are saved in ./figs

clear; close all; clc;
p = params();
[A, B, E, C] = hydrofoil_lin(p);
[u0, ~] = hydrofoil_trim(p);
ctrl = design_lqr(p);
obs  = design_kf(p);
figdir = fullfile(fileparts(mfilename('fullpath')), 'figs');
set(groot, 'defaultLineLineWidth', 1.2, 'defaultAxesFontSize', 10);
d2r = pi/180;
wv  = wave_params(p);

%% 1. Observer design
disp('Kalman gain L (rows: h, v, theta, q, bias; columns: bow height, pitch, pitch rate) =');
disp(obs.L);
fprintf('Observer poles: %s\n', mat2str(sort(obs.poles).', 3));
sd = sqrt(diag(obs.P));
% note: P treats the wave surface as broadband noise, so it overstates the height error
fprintf('Filter covariance (conservative), 1 std: h %.2f cm, v %.1f cm/s, theta %.3f deg, q %.2f deg/s, bias %.2f m/s^2\n', ...
        100*sd(1), 100*sd(2), sd(3)/d2r, sd(4)/d2r, sd(5));

% closed-loop linear model: plant x, observer xe, integrators xi (reference zero)
function Acl = cl_matrix(At, Bt, C, ctrl, obs)
    K = ctrl.K;  Ki = ctrl.Ki;  n = size(obs.Ae, 1);
    Sx = [eye(4), zeros(4, n-4)];                 % picks x-hat out of xe
    Acl = [At,          -Bt*K*Sx,                                  -Bt*Ki;
           obs.L*C,     obs.Ae - obs.L*obs.Ce - obs.Be*K*Sx,       -obs.Be*Ki;
           zeros(1,4),  [1, zeros(1, n-1)],                        zeros(1,2);   % h-hat
           C(2,:),      zeros(1, n),                               zeros(1,2)];  % measured theta
end
Acl = cl_matrix(A, B, C, ctrl, obs);
pc = sort(eig(Acl));  pu = sort([ctrl.poles; obs.poles]);
fprintf('Separation principle: closed-loop poles = LQR poles + observer poles, max difference %.1e\n', ...
        max(abs(pc - pu)));

%% 2. Estimation in closed loop: 5 cm step, waves and noise
t = (0:0.005:10)';
opt = struct('observer', true, 'wave', wv, 'noise', true);
o = sim_closed_loop(p, p, ctrl, obs, @(t) 0.05*(t >= 1), t, opt);
k = o.t > 3;
err = o.x(k,:) - o.xhat(k,1:4);
fprintf('Estimation error after 3 s (rms): h %.2f cm, v %.1f cm/s, theta %.3f deg, q %.2f deg/s\n', ...
        100*rms(err(:,1)), 100*rms(err(:,2)), rms(err(:,3))/d2r, rms(err(:,4))/d2r);

figure('Position', [50 50 1100 520]);
lab = {'heave h [cm]', 'heave rate v [cm/s]', 'pitch \theta [deg]', 'pitch rate q [deg/s]'};
sc  = [100 100 1/d2r 1/d2r];
for i = 1:4
    subplot(2,2,i);
    if i == 1
        plot(o.t, 100*o.y(:,1), 'Color', [0.75 0.75 0.75]); hold on;
    elseif i == 3
        plot(o.t, o.y(:,2)/d2r, 'Color', [0.75 0.75 0.75]); hold on;
    elseif i == 4
        plot(o.t, o.y(:,3)/d2r, 'Color', [0.75 0.75 0.75]); hold on;
    end
    plot(o.t, sc(i)*o.x(:,i), o.t, sc(i)*o.xhat(:,i), '--'); grid on;
    ylabel(lab{i}); xlabel('time [s]');
    if i == 1, legend('measured (bow height)', 'true', 'estimate', 'Location', 'southeast');
    elseif i == 2, legend('true', 'estimate (not measured)', 'Location', 'northeast');
    else, legend('measured', 'true', 'estimate', 'Location', 'northeast'); end
end
saveas(gcf, fullfile(figdir, 'fig11_observer_estimates.png'));

%% 3. Output feedback vs full-state feedback (same step, waves and noise)
of = sim_closed_loop(p, p, ctrl, obs, @(t) 0.05*(t >= 1), t, opt);
fsopt = opt;  fsopt.observer = false;
fs = sim_closed_loop(p, p, ctrl, obs, @(t) 0.05*(t >= 1), t, fsopt);
k1 = of.t > 3;  k2 = fs.t > 3;
fprintf('After 3 s, rms of h - 5 cm: full state %.2f cm, output feedback %.2f cm\n', ...
        100*rms(fs.x(k2,1) - 0.05), 100*rms(of.x(k1,1) - 0.05));
fprintf('After 3 s, rms pitch: full state %.3f deg, output feedback %.3f deg\n', ...
        rms(fs.x(k2,3))/d2r, rms(of.x(k1,3))/d2r);
fprintf('Peak front foil deflection: full state %.2f deg, output feedback %.2f deg\n', ...
        max(abs(fs.delta(:,1) - u0(1)))/d2r, max(abs(of.delta(:,1) - u0(1)))/d2r);

figure('Position', [50 50 1100 300]);
subplot(1,3,1); plot(fs.t, 100*fs.x(:,1), of.t, 100*of.x(:,1)); grid on;
xlabel('time [s]'); ylabel('heave h [cm]'); legend('full state', 'Kalman filter + LQR', 'Location', 'southeast');
subplot(1,3,2); plot(fs.t, fs.x(:,3)/d2r, of.t, of.x(:,3)/d2r); grid on;
xlabel('time [s]'); ylabel('pitch \theta [deg]');
subplot(1,3,3); plot(fs.t, (fs.delta(:,1) - u0(1))/d2r, of.t, (of.delta(:,1) - u0(1))/d2r); grid on;
xlabel('time [s]'); ylabel('front foil deflection [deg]');
saveas(gcf, fullfile(figdir, 'fig12_output_vs_full_state.png'));

%% 4. Robustness of the output-feedback loop
Us = 5.9:0.1:9;  ms = 190:5:240;  worst = -inf;
for U = Us
    for m = ms
        pt = p; pt.U = U; pt.m = m;
        [At, Bt] = hydrofoil_lin(pt);
        worst = max(worst, max(real(eig(cl_matrix(At, Bt, C, ctrl, obs)))));
    end
end
fprintf('Linear check with observer, U = %.1f-%.1f m/s, m = %d-%d kg: slowest pole %.3f (stable if < 0)\n', ...
        Us(1), Us(end), ms(1), ms(end), worst);

t = (0:0.005:12)';
ropt = struct('observer', true, 'wave', [], 'noise', false);
figure('Position', [50 50 1100 300]);
subplot(1,3,1); hold on; grid on;
for U = [6 7 8]
    pt = p; pt.U = U;
    o = sim_closed_loop(pt, p, ctrl, obs, @(t) 0.05*(t >= 1), t, ropt);
    plot(o.t, 100*o.x(:,1), 'DisplayName', sprintf('U = %d m/s', U));
    fprintf('U = %d m/s: final h %.3f cm, final theta %.4f deg\n', U, 100*o.x(end,1), o.x(end,3)/d2r);
end
xlabel('time [s]'); ylabel('heave h [cm]'); title('True speed differs (design 7 m/s)');
legend('Location', 'southeast');
subplot(1,3,2); hold on; grid on;
for m = [195 215 235]
    pt = p; pt.m = m;
    o = sim_closed_loop(pt, p, ctrl, obs, @(t) 0.05*(t >= 1), t, ropt);
    plot(o.t, 100*o.x(:,1), 'DisplayName', sprintf('m = %d kg', m));
    fprintf('m = %d kg: final h %.3f cm, final theta %.4f deg\n', m, 100*o.x(end,1), o.x(end,3)/d2r);
end
xlabel('time [s]'); ylabel('heave h [cm]'); title('True mass differs (design 215 kg)');
legend('Location', 'southeast');
subplot(1,3,3); hold on; grid on;
for U = [6 7 8]
    pt = p; pt.U = U;
    o = sim_closed_loop(pt, p, ctrl, obs, @(t) 0.05*(t >= 1), t, ropt);
    plot(o.t, o.xhat(:,5), 'DisplayName', sprintf('U = %d m/s', U));
end
xlabel('time [s]'); ylabel('estimated bias b [m/s^2]'); title('Bias state absorbs the model error');
legend('Location', 'east');
saveas(gcf, fullfile(figdir, 'fig13_output_feedback_robustness.png'));
fprintf('Done.\n');
