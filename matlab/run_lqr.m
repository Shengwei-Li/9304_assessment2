%% RUN_LQR  LQR state feedback, all four states assumed known (no observer yet).
%   1. Design, gains and closed-loop poles
%   2. 5 cm height step: linear vs nonlinear, with real actuators
%   3. Regular waves: open loop vs closed loop
%   4. Robustness: speed and mass different from the design values
%   5. Large step (20 cm): actuator saturation, with and without anti-windup
%   Figures fig7 to fig10 are saved in ./figs

clear; close all; clc;
p = params();
[A, B] = hydrofoil_lin(p);
[u0, ~] = hydrofoil_trim(p);
ctrl = design_lqr(p);
figdir = fullfile(fileparts(mfilename('fullpath')), 'figs');
set(groot, 'defaultLineLineWidth', 1.3, 'defaultAxesFontSize', 10);
d2r = pi/180;
fs  = struct('observer', false);                       % full-state feedback

%% 1. Design
disp('K (rows: front, rear foil; columns: h, v, theta, q) ='); disp(ctrl.K);
disp('Ki (columns: height integral, pitch integral) ='); disp(ctrl.Ki);
fprintf('Closed-loop poles: %s\n', mat2str(sort(ctrl.poles).', 3));

% linear closed loop with the prefilter; state [x; xi; rf], input r, outputs [x; u~]
K = ctrl.K; Ki = ctrl.Ki; Nx = ctrl.Nx; Nu = ctrl.Nu; tf = ctrl.tf; Cz = ctrl.Cz;
Acl = [A - B*K,    -B*Ki,      B*(Nu + K*Nx);
       Cz,          zeros(2),  [-1; 0];
       zeros(1,4),  zeros(1,2), -1/tf];
Bcl = [zeros(6,1); 1/tf];
Ccl = [eye(4), zeros(4,3); -K, -Ki, Nu + K*Nx];
syscl = ss(Acl, Bcl, Ccl, 0);

%% 2. 5 cm height step
r = 0.05;  t = (0:0.005:5)';
yl = lsim(syscl, r*ones(size(t)), t);
o  = sim_closed_loop(p, p, ctrl, [], @(t) r, t, fs);
si = stepinfo(yl(:,1)/r, t, 'SettlingTimeThreshold', 0.02);
fprintf('5 cm step (linear): settling %.2f s, overshoot %.1f %%, peak foil deflection %.2f / %.2f deg\n', ...
        si.SettlingTime, si.Overshoot, max(abs(yl(:,5)))/d2r, max(abs(yl(:,6)))/d2r);
fprintf('5 cm step (nonlinear): final h %.2f cm, peak foil deflection %.2f / %.2f deg\n', ...
        100*o.x(end,1), max(abs(o.delta(:,1) - u0(1)))/d2r, max(abs(o.delta(:,2) - u0(2)))/d2r);

figure('Position', [50 50 1100 300]);
subplot(1,3,1); plot(o.t, 100*o.x(:,1), t, 100*yl(:,1), '--'); grid on; hold on;
yline(100*r, ':'); xlabel('time [s]'); ylabel('heave h [cm]'); title('Height');
legend('nonlinear', 'linear', 'Location', 'southeast');
subplot(1,3,2); plot(o.t, o.x(:,3)/d2r, t, yl(:,3)/d2r, '--'); grid on;
xlabel('time [s]'); ylabel('pitch \theta [deg]'); title('Pitch');
subplot(1,3,3); plot(o.t, (o.delta - u0)/d2r); grid on; hold on;
yline((p.u_max(1) - u0(1))/d2r, ':', 'front upper limit');
xlabel('time [s]'); ylabel('foil deflection from trim [deg]'); title('Foil angles');
legend('front', 'rear', 'Location', 'east');
saveas(gcf, fullfile(figdir, 'fig7_lqr_step.png'));

%% 3. Regular head waves: open loop vs closed loop
wv = wave_params(p);
t = (0:0.005:10)';
wfun = @(t) [wv.aw*sin(wv.we*t + wv.kw*p.x(1)), wv.aw*sin(wv.we*t + wv.kw*p.x(2))];
[to, xo] = ode45(@(tt, x) hydrofoil_nl(x, u0, wfun(tt), p), t, zeros(4,1), odeset('MaxStep', 0.005));
fsw = fs;  fsw.wave = wv;
o = sim_closed_loop(p, p, ctrl, [], @(t) 0, t, fsw);
k = o.t > 5;                                 % ignore the start-up transient
fprintf('Waves, closed loop after 5 s: h %.2f cm rms, theta %.3f deg rms, foil %.2f / %.2f deg peak\n', ...
        100*rms(o.x(k,1)), rms(o.x(k,3))/d2r, max(abs(o.delta(k,1)-u0(1)))/d2r, max(abs(o.delta(k,2)-u0(2)))/d2r);
figure('Position', [50 50 1100 300]);
subplot(1,3,1); plot(to, 100*xo(:,1), o.t, 100*o.x(:,1)); grid on;
xlabel('time [s]'); ylabel('heave h [cm]'); legend('open loop', 'LQR', 'Location', 'northwest');
subplot(1,3,2); plot(to, xo(:,3)/d2r, o.t, o.x(:,3)/d2r); grid on;
xlabel('time [s]'); ylabel('pitch \theta [deg]');
subplot(1,3,3); plot(o.t, (o.delta - u0)/d2r); grid on;
xlabel('time [s]'); ylabel('foil deflection from trim [deg]'); legend('front', 'rear');
saveas(gcf, fullfile(figdir, 'fig8_lqr_waves.png'));

%% 4. Robustness: the plant differs from the design model
% (a) linear check over a grid: closed-loop poles of the true plant with the nominal gains
Us = 5.9:0.1:9;  ms = 190:5:240;  worst = -inf;
for U = Us
    for m = ms
        pt = p; pt.U = U; pt.m = m;
        [At, Bt] = hydrofoil_lin(pt);
        Aat = [At, zeros(4,2); Cz, zeros(2)];  Bat = [Bt; zeros(2,2)];
        worst = max(worst, max(real(eig(Aat - Bat*[K Ki]))));
    end
end
fprintf('Linear check, U = %.1f-%.1f m/s and m = %d-%d kg: slowest closed-loop pole %.3f (stable if < 0)\n', ...
        Us(1), Us(end), ms(1), ms(end), worst);

% (b) nonlinear: same controller and trim inputs, different true speed or mass
t = (0:0.005:6)';
figure('Position', [50 50 900 300]);
subplot(1,2,1); hold on; grid on;
for U = [6 7 8]
    pt = p; pt.U = U;
    o = sim_closed_loop(pt, p, ctrl, [], @(t) 0.05*(t >= 1), t, fs);
    plot(o.t, 100*o.x(:,1), 'DisplayName', sprintf('U = %d m/s', U));
end
xlabel('time [s]'); ylabel('heave h [cm]'); title('True speed differs (design 7 m/s)');
legend('Location', 'southeast');
subplot(1,2,2); hold on; grid on;
for m = [195 215 235]
    pt = p; pt.m = m;
    o = sim_closed_loop(pt, p, ctrl, [], @(t) 0.05*(t >= 1), t, fs);
    plot(o.t, 100*o.x(:,1), 'DisplayName', sprintf('m = %d kg', m));
end
xlabel('time [s]'); ylabel('heave h [cm]'); title('True mass differs (design 215 kg)');
legend('Location', 'southeast');
saveas(gcf, fullfile(figdir, 'fig9_lqr_robustness.png'));

%% 5. Large step: saturation and anti-windup
t = (0:0.005:8)';
ref = @(t) 0.20*(t >= 1);                    % 20 cm higher
ctrl_nf = ctrl;  ctrl_nf.tf = 0.01;          % no prefilter, so the foils saturate
o1 = sim_closed_loop(p, p, ctrl_nf, [], ref, t, struct('observer', false, 'antiwindup', false));
o2 = sim_closed_loop(p, p, ctrl_nf, [], ref, t, fs);
figure('Position', [50 50 900 300]);
subplot(1,2,1); plot(o1.t, 100*o1.x(:,1), o2.t, 100*o2.x(:,1)); grid on; hold on; yline(20, ':');
xlabel('time [s]'); ylabel('heave h [cm]'); legend('no anti-windup', 'with anti-windup', 'Location', 'southeast');
subplot(1,2,2); plot(o1.t, o1.delta(:,1)/d2r, o2.t, o2.delta(:,1)/d2r); grid on; hold on;
yline(p.u_max(1)/d2r, ':', 'limit'); yline(p.u_min(1)/d2r, ':', 'limit');
xlabel('time [s]'); ylabel('front foil angle [deg]');
saveas(gcf, fullfile(figdir, 'fig10_lqr_saturation.png'));
fprintf('20 cm step, overshoot: without anti-windup %.1f cm, with %.1f cm\n', ...
        100*(max(o1.x(:,1)) - 0.20), 100*(max(o2.x(:,1)) - 0.20));
fprintf('Done.\n');
