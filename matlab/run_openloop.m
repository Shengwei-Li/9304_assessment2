%% RUN_OPENLOOP  Open-loop analysis and simulation of the hydrofoil boat.
%   1. Checks of the linearisation, modes, reachability, observability
%   2. Linear vs nonlinear response to step, pulse and sine inputs
%   3. Range of validity of the linear model
%   4. Effect of actuator position and rate limits
%   5. Effect of model uncertainty (speed and mass)
%   6. Response to regular waves, with sensor noise
%   Figures are saved in ./figs

clear; close all; clc;
p = params();
[A, B, E, C, D, u0] = hydrofoil_lin(p);
figdir = fullfile(fileparts(mfilename('fullpath')), 'figs');
if ~exist(figdir, 'dir'), mkdir(figdir); end
set(groot, 'defaultLineLineWidth', 1.3, 'defaultAxesFontSize', 10);
d2r = pi/180;

%% 1. Checks of the linear model
fprintf('Trim incidence at %.1f m/s: front %.2f deg, rear %.2f deg\n', p.U, u0/d2r);
fprintf('Residual of nonlinear model at trim: %.1e\n', ...
        norm(hydrofoil_nl(zeros(4,1), u0, [0 0], p)));

% central-difference Jacobian of the nonlinear model
An = zeros(4); Bn = zeros(4,2); En = zeros(4,2); ep = 1e-6;
for i = 1:4
    e = zeros(4,1); e(i) = ep;
    An(:,i) = (hydrofoil_nl(e, u0, [0 0], p) - hydrofoil_nl(-e, u0, [0 0], p))/(2*ep);
end
for i = 1:2
    e = zeros(1,2); e(i) = ep;
    Bn(:,i) = (hydrofoil_nl(zeros(4,1), u0+e, [0 0], p) - hydrofoil_nl(zeros(4,1), u0-e, [0 0], p))/(2*ep);
    En(:,i) = (hydrofoil_nl(zeros(4,1), u0,  e, p) - hydrofoil_nl(zeros(4,1), u0, -e, p))/(2*ep);
end
fprintf('Analytic vs numerical Jacobian, max error: A %.1e, B %.1e, E %.1e\n', ...
        max(abs(A-An),[],'all'), max(abs(B-Bn),[],'all'), max(abs(E-En),[],'all'));

disp('A ='); disp(A); disp('B ='); disp(B); disp('E ='); disp(E); disp('C ='); disp(C);
fprintf('Open-loop eigenvalues: %s\n', mat2str(sort(eig(A)).', 4));
fprintf('rank(ctrb): both %d, front only %d, rear only %d\n', ...
        rank(ctrb(A,B)), rank(ctrb(A,B(:,1))), rank(ctrb(A,B(:,2))));
fprintf('rank(obsv): all three sensors %d, height only %d, pitch angle only %d, pitch rate only %d\n', ...
        rank(obsv(A,C)), rank(obsv(A,C(1,:))), rank(obsv(A,C(2,:))), rank(obsv(A,C(3,:))));

sysl = ss(A, [B E], eye(4), 0);      % linear model, inputs [u~; w], outputs = states

%% 2. Linear vs nonlinear: step, pulse, sine (small inputs, ideal actuators)
t = (0:0.005:4)';
cases = { ...
  'Step: front foil +0.1 deg',        @(t) [0.1*d2r*(t>=1), 0*t]; ...
  'Pulse: rear foil +1 deg for 0.2 s', @(t) [0*t, 1*d2r*(t>=1 & t<1.2)]; ...
  'Sine: front foil 0.5 deg at 1 Hz', @(t) [0.5*d2r*sin(2*pi*(t-1)).*(t>=1), 0*t]};
nowave = @(t) [0*t, 0*t];

figure('Position', [50 50 1100 520]);
for k = 1:3
    du = cases{k,2};
    xl = lsim(sysl, [du(t), nowave(t)], t);
    [~, xn] = sim_nl(p, @(tt) u0 + du(tt), nowave, t, false);
    subplot(2,3,k);   plot(t, 100*xn(:,1), t, 100*xl(:,1), '--'); grid on;
    ylabel('heave h [cm]'); title(cases{k,1});
    if k == 1, legend('nonlinear', 'linear', 'Location', 'northwest'); end
    subplot(2,3,3+k); plot(t, xn(:,3)/d2r, t, xl(:,3)/d2r, '--'); grid on;
    ylabel('pitch \theta [deg]'); xlabel('time [s]');
end
saveas(gcf, fullfile(figdir, 'fig1_linear_vs_nonlinear.png'));

%% 3. Range of validity: front-foil steps of increasing size
t = (0:0.005:6)';
steps = [0.25 1 3];                              % deg
figure('Position', [50 50 1100 300]);
for k = 1:numel(steps)
    du = @(t) [steps(k)*d2r*(t>=1), 0*t];
    xl = lsim(sysl, [du(t), nowave(t)], t);
    [tn, xn] = sim_nl(p, @(tt) u0 + du(tt), nowave, t, false);
    subplot(1,3,k); plot(tn, 100*xn(:,1), t, 100*xl(:,1), '--'); grid on; hold on;
    yline(100*p.d0, ':', 'foils reach the surface');
    xlabel('time [s]'); ylabel('heave h [cm]'); ylim([-5 60]);
    title(sprintf('Front step +%.2g deg', steps(k)));
    if k == 1, legend('nonlinear', 'linear', 'Location', 'northwest'); end
end
saveas(gcf, fullfile(figdir, 'fig2_validity_range.png'));

%% 4. Actuator limits: a command beyond the position limit
t = (0:0.005:4)';
du = @(t) [5*d2r*(t>=1), 0*t];                   % +5 deg asked, +3.47 deg available
[t1, x1, d1] = sim_nl(p, @(tt) u0 + du(tt), nowave, t, false);
[t2, x2, d2] = sim_nl(p, @(tt) u0 + du(tt), nowave, t, true);
figure('Position', [50 50 900 300]);
subplot(1,2,1); plot(t1, d1(:,1)/d2r, t2, d2(:,1)/d2r); grid on; hold on;
yline(p.u_max(1)/d2r, ':', 'position limit');
xlabel('time [s]'); ylabel('front incidence \delta_f [deg]');
legend('ideal actuator', 'with position and rate limits', 'Location', 'southeast');
subplot(1,2,2); plot(t1, 100*x1(:,1), t2, 100*x2(:,1)); grid on;
xlabel('time [s]'); ylabel('heave h [cm]');
saveas(gcf, fullfile(figdir, 'fig3_actuator_limits.png'));

%% 5. Model uncertainty: inputs frozen at the nominal trim
t = (0:0.01:8)';
figure('Position', [50 50 900 300]);
subplot(1,2,1); hold on; grid on;
for U = [6.8 7.0 7.2]
    pp = p; pp.U = U;
    [tn, xn] = sim_nl(pp, @(tt) u0 + 0*[tt tt], nowave, t, false);
    plot(tn, 100*xn(:,1), 'DisplayName', sprintf('U = %.1f m/s', U));
end
xlabel('time [s]'); ylabel('heave h [cm]'); title('Speed error, trim inputs for 7 m/s');
legend('Location', 'northwest'); ylim([-40 40]);
subplot(1,2,2); hold on; grid on;
for m = [205 215 225]
    pp = p; pp.m = m;
    [tn, xn] = sim_nl(pp, @(tt) u0 + 0*[tt tt], nowave, t, false);
    plot(tn, 100*xn(:,1), 'DisplayName', sprintf('m = %d kg', m));
end
xlabel('time [s]'); ylabel('heave h [cm]'); title('Mass error, trim inputs for 215 kg');
legend('Location', 'northwest'); ylim([-40 40]);
saveas(gcf, fullfile(figdir, 'fig4_uncertainty.png'));

%% 6. Regular head waves and the measured output
zeta_a = 0.05;  lambda = 8;                      % wave amplitude [m], wavelength [m]
kw  = 2*pi/lambda;  w0 = sqrt(p.g*kw);           % wave number, wave frequency
we  = w0 + kw*p.U;                               % encounter frequency in head seas
aw  = zeta_a*w0*exp(-kw*p.d0);                   % vertical water velocity at foil depth
wave = @(t) [aw*sin(we*t + kw*p.x(1)), aw*sin(we*t + kw*p.x(2))];
zeta_s = @(t) zeta_a*cos(we*t + kw*p.ls);        % surface elevation under the sensor

t = (0:0.005:10)';
xl = lsim(sysl, [0*wave(t), wave(t)], t);
[~, xn] = sim_nl(p, @(tt) u0 + 0*[tt tt], wave, t, false);
rng(1);
y_true = xn(:,1) + p.ls*sin(xn(:,3));
y_meas = y_true - zeta_s(t) + p.sigma_y*randn(size(t));

fprintf('Wave: amplitude %.2f m, length %.0f m, encounter frequency %.2f rad/s (%.2f Hz)\n', ...
        zeta_a, lambda, we, we/2/pi);
figure('Position', [50 50 1100 300]);
subplot(1,3,1); plot(t, 100*xn(:,1), t, 100*xl(:,1), '--'); grid on;
xlabel('time [s]'); ylabel('heave h [cm]'); legend('nonlinear', 'linear', 'Location', 'northwest');
subplot(1,3,2); plot(t, xn(:,3)/d2r, t, xl(:,3)/d2r, '--'); grid on;
xlabel('time [s]'); ylabel('pitch \theta [deg]');
subplot(1,3,3); plot(t, 100*y_meas, t, 100*y_true); grid on;
xlabel('time [s]'); ylabel('bow height y [cm]');
legend('measured (waves + noise)', 'true h + l_s sin\theta', 'Location', 'northwest');
saveas(gcf, fullfile(figdir, 'fig5_waves_and_sensor.png'));

fprintf('Done. Figures saved in %s\n', figdir);

%% ------------------------------------------------------------------------
function [t, x, d] = sim_nl(p, ucmd, wfun, tgrid, useAct)
%SIM_NL  Simulate the nonlinear boat on the time grid tgrid.
%   ucmd(t) = commanded absolute incidences [1x2], wfun(t) = water velocities [1x2].
%   useAct = true adds actuator position limits, rate limits and a servo lag.
%   The run stops if the hull would hit the water or the boat leaves it.
%   Returns states x and the incidences d actually applied.

opts = odeset('MaxStep', 0.005, 'RelTol', 1e-6, 'Events', @stopEvent);
[~, ini] = deal([], ucmd(tgrid(1)));
[t, s] = ode45(@rhs, tgrid, [zeros(4,1); ini(:)], opts);
x = s(:,1:4);
if useAct
    d = s(:,5:6);
else
    d = ucmd(t);
end

    function ds = rhs(tt, s)
        uc = ucmd(tt);
        if useAct
            uc    = min(max(uc, p.u_min), p.u_max);                 % position limit
            ddot  = (uc(:) - s(5:6))/p.tau_act;                     % servo lag
            ddot  = min(max(ddot, -p.rate_max), p.rate_max);        % rate limit
            uNow  = s(5:6);
        else
            ddot  = zeros(2,1);
            uNow  = uc(:);
        end
        ds = [hydrofoil_nl(s(1:4), uNow, wfun(tt), p); ddot];
    end

    function [val, isterm, dir] = stopEvent(~, s)
        val    = [s(1) + 0.40;  0.60 - s(1);  0.35 - abs(s(3))];    % hull in water, airborne, pitch
        isterm = [1; 1; 1];
        dir    = [0; 0; 0];
    end
end
