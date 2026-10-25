function dc = design_digital(p, Ts, delay, tune)
%DESIGN_DIGITAL  Direct digital design: discrete LQR + discrete Kalman filter.
%   dc = design_digital(p)            Ts = p.Ts_ctrl, no computation delay
%   dc = design_digital(p, Ts, delay) delay = 1 designs for a one-sample
%                                     computation delay (u computed from y[k]
%                                     is applied at k+1)
%
%   The controller runs once every Ts seconds (digital_controller.m):
%     1. predict  : xp = Ade*xh + Bde*((d_prev + delta)/2 - u0)   (encoder readings)
%     2. correct  : xh = xp + M*(y - Ce*xp)                        (current estimate)
%     3. control  : u~ = Nu*rf - K*(xh(1:4) - Nx*rf) - Ki*xi  [- Kd*(u~prev - Nu*rf)]
%     4. integrate: xi = xi + Ts*[xh(1) - rf; y(2)]                (frozen while saturated)
%     5. prefilter: rf = a*rf + (1-a)*r,  a = exp(-Ts/tf)
%   With delay = 1 the command u~ is applied one sample later, and the design
%   model includes that delay as an extra state u_prev (feedback gain Kd).
%
%   Weights and noise intensities are the same as in design_lqr / design_kf,
%   converted to discrete time, so the two designs are directly comparable.
%   The reference prefilter is slower (tf = 0.6 s instead of 0.3 s), see below.

if nargin < 2 || isempty(Ts), Ts = p.Ts_ctrl; end
if nargin < 3, delay = 0; end
if nargin < 4, tune = struct(); end   % observer tuning, passed to design_kf

[A, B] = hydrofoil_lin(p);
ctrl = design_lqr(p);              % continuous design: weights, Nx, Nu, tf
obs  = design_kf(p, tune);         % continuous observer: model and noise

% Reference prefilter: slower than in the continuous design (0.3 s). It leaves
% about 1 deg more of the front foil's upward margin for larger waves and still
% settles a 5 cm step in about 2 s (see control_design.pdf, D10).
tf = 0.6;
if isfield(tune, 'tf'), tf = tune.tf; end

%% plant discretised with zero-order hold
sysd = c2d(ss(A, B, eye(4), 0), Ts, 'zoh');
Ad = sysd.A;  Bd = sysd.B;
Cz = ctrl.Cz;

%% discrete LQR on [x; xi] (xi integrated with forward Euler, as implemented)
Aa = [Ad, zeros(4,2); Ts*Cz, eye(2)];
Ba = [Bd; zeros(2,2)];
Qd = ctrl.Q*Ts;  Rd = ctrl.R*Ts;           % cost J = sum (x'Qx + u'Ru) Ts
if delay
    % state [x; xi; u_prev]; the plant is driven by u_prev
    Aa = [Aa, Ba; zeros(2,8)];
    Ba = [zeros(6,2); eye(2)];
    Qd = blkdiag(Qd, zeros(2));
end
[Ka, ~, polesK] = dlqr(Aa, Ba, Qd, Rd);
K  = Ka(:, 1:4);
Ki = Ka(:, 5:6);
Kd = zeros(2);  if delay, Kd = Ka(:, 7:8); end

%% discrete Kalman filter (current-estimator form) for [x; b]
sysde = c2d(ss(obs.Ae, [obs.Be, eye(5)], eye(5), 0), Ts, 'zoh');
Ade = sysde.A;  Bde = sysde.B(:, 1:2);
% process noise: exact discretisation of the continuous intensity (Van Loan)
Gc = [hydrofoil_E(p), zeros(4,1); zeros(1,2), 1];
Qc = Gc*obs.Qn*Gc';
n  = 5;
F  = expm([-obs.Ae, Qc; zeros(n), obs.Ae']*Ts);
Qw = F(n+1:end, n+1:end)' * F(1:n, n+1:end);
Qw = (Qw + Qw')/2;
Rw = obs.Rn/Ts;                    % continuous intensity -> sample variance
[M, P, ~, polesL] = dlqe(Ade, eye(n), obs.Ce, Qw, Rw);

dc = struct('Ts', Ts, 'delay', delay, 'Ad', Ad, 'Bd', Bd, ...
            'K', K, 'Ki', Ki, 'Kd', Kd, 'Nx', ctrl.Nx, 'Nu', ctrl.Nu, 'Cz', Cz, ...
            'a', exp(-Ts/tf), 'tf', tf, ...
            'Ade', Ade, 'Bde', Bde, 'Ce', obs.Ce, 'M', M, 'P', P, ...
            'polesK', polesK, 'polesL', polesL);
end

function E = hydrofoil_E(p)
[~, ~, E] = hydrofoil_lin(p);
end
