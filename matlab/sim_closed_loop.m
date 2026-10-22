function out = sim_closed_loop(ptrue, pdes, ctrl, obs, ref, tgrid, opt)
%SIM_CLOSED_LOOP  Nonlinear boat + actuators + sensors + LQR (+ observer).
%   out = sim_closed_loop(ptrue, pdes, ctrl, obs, ref, tgrid, opt)
%
%   ptrue  parameters of the simulated boat
%   pdes   parameters the controller was designed with (trim, limits)
%   ctrl   from design_lqr;  obs from design_kf (ignored if opt.observer = false)
%   ref    function handle, requested height change r(t) [m]
%   tgrid  output times [s]
%   opt    struct with fields (all optional):
%            observer   true = feed back the estimate, false = the true state (default true)
%            antiwindup true = stop integrating while the foils saturate (default true)
%            wave       [] or struct from wave_params / wave_irregular (default [])
%            noise      true = sensor noise on (default false)
%            seed       random seed for the noise (default 1)
%
%   out.t, out.x (true states), out.delta (applied foil angles),
%   out.xhat (estimates incl. bias, empty without observer),
%   out.y (measurements), out.stopped (true if the run hit a stop condition).

if nargin < 7, opt = struct(); end
def = struct('observer', true, 'antiwindup', true, 'wave', [], 'noise', false, 'seed', 1);
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(opt, f{k}), opt.(f{k}) = def.(f{k}); end
end

[u0, ~] = hydrofoil_trim(pdes);  u0 = u0(:);
K = ctrl.K;  Ki = ctrl.Ki;  Nx = ctrl.Nx;  Nu = ctrl.Nu;

% sensor noise: one sample every Ts, held in between (like a 50 Hz sensor)
rng(opt.seed);
Ts = ptrue.Ts_sens;
nN = ceil(tgrid(end)/Ts) + 2;
sig = [ptrue.sigma_y, ptrue.sigma_th, ptrue.sigma_q] * opt.noise;
noise = randn(nN, 3) .* sig;

% waves
if isempty(opt.wave)
    wfun = @(t) [0 0];  zfun = @(t) 0;
else
    wv = opt.wave;
    wfun = @(t) wave_eval(wv, t, ptrue);             % water velocity at the foils
    zfun = @(t) zeta_of(wv, t, ptrue);               % surface under the height sensor
end

ne = 0;  if opt.observer, ne = size(obs.Ae, 1); end
% state: [x(4); delta(2); xi(2); rf; xe(ne)]
s0 = [zeros(4,1); u0; 0; 0; 0; zeros(ne,1)];
odeopt = odeset('MaxStep', 0.005, 'RelTol', 1e-6, 'Events', @stopEvent);
[t, s] = ode45(@rhs, tgrid, s0, odeopt);

out.t = t;
out.x = s(:, 1:4);
out.delta = s(:, 5:6);
out.xhat = s(:, 10:9+ne);
out.y = zeros(numel(t), 3);
for k = 1:numel(t), out.y(k,:) = measure(t(k), s(k,1:4)')'; end
out.stopped = t(end) < tgrid(end) - 1e-9;

    function y = measure(tt, x)
        n = noise(floor(tt/Ts) + 1, :);
        y = [x(1) + ptrue.ls*sin(x(3)) - zfun(tt) + n(1);   % ultrasonic bow height
             x(3) + n(2);                                    % AHRS pitch angle
             x(4) + n(3)];                                   % rate gyro
    end

    function ds = rhs(tt, s)
        x = s(1:4);  dl = s(5:6);  xi = s(7:8);  rf = s(9);
        y = measure(tt, x);
        if opt.observer
            xe = s(10:9+ne);
            xfb = xe(1:4);              % feedback uses the estimate
            hint = xe(1);               % height for the integrator: estimate
        else
            xfb = x;
            hint = x(1);                % full-state case: true height
        end
        du   = Nu*rf - K*(xfb - Nx*rf) - Ki*xi;               % LQR law
        ucmd = u0 + du;
        usat = min(max(ucmd, pdes.u_min(:)), pdes.u_max(:));  % position limit
        ddot = (usat - dl)/pdes.tau_act;                      % servo lag
        ddot = min(max(ddot, -pdes.rate_max), pdes.rate_max); % rate limit
        xidot = [hint - rf; y(2)];                            % height error, AHRS pitch
        if opt.antiwindup && any(usat ~= ucmd)
            xidot = [0; 0];
        end
        rfdot = (ref(tt) - rf)/ctrl.tf;
        ds = [hydrofoil_nl(x, dl, wfun(tt), ptrue); ddot; xidot; rfdot];
        if opt.observer
            dsx = obs.Ae*xe + obs.Be*(dl - u0) + obs.L*(y - obs.Ce*xe);
            ds = [ds; dsx];
        end
    end
end

function [val, isterm, dir] = stopEvent(~, s)
val    = [s(1) + 0.40; 0.60 - s(1); 0.35 - abs(s(3))];   % hull in water, airborne, pitch
isterm = [1; 1; 1];
dir    = [0; 0; 0];
end

function z = zeta_of(wv, t, p)
[~, z] = wave_eval(wv, t, p);
end
