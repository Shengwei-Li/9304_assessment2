function out = sim_digital(ptrue, pdes, dc, ref, Tend, opt)
%SIM_DIGITAL  Continuous nonlinear boat controlled by the sampled controller.
%   out = sim_digital(ptrue, pdes, dc, ref, Tend, opt)
%
%   The boat, actuators and waves are integrated in continuous time (ode45).
%   Every dc.Ts seconds the controller of design_digital runs once: it reads
%   the three sensors and the actuator encoders, updates the Kalman filter,
%   computes a new command and holds it until the next sample (zero-order hold).
%
%   ptrue  parameters of the simulated boat;  pdes  parameters used by the controller
%   dc     from design_digital;  ref  function handle r(t) [m];  Tend  end time [s]
%   opt    struct (all optional):
%            delay      1 = command applied one sample late (computation delay);
%                       defaults to dc.delay
%            wave       [] or struct from wave_params / wave_irregular (default [])
%            noise      true = sensor noise on (default false)
%            antiwindup (default true);  seed (default 1)
%
%   out.t, out.x, out.delta  continuous signals (dense)
%   out.tk, out.y, out.xhat, out.ucmd  one row per sample
%   out.stopped  true if the boat left the valid range

if nargin < 6, opt = struct(); end
def = struct('delay', dc.delay, 'wave', [], 'noise', false, 'antiwindup', true, 'seed', 1);
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(opt, f{k}), opt.(f{k}) = def.(f{k}); end
end

Ts = dc.Ts;
N  = round(Tend/Ts);
[u0, ~] = hydrofoil_trim(pdes);  u0 = u0(:);

rng(opt.seed);
noise = randn(N+1, 3) .* ([ptrue.sigma_y, ptrue.sigma_th, ptrue.sigma_q] * opt.noise);

if isempty(opt.wave)
    wfun = @(t) [0 0];  zfun = @(t) 0;
else
    wv = opt.wave;
    wfun = @(t) wave_eval(wv, t, ptrue);             % water velocity at the foils
    zfun = @(t) zeta_of(wv, t, ptrue);               % surface under the height sensor
end

% continuous state [x; delta], controller memory
s   = [zeros(4,1); u0];
xp  = zeros(5,1);          % predicted estimate x(k|k-1)
xi  = zeros(2,1);          % integrators
rf  = 0;                   % filtered reference
du_prev = zeros(2,1);      % command computed at the previous sample
d_prev  = u0;              % encoder reading at the previous sample

out.tk = (0:N)'*Ts;
out.y = nan(N+1, 3);  out.xhat = nan(N+1, 5);  out.ucmd = nan(N+1, 2);
tt = cell(N,1);  ss = cell(N,1);
odeopt = odeset('MaxStep', Ts/4, 'RelTol', 1e-6);
out.stopped = false;

for k = 0:N
    t = k*Ts;
    x = s(1:4);  dl = s(5:6);
    % --- sensors and encoders, sampled at t
    y = [x(1) + ptrue.ls*sin(x(3)) - zfun(t); x(3); x(4)] + noise(k+1,:)';
    % --- Kalman filter: predict with the mean encoder reading over the last
    %     interval, then correct with the new measurement
    if k > 0
        xp = dc.Ade*xh + dc.Bde*((d_prev + dl)/2 - u0);
    end
    xh = xp + dc.M*(y - dc.Ce*xp);
    % --- control law
    du = dc.Nu*rf - dc.K*(xh(1:4) - dc.Nx*rf) - dc.Ki*xi - dc.Kd*du_prev;
    ucmd = u0 + du;
    usat = min(max(ucmd, pdes.u_min(:)), pdes.u_max(:));
    if ~(opt.antiwindup && any(usat ~= ucmd))
        xi = xi + Ts*[xh(1) - rf; y(2)];
    end
    rf = dc.a*rf + (1 - dc.a)*ref(t);
    out.y(k+1,:) = y';  out.xhat(k+1,:) = xh';  out.ucmd(k+1,:) = ucmd';
    if k == N, break; end
    % --- command held over [t, t+Ts]; one sample late if there is a delay
    if opt.delay
        uhold = u0 + du_prev;
    else
        uhold = ucmd;
    end
    du_prev = du;  d_prev = dl;
    % --- continuous plant and actuators over one sample
    [ti, si] = ode45(@(tq, sq) plant(tq, sq, uhold), [t, t + Ts], s, odeopt);
    s = si(end, :)';
    tt{k+1} = ti(1:end-1);  ss{k+1} = si(1:end-1, :);
    if s(1) < -0.40 || s(1) > 0.60 || abs(s(3)) > 0.35
        out.stopped = true;  break;
    end
end

T = vertcat(tt{:});  S = vertcat(ss{:});
out.t = T;  out.x = S(:,1:4);  out.delta = S(:,5:6);
nk = find(~isnan(out.y(:,1)), 1, 'last');
out.tk = out.tk(1:nk);  out.y = out.y(1:nk,:);  out.xhat = out.xhat(1:nk,:);  out.ucmd = out.ucmd(1:nk,:);

    function ds = plant(tq, sq, uc)
        usq  = min(max(uc, ptrue.u_min(:)), ptrue.u_max(:));
        ddot = (usq - sq(5:6))/ptrue.tau_act;
        ddot = min(max(ddot, -ptrue.rate_max), ptrue.rate_max);
        ds = [hydrofoil_nl(sq(1:4), sq(5:6), wfun(tq), ptrue); ddot];
    end
end

function z = zeta_of(wv, t, p)
[~, z] = wave_eval(wv, t, p);
end
