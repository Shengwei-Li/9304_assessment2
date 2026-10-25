%% RUN_SIL  Software in the loop: the controller as a separate process.
%   The controller (ctrl_server.m) runs in its own MATLAB session and talks to
%   hydrofoil_sim.slx over TCP; the model is the "boat", the other process is
%   the "boat's computer". Start the controller first, in a second MATLAB:
%       ctrl_server                          (current folder matlab/; start_sil.bat does this)
%
%   1. Lockstep: the external controller must give exactly the same result as
%      the controller inside the model
%   2. Real time (Simulink pacing): how much computing time the controller may
%      take before the 20 ms deadline is missed, and what a miss costs
%   3. Random jitter: occasional late replies
%   4. Linear check: closed-loop poles with extra samples of delay
%   The live demo for the 3D view is run_sil_demo.m.
%   Figure fig20 is saved in ./figs. Sections 2 and 3 run in real time (~2.5 min).

clear; close all; clc;
mdl = 'hydrofoil_sim';
figdir = fullfile(fileparts(mfilename('fullpath')), 'figs');
set(groot, 'defaultLineLineWidth', 1.2, 'defaultAxesFontSize', 10);
tsd = @(ts) reshape(permute(ts.Data, [3 1 2]), numel(ts.Time), []);

try                                            % is the controller running?
    s_ = java.net.Socket('127.0.0.1', 5501);  s_.close();  clear s_
catch
    error(['ctrl_server is not running. Start it in a second MATLAB: ' ...
           'run ctrl_server with matlab/ as the current folder (or double-click start_sil.bat).']);
end

% default scenario: 5 cm step at 1 s, 5 cm regular waves, noise, 10 s
hydrofoil_sim_init;
load_system(mdl);
set_param(mdl, 'EnablePacing', 'off');
viz_on = 0;
herr = @(o) rms(interp1(o.x_nl.Time, tsd(o.x_nl), (2:0.005:Tstop)') * [1;0;0;0] - 0.05);

%% 1. Lockstep: external controller vs controller inside the model
ctrl_ext = 0;                  o0 = sim(mdl);
ctrl_ext = 1; sil_lockstep = 1; o1 = sim(mdl);
x0 = tsd(o0.x_nl);  x1 = tsd(o1.x_nl);  lk = tsd(o1.link);
fprintf('Lockstep vs in-model controller: max |x difference| = %.3g\n', max(abs(x0(:) - x1(:))));
fprintf('Round trip over TCP: median %.2f ms, 99%% %.2f ms; controller computing %.3f ms\n', ...
        median(lk(2:end,3)), prctile(lk(2:end,3), 99), mean(lk(2:end,2)));

%% 2. Real time: computing-time sweep
set_param(mdl, 'EnablePacing', 'on', 'PacingRate', 1);
sil_lockstep = 0;
extra = [0 10 18 22 25 30 35 40 50];
res = zeros(numel(extra), 3);                  % [rms height error, late fraction, max |h - h_ideal|]
keep = {};
for j = 1:numel(extra)
    sil_extra_ms = extra(j);  sil_jitter_ms = 0;
    o = sim(mdl);
    lk = tsd(o.link);  x = tsd(o.x_nl);
    res(j,:) = [herr(o), lk(end,1)/(size(lk,1)-1), ...
                max(abs(interp1(o.x_nl.Time, x(:,1), o0.x_nl.Time) - x0(:,1)))];
    fprintf('extra %2d ms: h error rms %.2f cm, late %3.0f%%, max deviation from ideal %.1f mm\n', ...
            extra(j), 100*res(j,1), 100*res(j,2), 1e3*res(j,3));
    if any(extra(j) == [0 30 50]), keep{end+1} = o; end %#ok<SAGROW>
end
fprintf('In-model controller: h error rms %.2f cm\n', 100*herr(o0));

%% 3. Random jitter: 10 ms plus uniform 0..20 ms (late whenever the total > 20 ms)
sil_extra_ms = 10;  sil_jitter_ms = 20;
o = sim(mdl);  lk = tsd(o.link);
jit = [100*lk(end,1)/(size(lk,1)-1), 100*herr(o)];
fprintf('Jitter 10 + U(0,20) ms: late %.0f%%, h error rms %.2f cm\n', jit(1), jit(2));
sil_extra_ms = 0;  sil_jitter_ms = 0;

%% 4. Linear check: how many extra samples of delay the loop tolerates
% Linearised plant with the servo lag, the digital controller exactly as
% implemented (no noise, no limits), command applied 1 + n samples after the
% reading it was computed from. The ~2 Hz mode is the one the delay destabilises.
p = params();  [A, B, ~, C] = hydrofoil_lin(p);
zeta2 = zeros(1, 4);
for n = 0:3
    z = sil_delay_poles(dc, p, A, B, C, n);
    zf = z(abs(angle(z))/(2*pi*dc.Ts) > 0.5);              % oscillatory modes above 0.5 Hz
    [~, i] = max(abs(zf));  s_ = log(zf(i))/dc.Ts;
    fprintf('extra delay %d samples (%2d ms): max |z| = %.4f, ~%.1f Hz mode damping ratio %.2f\n', ...
            n, 1e3*n*dc.Ts, max(abs(z)), abs(imag(s_))/(2*pi), -real(s_)/abs(s_));
    zeta2(n+1) = -real(s_)/abs(s_);
end

% data for the paper figure (make_paper_figs.m)
traces = cellfun(@(o) [o.x_nl.Time, tsd(o.x_nl)*[1;0;0;0]], keep, 'UniformOutput', false);
nominal = [o0.x_nl.Time, x0(:,1)];
herr0 = herr(o0);
save(fullfile(figdir, 'sil_results.mat'), 'extra', 'res', 'jit', 'zeta2', 'traces', 'nominal', 'herr0');

%% figure 20
f = figure('Position', [100 100 900 600]);
subplot(2,2,[1 2]);
lbl = {'0 ms (on time)', '30 ms (1.5 periods)', '50 ms (2.5 periods)'};
plot(o0.x_nl.Time, 100*x0(:,1), 'k', 'LineWidth', 2); hold on;
for j = 1:numel(keep)
    x = tsd(keep{j}.x_nl);  plot(keep{j}.x_nl.Time, 100*x(:,1));
end
grid on; xlabel('t [s]'); ylabel('h [cm]');
legend(['in-model controller', strcat('external, computing', {' '}, lbl)], 'Location', 'southeast');
title('Height, controller in a separate process (real time), 5 cm step in 5 cm waves');
subplot(2,2,3);
plot(extra, 100*res(:,1), 'o-'); hold on; yline(100*herr(o0), 'k--', 'in-model');
xline(20, 'r:', '20 ms period'); grid on;
xlabel('controller computing time [ms]'); ylabel('h error rms, t > 2 s [cm]');
subplot(2,2,4);
plot(extra, 100*res(:,2), 'o-'); xline(20, 'r:'); grid on; ylim([-5 105]);
xlabel('controller computing time [ms]'); ylabel('samples with a late reply [%]');
exportgraphics(f, fullfile(figdir, 'fig20_sil_timing.png'), 'Resolution', 150);

set_param(mdl, 'EnablePacing', 'off');

function z = sil_delay_poles(dc, p, A, B, C, n)
% Closed-loop poles: state [x; foil angle; xh; previous encoder reading; xi;
% du_prev; queue of the last n+1 commands]. Built column by column from one
% step of the loop, so it follows digital_controller.m line by line.
sd = c2d(ss([A, B; zeros(2,4), -eye(2)/p.tau_act], [zeros(4,2); eye(2)/p.tau_act], eye(6), 0), dc.Ts);
N  = 6 + 5 + 2 + 2 + 2 + 2*(n+1);
F  = zeros(N);
for j = 1:N
    s = zeros(N, 1);  s(j) = 1;
    x = s(1:6);  xh = s(7:11);  dp = s(12:13);  xi = s(14:15);  up = s(16:17);
    q = reshape(s(18:end), 2, []);
    u  = q(:, end);                                   % command applied in this sample
    y  = C*x(1:4);  d = x(5:6);                       % sensors and encoders
    xp = dc.Ade*xh + dc.Bde*((dp + d)/2);
    xh = xp + dc.M*(y - dc.Ce*xp);
    du = -dc.K*xh(1:4) - dc.Ki*xi - dc.Kd*up;
    xi = xi + dc.Ts*[xh(1); y(2)];
    q  = [du, q(:, 1:end-1)];
    F(:, j) = [sd.A*x + sd.B*u; xh; d; xi; du; q(:)];
end
z = eig(F);
end
