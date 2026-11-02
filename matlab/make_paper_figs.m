%% MAKE_PAPER_FIGS  Figures for the IEEE paper (paper/figs/*.pdf).
%   Column-width vector figures in the paper's font. Uses the same functions
%   as the run_ scripts; the observer comparison and the software-in-the-loop
%   figure read the results saved by run_observer_compare.m and run_sil.m.

clear; close all; clc;
here = fileparts(mfilename('fullpath'));
out  = fullfile(here, '..', 'paper', 'figs');
if ~exist(out, 'dir'), mkdir(out); end
set(groot, 'defaultAxesFontName', 'Times New Roman', 'defaultTextFontName', 'Times New Roman', ...
    'defaultAxesFontSize', 8, 'defaultLegendFontSize', 7, 'defaultLineLineWidth', 0.9, ...
    'defaultAxesLineWidth', 0.5, 'defaultAxesTickDir', 'out', 'defaultAxesBox', 'off', ...
    'defaultAxesXGrid', 'on', 'defaultAxesYGrid', 'on', 'defaultAxesGridAlpha', 0.12, ...
    'defaultLegendBox', 'off', 'defaultAxesTitleFontWeight', 'normal');
W = 8.9;                                          % IEEE column width [cm]
cB = [0.12 0.37 0.75];  cO = [0.88 0.40 0.11];  cG = [0.62 0.65 0.69];  cK = [0.10 0.10 0.10];
cT = [0.07 0.54 0.45];  cR = [0.78 0.16 0.16];  cP = [0.48 0.25 0.75];
d2r = pi/180;

p  = params();
[A, B, ~, ~, ~, u0] = hydrofoil_lin(p);
dc = design_digital(p, p.Ts_ctrl, 1);
ref = @(t) 0.05*(t >= 1);

%% Fig. 2: open loop, linear vs nonlinear, 1 deg front-foil pulse for 0.5 s
Tend = 8;  t = (0:0.005:Tend)';
du = @(tq) [1*d2r*(tq >= 0.5 & tq < 1.0); 0];
[~, xn] = ode45(@(tq, x) hydrofoil_nl(x, u0(:) + du(tq), [0; 0], p), t, zeros(4,1), ...
                odeset('RelTol', 1e-8, 'MaxStep', 0.005));
U = cell2mat(arrayfun(du, t', 'UniformOutput', false))';
xl = lsim(ss(A, B, eye(4), 0), U, t);
f = figure('Units', 'centimeters', 'Position', [2 2 W 4.1]);
tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; plot(t, 100*xn(:,1), 'Color', cB); hold on; plot(t, 100*xl(:,1), '--', 'Color', cO);
ylabel('h (cm)'); xticklabels([]);
lg = leg('nonlinear', 'linear', 'NumColumns', 2); lg.Layout.Tile = 'north';
nexttile; plot(t, xn(:,3)/d2r, 'Color', cB); hold on; plot(t, xl(:,3)/d2r, '--', 'Color', cO);
ylabel('\theta (deg)'); xlabel('time (s)');
save_fig(f, out, 'fig_openloop');

%% Fig. 4: closed loop, 5 cm step in regular waves with sensor noise
o = sim_digital(p, p, dc, ref, 10, struct('delay', 1, 'wave', wave_params(p), 'noise', true, 'seed', 1));
yh = o.y(:,1) - p.ls*sin(o.y(:,2));               % height from the bow sensor and the AHRS
f = figure('Units', 'centimeters', 'Position', [2 2 W 6.0]);
tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; plot(o.tk, 100*yh, '.', 'Color', cG, 'MarkerSize', 3); hold on;
plot(o.t, 100*o.x(:,1), 'Color', cB, 'LineWidth', 1.1);
plot(o.tk, 100*o.xhat(:,1), 'Color', cO);
plot([0 1 1 10], [0 0 5 5], '--', 'Color', cK, 'LineWidth', 0.7);
ylabel('h (cm)'); ylim([-12 16]); xticklabels([]);
lg = leg('bow sensor', 'true', 'estimate', 'reference', 'NumColumns', 4); lg.Layout.Tile = 'north';
nexttile; plot(o.tk, o.y(:,2)/d2r, '.', 'Color', cG, 'MarkerSize', 3); hold on;
plot(o.t, o.x(:,3)/d2r, 'Color', cB, 'LineWidth', 1.1); plot(o.tk, o.xhat(:,3)/d2r, 'Color', cO);
ylabel('\theta (deg)'); ylim([-1.5 1.5]); xticklabels([]);
nexttile; plot(o.t, o.delta(:,1)/d2r, 'Color', cB); hold on; plot(o.t, o.delta(:,2)/d2r, 'Color', cT);
yline(p.u_max(1)/d2r, ':', 'Color', cR); yline(p.u_min(1)/d2r, ':', 'Color', cR);
ylabel('\delta (deg)'); xlabel('time (s)'); ylim([-4 8]);
save_fig(f, out, 'fig_closedloop');

%% Fig. 5: observer comparison
S = load(fullfile(here, 'figs', 'observer_compare.mat'));
f = figure('Units', 'centimeters', 'Position', [2 2 W 5.0]);
tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; ok = S.R(1).wave;  op = S.R(2).wave;
k = ok.t >= 4 & ok.t <= 7;  kk = ok.tk >= 4 & ok.tk <= 7;  kp = op.tk >= 4 & op.tk <= 7;
plot(ok.t(k), 100*ok.x(k,1), 'Color', cB, 'LineWidth', 1.1); hold on;
plot(ok.tk(kk), 100*ok.xhat(kk,1), 'Color', cO);
plot(op.tk(kp), 100*op.xhat(kp,1), 'Color', cP);
ylabel('h (cm)'); ylim([-15 25]);
text(4.03, -14, '(a) regular waves', 'FontSize', 8, 'VerticalAlignment', 'bottom');
lg = leg('true', 'Kalman estimate', 'PP-3x estimate', 'NumColumns', 3); lg.Layout.Tile = 'north';
xlabel('time (s)');
nexttile; cols = {cO, cP, cT, cG}; sty = {'-', '-', '-', '--'};
for j = 1:numel(S.R)
    os = S.R(j).speed;
    plot(os.t, 100*(os.x(:,1) - interp1(S.nom.t, S.nom.x(:,1), os.t)), sty{j}, 'Color', cols{j}); hold on;
end
ylabel('\Delta h (cm)'); xlabel('time (s)'); xlim([0 12]); ylim([-45 5]);
text(0.2, -42, '(b) true speed 6 m/s', 'FontSize', 8, 'VerticalAlignment', 'bottom');
leg(S.names{:}, 'NumColumns', 2, 'Location', 'southeast');
save_fig(f, out, 'fig_observers');

%% Fig. 6: speed and mass errors, calm water
cases = {'U = 6 m/s', 'U', 6; 'U = 8 m/s', 'U', 8; 'm = 195 kg', 'm', 195; 'm = 235 kg', 'm', 235};
cols = {cR, cB, cT, cO};
f = figure('Units', 'centimeters', 'Position', [2 2 W 4.3]);
tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = nexttile; ax2 = nexttile; hold(ax1, 'on'); hold(ax2, 'on');
on = sim_digital(p, p, dc, ref, 15, struct('delay', 1));
plot(ax1, on.t, 100*on.x(:,1), 'Color', cK, 'LineWidth', 1.1);
for j = 1:size(cases, 1)
    pt = p;  pt.(cases{j,2}) = cases{j,3};
    os = sim_digital(pt, p, dc, ref, 15, struct('delay', 1));
    plot(ax1, os.t, 100*os.x(:,1), 'Color', cols{j});
    plot(ax2, os.tk, os.xhat(:,5), 'Color', cols{j});
end
ylabel(ax1, 'h (cm)'); xticklabels(ax1, []); ylim(ax1, [-18 14]);
lg = leg(ax1, ['nominal', cases(:,1)'], 'NumColumns', 5); lg.Layout.Tile = 'north';
ylabel(ax2, '$\hat b$ (m/s$^2$)', 'Interpreter', 'latex'); xlabel(ax2, 'time (s)');
save_fig(f, out, 'fig_robust');

%% Fig. 7: software in the loop
S = load(fullfile(here, 'figs', 'sil_results.mat'));
f = figure('Units', 'centimeters', 'Position', [2 2 W 4.9]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile([1 2]);
plot(S.nominal(:,1), 100*S.nominal(:,2), 'Color', cK, 'LineWidth', 1.8); hold on;
cl = {cO, cT, cP};
for j = 1:numel(S.traces), plot(S.traces{j}(:,1), 100*S.traces{j}(:,2), 'Color', cl{j}); end
ylabel('h (cm)'); ylim([-12 12]);
lg = leg('in model', '0 ms', '30 ms', '50 ms', 'NumColumns', 4); lg.Layout.Tile = 'north';
nexttile; plot(S.extra, 100*S.res(:,1), 'o-', 'Color', cB, 'MarkerSize', 3); hold on;
yline(100*S.herr0, '--', 'Color', cK); xline(20, ':', 'Color', cR);
ylabel('h error rms (cm)'); xlabel('computing time (ms)');
nexttile; plot(S.extra, 100*S.res(:,2), 'o-', 'Color', cB, 'MarkerSize', 3); xline(20, ':', 'Color', cR);
ylabel('late replies (%)'); xlabel('computing time (ms)'); ylim([-5 105]);
save_fig(f, out, 'fig_sil');

%% Fig. 8: irregular sea S2
wv = wave_irregular(p, 0.15, 3.0, 1);
o = sim_digital(p, p, dc, @(t) 0, 30, struct('delay', 1, 'wave', wv, 'noise', true, 'seed', 1));
[~, zeta] = wave_eval(wv, o.tk, p);
f = figure('Units', 'centimeters', 'Position', [2 2 W 4.2]);
tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; plot(o.tk, 100*zeta, 'Color', cG); hold on;
plot(o.t, 100*o.x(:,1), 'Color', cB, 'LineWidth', 1.1); plot(o.tk, 100*o.xhat(:,1), 'Color', cO);
ylabel('(cm)'); xticklabels([]); xlim([5 30]); ylim([-20 20]);
lg = leg('wave under sensor', 'h true', 'h estimate', 'NumColumns', 3); lg.Layout.Tile = 'north';
nexttile; plot(o.t, (o.delta(:,1) - u0(1))/d2r, 'Color', cB); hold on;
plot(o.t, (o.delta(:,2) - u0(2))/d2r, 'Color', cT);
yline((p.u_max(1) - u0(1))/d2r, ':', 'Color', cR);
ylabel('\delta - \delta_0 (deg)'); xlabel('time (s)'); xlim([5 30]); ylim([-5 5]);
save_fig(f, out, 'fig_irregular');

%% Fig. 9: convergence of the estimation error from a wrong initial estimate
% The boat starts 5 cm high and 1 deg bow-up, the estimate at zero; regular
% waves and sensor noise, reference zero.
x0 = [0.05; 0; 1*d2r; 0];
o = sim_digital(p, p, dc, @(t) 0, 10, struct('delay', 1, 'wave', wave_params(p), 'noise', true, ...
                'seed', 2, 'x0', x0));
kv = o.tk < o.t(end);  o.tk = o.tk(kv);  o.xhat = o.xhat(kv,:);
xt = interp1(o.t, o.x, o.tk);  ee = xt - o.xhat(:,1:4);
f = figure('Units', 'centimeters', 'Position', [2 2 W 4.0]);
tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; plot(o.tk, 100*ee(:,1), 'Color', cB); hold on; plot(o.tk, 10*ee(:,2), 'Color', cO);
ylabel('error'); xticklabels([]); ylim([-4 6]);
lg = leg('h (cm)', 'v (10 cm/s)', 'NumColumns', 2, 'Location', 'northeast');
nexttile; plot(o.tk, ee(:,3)/d2r, 'Color', cB); hold on; plot(o.tk, 0.1*ee(:,4)/d2r, 'Color', cO);
ylabel('error'); xlabel('time (s)'); ylim([-0.6 1.2]);
lg = leg('\theta (deg)', 'q (10 deg/s)', 'NumColumns', 2, 'Location', 'northeast');
save_fig(f, out, 'fig_estimation');
k = o.tk > 3;
fprintf('Estimation error rms after 3 s: h %.2f cm, v %.2f cm/s, theta %.3f deg, q %.3f deg/s\n', ...
        100*rms(ee(k,1)), 100*rms(ee(k,2)), rms(ee(k,3))/d2r, rms(ee(k,4))/d2r);
fprintf('Figures written to %s\n', out);

function l = leg(varargin)
l = legend(varargin{:});
l.ItemTokenSize = [12 8];
end

function save_fig(f, out, name)
exportgraphics(f, fullfile(out, [name '.pdf']), 'ContentType', 'vector');
exportgraphics(f, fullfile(out, [name '.png']), 'Resolution', 200);   % for checking
end
