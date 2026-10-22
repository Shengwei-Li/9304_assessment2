%% RUN_IRREGULAR  The 50 Hz digital controller in irregular head seas.
%   1. Three sea states from the JONSWAP spectrum: spectra and sample records
%   2. Closed loop in each sea state, five different random seas each
%   3. One run in detail: states, estimates, foil angles
%   Figures fig18 to fig19 are saved in ./figs

clear; close all; clc;
p = params();
[u0, ~] = hydrofoil_trim(p);
dc = design_digital(p, p.Ts_ctrl, 1);
figdir = fullfile(fileparts(mfilename('fullpath')), 'figs');
set(groot, 'defaultLineLineWidth', 1.2, 'defaultAxesFontSize', 10);
d2r = pi/180;

seas = struct('name', {'S1', 'S2', 'S3'}, 'Hs', {0.10, 0.15, 0.25}, 'Tp', {2.3, 3.0, 3.5});
nseed = 5;  Tend = 40;  tcut = 5;          % statistics after the first 5 s
ref = @(t) 0;                              % hold the trim height

%% 1. Spectra and sample records
figure('Position', [50 50 1000 320]);
subplot(1,2,1); hold on; grid on;
for i = 1:numel(seas)
    wv = wave_irregular(p, seas(i).Hs, seas(i).Tp, 1);
    plot(wv.om/(2*pi), wv.S*2*pi, '.-', 'DisplayName', sprintf('%s: H_s %.2f m, T_p %.1f s', seas(i).name, seas(i).Hs, seas(i).Tp));
end
xlabel('wave frequency [Hz]'); ylabel('S [m^2/Hz]'); title('JONSWAP spectra (\gamma = 3.3)');
legend('Location', 'northeast');
subplot(1,2,2); hold on; grid on;
tt = (0:0.01:30)';
for i = 1:numel(seas)
    wv = wave_irregular(p, seas(i).Hs, seas(i).Tp, 1);
    [~, z] = wave_eval(wv, tt, p);
    plot(tt, 100*z + 40*(i-2), 'DisplayName', seas(i).name);
end
xlabel('time [s]'); ylabel('surface under the bow [cm] (offset per sea)');
title('Surface seen by the boat at 7 m/s');
saveas(gcf, fullfile(figdir, 'fig18_irregular_seas.png'));

%% 2. Closed loop, five random seas per sea state
fprintf('%-4s | %-26s | %-14s | %-12s | %-12s | %s\n', 'sea', 'heave error rms / max [cm]', 'pitch rms [deg]', 'foil peak', 'saturated', 'est. h err');
res = struct();
for i = 1:numel(seas)
    hr = []; hm = []; th = []; fp = []; sat = []; eh = []; ok = true;
    for sd = 1:nseed
        wv = wave_irregular(p, seas(i).Hs, seas(i).Tp, sd);
        o = sim_digital(p, p, dc, ref, Tend, struct('delay', 1, 'wave', wv, 'noise', true, 'seed', sd));
        if o.stopped, ok = false; end
        k  = o.t > tcut;
        kk = o.tk > tcut & o.tk < o.t(end);
        hr(end+1)  = 100*rms(o.x(k,1));
        hm(end+1)  = 100*max(abs(o.x(k,1)));
        th(end+1)  = rms(o.x(k,3))/d2r;
        fp(end+1)  = max(abs(o.delta(:,1) - u0(1)))/d2r;
        sat(end+1) = 100*mean(any(o.ucmd(kk,:) >= p.u_max - 1e-9 | o.ucmd(kk,:) <= p.u_min + 1e-9, 2));
        eh(end+1)  = 100*rms(interp1(o.t, o.x(:,1), o.tk(kk)) - o.xhat(kk,1));
    end
    res(i).hr = hr; res(i).hm = hm; res(i).th = th; res(i).fp = fp; res(i).sat = sat; res(i).ok = ok;
    note = '';  if ~ok, note = '  (a run left the valid range)'; end
    fprintf('%-4s | %5.2f-%5.2f / %5.1f-%5.1f   | %5.3f-%5.3f    | %4.2f-%4.2f  | %4.1f-%4.1f %% | %4.2f-%4.2f cm%s\n', ...
            seas(i).name, min(hr), max(hr), min(hm), max(hm), min(th), max(th), min(fp), max(fp), ...
            min(sat), max(sat), min(eh), max(eh), note);
end

%% 3. One run in detail (sea state S2, seed 1)
wv = wave_irregular(p, seas(2).Hs, seas(2).Tp, 1);
o = sim_digital(p, p, dc, ref, Tend, struct('delay', 1, 'wave', wv, 'noise', true, 'seed', 1));
figure('Position', [50 50 1100 560]);
subplot(3,1,1); hold on; grid on;
plot(o.tk, 100*o.y(:,1), '.', 'Color', [0.75 0.75 0.75], 'MarkerSize', 4, 'DisplayName', 'bow height samples');
plot(o.t, 100*o.x(:,1), 'DisplayName', 'true heave');
stairs(o.tk, 100*o.xhat(:,1), 'DisplayName', 'estimate');
ylabel('height [cm]'); legend('Location', 'eastoutside'); title(sprintf('Sea state S2 (H_s %.2f m, T_p %.1f s), 50 Hz controller', seas(2).Hs, seas(2).Tp));
subplot(3,1,2); hold on; grid on;
plot(o.t, o.x(:,3)/d2r, 'DisplayName', 'true pitch');
stairs(o.tk, o.xhat(:,3)/d2r, 'DisplayName', 'estimate');
ylabel('pitch [deg]'); legend('Location', 'eastoutside');
subplot(3,1,3); hold on; grid on;
plot(o.t, (o.delta - u0(:)')/d2r);
yline((p.u_max(1) - u0(1))/d2r, ':', 'front upper limit');
ylabel('foil deflection [deg]'); xlabel('time [s]'); legend('front', 'rear', 'Location', 'eastoutside');
saveas(gcf, fullfile(figdir, 'fig19_irregular_closed_loop.png'));
fprintf('Done.\n');
