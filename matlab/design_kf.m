function obs = design_kf(p, tune)
%DESIGN_KF  Steady-state Kalman filter with a heave-force bias state.
%   obs = design_kf(p) designs the observer for the linear model at p.U.
%   obs = design_kf(p, tune) overrides the tuning: tune.b_rate (bias drift
%   rate) and/or tune.df (bandwidth of the wave surface signal, Hz).
%
%   Observer state xe = [h; v; theta; q; b], where b is an unknown constant
%   vertical acceleration [m/s^2]. Speed and mass errors both act like such a
%   force (front and rear lift change in proportion, so the pitching moment
%   stays balanced), and estimating b removes the bias they would otherwise
%   cause in the state estimates.
%
%       xe' = Ae*xe + Be*u~ + L*(y - Ce*xe)
%
%   y = [bow height; pitch angle; pitch rate] (deviations from trim readings),
%   u~ = applied foil angles minus trim (read from the actuator encoders).
%
%   Fields of obs: Ae, Be, Ce, L, Qn, Rn, P (error covariance), poles,
%   and Ao = Ae - L*Ce, Bo = [Be L] for an implementation as one LTI block.

[A, B, E, C] = hydrofoil_lin(p);
Ts = p.Ts_sens;
if nargin < 2, tune = struct(); end

Ae = [A, [0; 1; 0; 0]; zeros(1, 5)];
Be = [B; 0 0];
Ce = [C, zeros(3, 1)];

% process noise: wave water velocities through E, and a random walk on b
w_var  = 0.11^2/2;          % variance of the wave velocity (amplitude 0.11 m/s)
w_corr = 0.1;               % correlation time used to turn it into a white-noise intensity [s]
b_rate = 0.5;               % how fast the bias may drift [m/s^2 per sqrt(s)] (tuned: faster recovery
                            % from speed/mass errors vs. more sensitivity to long waves)
if isfield(tune, 'b_rate'), b_rate = tune.b_rate; end
G  = [E, zeros(4,1); zeros(1,2), 1];
Qn = blkdiag(w_var*w_corr*eye(2), b_rate^2);

% measurement noise: sampled white noise of variance s^2 at Ts -> intensity s^2*Ts.
% The height reading also contains the wave surface under the bow. Its variance
% (5 cm amplitude -> 0.05^2/2) is concentrated in a narrow band around the
% encounter frequency, about df = 0.25 Hz wide, so its spectral density there is
% var/(2*df), about 100 times larger than if the same variance were white. Using
% this density makes the filter rely on the model and the IMU at wave frequencies
% instead of following the water surface.
zeta_var = 0.05^2/2;        % variance of the wave surface under the sensor [m^2]
df       = 0.25;            % bandwidth of the wave surface signal [Hz] (assumed)
if isfield(tune, 'df'), df = tune.df; end
Rn = diag([p.sigma_y^2*Ts + zeta_var/(2*df), p.sigma_th^2*Ts, p.sigma_q^2*Ts]);

[L, P] = lqe(Ae, G, Ce, Qn, Rn);

obs = struct('Ae', Ae, 'Be', Be, 'Ce', Ce, 'L', L, 'Qn', Qn, 'Rn', Rn, 'P', P, ...
             'poles', eig(Ae - L*Ce), 'Ao', Ae - L*Ce, 'Bo', [Be, L]);
end
