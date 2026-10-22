function ctrl = design_lqr(p)
%DESIGN_LQR  LQR state feedback with integral action on the measured outputs.
%   ctrl = design_lqr(p) designs the controller for the linear model at p.U.
%
%   Control law (all quantities are deviations from trim, angles in rad):
%       u~  = Nu*rf - K*(x - Nx*rf) - Ki*xi
%       xi' = [h - rf; theta]             (integrals of height error and pitch)
%       rf' = (r - rf)/tf                 (reference prefilter)
%   r is the requested height change [m], x = [h; v; theta; q].
%
%   With the observer, h in the integrator is the estimate (the Kalman filter
%   removes the wave surface seen by the ultrasonic sensor; its bias state
%   keeps the estimate unbiased under speed and mass errors) and theta is the
%   AHRS reading. The two integrators make the height and the pitch angle
%   reach their targets exactly in steady state.
%
%   Fields of ctrl: K (2x4), Ki (2x2), Nx (4x1), Nu (2x1), Cz (2x4), tf,
%   Q, R, Aa, Ba (augmented design model), poles (closed-loop eigenvalues).

[A, B] = hydrofoil_lin(p);
d2r = pi/180;
Cz  = [1 0 0 0;                          % heave h
       0 0 1 0];                          % pitch angle theta

% augmented model with the two integrators as states 5 and 6
Aa = [A, zeros(4,2); Cz, zeros(2)];
Ba = [B; zeros(2,2)];

% Bryson's rule: weight = 1/(largest acceptable value)^2
h_max  = 0.02;          % heave error [m]
v_max  = 0.5;           % heave rate [m/s]
th_max = 2*d2r;         % pitch angle [rad]
q_max  = 10*d2r;        % pitch rate [rad/s]
qh     = 10;            % extra weight on the height integral (tuned)
qt     = 10;            % extra weight on the pitch integral (tuned)
u_max  = 3*d2r;         % foil deflection from trim [rad], margin is 3.5 deg
Q = diag([1/h_max^2, 1/v_max^2, 1/th_max^2, 1/q_max^2, qh/h_max^2, qt/(1*d2r)^2]);
R = diag([1 1]/u_max^2);

Ka = lqr(Aa, Ba, Q, R);
K  = Ka(:, 1:4);
Ki = Ka(:, 5:6);

% steady state for a height change r with level pitch: x_ss = Nx*r, u~_ss = Nu*r
N  = [A, B; Cz, zeros(2,2)] \ [zeros(4,1); 1; 0];
Nx = N(1:4);
Nu = N(5:6);

ctrl = struct('K', K, 'Ki', Ki, 'Nx', Nx, 'Nu', Nu, 'Cz', Cz, 'tf', 0.3, ...
              'Q', Q, 'R', R, 'Aa', Aa, 'Ba', Ba, 'poles', eig(Aa - Ba*Ka));
end
