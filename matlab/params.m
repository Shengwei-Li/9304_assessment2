function p = params()
%PARAMS  Parameters of the hydrofoil boat heave/pitch model.
%   All values in SI units, angles in radians. Sources:
%   [B] Belime, MSc thesis, IST Lisbon, 2020 (SR02 solar boat), Tables 2.1, 3.2
%   [est] estimated / assumed, see model_derivation.pdf

% --- environment
p.g    = 9.81;              % gravity [m/s^2]
p.rho  = 1025;              % sea-water density [kg/m^3]            [B]

% --- rigid body
p.m    = 215;               % mass of boat and pilot [kg]           [B]
p.Iy   = 503;               % pitch moment of inertia [kg m^2]      [B]

% --- foils: column 1 = front pair, column 2 = rear
p.S    = [2*0.048, 0.046];  % planform area [m^2]                   [B]
p.x    = [1.10, -2.45];     % position ahead of CM [m]              [B]
p.z    = [0.75,  0.96];     % position below CM [m]                 [B]
p.CL0  = 0.288;             % lift coefficient at zero incidence    [B]
p.CLa  = 0.090*180/pi;      % lift-curve slope [1/rad]              [B]
p.c    = 0.073;             % chord [m]                             [est]
p.d0   = 0.30;              % foil submergence at trim [m]          [est]
p.alpha_stall = deg2rad(12);% lift stops increasing beyond this     [est]

% --- operating point
p.U    = 7.0;               % forward speed [m/s]                   [B]

% --- sensor
p.ls        = 3.0;          % ultrasonic sensor ahead of CM [m]     [B]
p.sigma_y   = 0.01;         % height noise, std [m]                 [est]
p.sigma_th  = deg2rad(0.3); % AHRS pitch-angle noise, std [rad]     [est]
p.sigma_q   = deg2rad(0.2); % pitch-rate gyro noise, std [rad/s]    [est]
p.Ts_sens   = 0.02;         % sensor sample time (50 Hz) [s]        [est]
p.Ts_ctrl   = 0.02;         % controller sample time (50 Hz) [s]    [est]

% --- actuators (absolute incidence, relative to the hull)
p.u_min     = deg2rad([-3, -3]);   % lower position limit [rad]     [B]
p.u_max     = deg2rad([ 7,  7]);   % upper position limit [rad]     [B]
p.rate_max  = deg2rad(20);         % rate limit [rad/s]             [est]
p.tau_act   = 0.02;                % servo time constant [s]        [est]
end
