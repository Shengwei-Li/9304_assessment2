function [A, B, E, C, D, u0] = hydrofoil_lin(p)
%HYDROFOIL_LIN  Analytic linearisation about level-flight trim.
%   dx/dt = A x + B (u - u0) + E w,   y = C x + D (u - u0)
%   y = [bow height; pitch angle; pitch rate]
%   x = [h; v; theta; q], u = foil incidences [rad], w = water velocities [m/s].

[u0, L0] = hydrofoil_trim(p);
U  = p.U;  x = p.x;  z = p.z;
xi = p.d0/p.c;
K0 = (1 + 16*xi^2)/(2 + 16*xi^2);
dK = 32*xi/(2 + 16*xi^2)^2/p.c;            % dK/d(depth)

k   = 0.5*p.rho*U^2*p.S*K0*p.CLa;          % dL/d(alpha)   [N/rad]
kap = L0*dK/K0;                            % dL/d(depth)   [N/m]
sig = 2*L0.*z/U;                           % dL/dq via forward speed [N s/rad]
Wz  = sum(L0.*z);                          % inverted-pendulum moment [N m/rad]

A = zeros(4);
A(1,2) = 1;  A(3,4) = 1;
A(2,:) = [-sum(kap), -sum(k)/U, sum(k - kap.*x), sum(sig - k.*x/U)]/p.m;
A(4,:) = [-sum(kap.*x), -(sum(k.*x) + Wz)/U, ...
          sum(k.*x - kap.*x.^2) + Wz, -sum(k.*x.^2 - L0.*z.*x)/U]/p.Iy;

B = [0 0; k/p.m; 0 0; k.*x/p.Iy];
E = [0 0; k/p.m; 0 0; (k.*x + L0.*z)/p.Iy]/U;
C = [1 0 p.ls 0;       % ultrasonic bow height  h + ls*theta
     0 0 1    0;       % AHRS pitch angle       theta
     0 0 0    1];      % pitch-rate gyro        q
D = zeros(3, 2);
end
