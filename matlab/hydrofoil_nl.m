function [xdot, out] = hydrofoil_nl(x, u, w, p)
%HYDROFOIL_NL  Nonlinear heave/pitch dynamics of the hydrofoil boat.
%   x = [h; v; theta; q]   heave [m], heave rate [m/s], pitch [rad], pitch rate [rad/s]
%   u = [delta_f; delta_r] absolute foil incidence relative to the hull [rad]
%   w = [w_f; w_r]         vertical water velocity at each foil [m/s]
%   out.L, out.alpha, out.depth are per-foil lift, angle of attack and submergence.

h = x(1); v = x(2); th = x(3); q = x(4);
Fz = 0; My = 0;
L = zeros(1,2); alpha = zeros(1,2); depth = zeros(1,2);

for i = 1:2
    % foil position relative to the CM (X forward, Z up)
    X =  p.x(i)*cos(th) + p.z(i)*sin(th);
    Z =  p.x(i)*sin(th) - p.z(i)*cos(th);

    % foil velocity relative to the water
    ui = p.U + q*(p.z(i)*cos(th) - p.x(i)*sin(th));
    vi = v + q*X - w(i);
    gam = atan2(vi, ui);

    % angle of attack, limited at stall
    alpha(i) = u(i) + th - gam;
    a_eff = min(max(alpha(i), -p.alpha_stall), p.alpha_stall);

    % submergence and free-surface factor (no lift once the foil is out)
    depth(i) = p.d0 - h - p.x(i)*sin(th) - p.z(i)*(1 - cos(th));
    L(i) = 0.5*p.rho*(ui^2 + vi^2)*p.S(i)*Kfs(depth(i)/p.c)*(p.CL0 + p.CLa*a_eff);

    Fz = Fz + L(i)*cos(gam);
    My = My + L(i)*(X*cos(gam) + Z*sin(gam));
end

xdot = [v; Fz/p.m - p.g; q; My/p.Iy];
if nargout > 1
    out.L = L; out.alpha = alpha; out.depth = depth;
end
end

function K = Kfs(xi)
% Free-surface lift factor at high Froude number, xi = depth/chord.
if xi <= 0
    K = 0;
else
    K = (1 + 16*xi^2)/(2 + 16*xi^2);
end
end
