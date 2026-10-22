function [u0, L0] = hydrofoil_trim(p)
%HYDROFOIL_TRIM  Level-flight trim (x = 0, calm water) at speed p.U.
%   u0 = trim incidence [front, rear] in rad, L0 = trim lift per foil group [N].

L0 = p.m*p.g*[-p.x(2), p.x(1)]/(p.x(1) - p.x(2));   % force and moment balance
xi = p.d0/p.c;
K0 = (1 + 16*xi^2)/(2 + 16*xi^2);
u0 = (L0./(0.5*p.rho*p.U^2*p.S*K0) - p.CL0)/p.CLa;
end
