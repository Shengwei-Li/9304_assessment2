function [u_fb, xh_out, du_out] = digital_controller(y, delta, r, dc, p, u0)
%DIGITAL_CONTROLLER  One step of the 50 Hz controller (Kalman filter + LQR).
%   Called once per sample by the Controller block of hydrofoil_sim.slx; it is
%   written the way it would run on the boat's microcontroller and does the
%   same as the controller loop in sim_digital.m.
%
%   y      [bow height; pitch angle; pitch rate], deviations from trim readings
%   delta  foil angles read from the actuator encoders [rad] (absolute)
%   r      requested height change [m]
%   dc     from design_digital;  p from params;  u0 trim foil angles
%   u_fb   foil-angle correction to add to the trim angles [rad]
%   xh_out current estimate [h v theta q b] (for logging and telemetry)
%   du_out command computed this sample. With dc.delay = 1 it is the one sent
%          at the next sample; ctrl_server.m returns it over the network and
%          the link itself provides that one-sample delay.
%
%   clear digital_controller   resets the internal state.

persistent xp xh xi rf du_prev d_prev started
if isempty(started)
    xp = zeros(5,1);  xh = zeros(5,1);  xi = zeros(2,1);  rf = 0;
    du_prev = zeros(2,1);  d_prev = u0(:);  started = false;
end

% 1. Kalman filter: predict with the mean encoder reading over the last
%    sample interval, then correct with the new measurements
if started
    xp = dc.Ade*xh + dc.Bde*((d_prev + delta(:))/2 - u0(:));
end
started = true;
xh = xp + dc.M*(y(:) - dc.Ce*xp);

% 2. LQR law. Kd (delay-compensated design only) acts on the previous command
%    relative to its steady-state value Nu*rf, like K acts on x - Nx*rf
du = dc.Nu*rf - dc.K*(xh(1:4) - dc.Nx*rf) - dc.Ki*xi - dc.Kd*(du_prev - dc.Nu*rf);

% 3. integrators on estimated height and measured pitch, frozen while saturated
ucmd = u0(:) + du;
usat = min(max(ucmd, p.u_min(:)), p.u_max(:));
if all(usat == ucmd)
    xi = xi + dc.Ts*[xh(1) - rf; y(2)];
end

% 4. reference prefilter
rf = dc.a*rf + (1 - dc.a)*r;

% 5. output: with a computation delay the command computed now is sent at
%    the next sample, so the previous one goes out now
if dc.delay > 0
    u_fb = du_prev;
else
    u_fb = du;
end
du_prev = du;
d_prev  = delta(:);
xh_out  = xh;
du_out  = du;
end
