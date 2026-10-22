function [w, zeta] = wave_eval(wv, t, p)
%WAVE_EVAL  Water motion seen by the boat in head seas (linear wave theory).
%   [w, zeta] = wave_eval(wv, t, p)
%   wv  wave description from wave_params (regular) or wave_irregular
%   t   time [s], scalar or column vector
%   p   boat parameters (speed p.U, foil positions p.x, depth p.d0, sensor p.ls)
%   w     [w_front, w_rear]: upward water velocity at the foils [m/s]
%   zeta  water-surface elevation under the height sensor [m]
%
%   The wave field is a sum of components travelling towards the bow:
%       zeta(x,t) = sum a_j cos(k_j x + om_j t + ph_j)
%   with x measured forward in a frame fixed to the water. In the boat frame
%   x = x_b + U t, so each component is seen at the encounter frequency
%   om_e = om + k U. The vertical water velocity is the time derivative of the
%   surface elevation, decaying as exp(-k d) with depth d:
%       w(x,t) = - sum a_j om_j exp(-k_j d) sin(k_j x + om_e,j t + ph_j)

t  = t(:);
we = wv.om + wv.k*p.U;                                   % encounter frequencies
ph = t*we.' + ones(numel(t),1)*wv.ph.';                  % N x n
dec = wv.a.*wv.om.*exp(-wv.k*p.d0);                      % velocity amplitudes at foil depth
w = [ -sin(ph + ones(numel(t),1)*(wv.k.'*p.x(1)))*dec, ...
      -sin(ph + ones(numel(t),1)*(wv.k.'*p.x(2)))*dec ];
zeta = cos(ph + ones(numel(t),1)*(wv.k.'*p.ls))*wv.a;
end
