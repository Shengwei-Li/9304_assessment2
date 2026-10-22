function wv = wave_params(p, zeta_a, lambda)
%WAVE_PARAMS  Regular head-sea wave seen by the boat at speed p.U.
%   wv = wave_params(p)                 5 cm amplitude, 8 m wavelength
%   wv = wave_params(p, zeta_a, lambda) amplitude [m], wavelength [m]
%   Fields: zeta_a (amplitude), kw (wave number), we (encounter frequency),
%   aw (amplitude of the vertical water velocity at foil depth).
if nargin < 2, zeta_a = 0.05; end
if nargin < 3, lambda = 8; end
wv.zeta_a = zeta_a;
wv.kw = 2*pi/lambda;
wv.we = sqrt(p.g*wv.kw) + wv.kw*p.U;
wv.aw = zeta_a*sqrt(p.g*wv.kw)*exp(-wv.kw*p.d0);
end
