function wv = wave_params(p, zeta_a, lambda)
%WAVE_PARAMS  Regular head-sea wave (a single component), for wave_eval.
%   wv = wave_params(p)                 5 cm amplitude, 8 m wavelength
%   wv = wave_params(p, zeta_a, lambda) amplitude [m], wavelength [m]
%
%   Fields used by wave_eval (column vectors, one entry per component):
%     a (amplitude), om (wave frequency), k (wave number), ph (phase)
%   Extra fields for information: zeta_a, we (encounter frequency at p.U),
%   aw (vertical water-velocity amplitude at foil depth), type.
if nargin < 2, zeta_a = 0.05; end
if nargin < 3, lambda = 8; end
k  = 2*pi/lambda;
om = sqrt(p.g*k);                     % deep-water dispersion relation
wv.type = 'regular';
wv.a  = zeta_a;   wv.om = om;   wv.k = k;   wv.ph = 0;
wv.zeta_a = zeta_a;
wv.we = om + k*p.U;
wv.aw = zeta_a*om*exp(-k*p.d0);
end
