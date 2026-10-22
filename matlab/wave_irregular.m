function wv = wave_irregular(p, Hs, Tp, seed, gamma, n)
%WAVE_IRREGULAR  Irregular head sea from a JONSWAP spectrum, for wave_eval.
%   wv = wave_irregular(p, Hs, Tp)               seed 1, gamma 3.3, 80 components
%   wv = wave_irregular(p, Hs, Tp, seed, gamma, n)
%   Hs     significant wave height [m] (4 x standard deviation of the surface)
%   Tp     peak period [s]
%   seed   random seed for the phases and frequencies (same seed = same sea)
%   gamma  peak-enhancement factor (3.3 = standard JONSWAP, 1 = Pierson-Moskowitz)
%
%   The surface is a sum of n components with random phases. Each component
%   has amplitude a = sqrt(2*S(om)*d_om). Frequencies are spread from 0.6 to
%   3 times the peak frequency, with a random offset inside each band so that
%   the signal does not repeat.

if nargin < 4 || isempty(seed),  seed  = 1;   end
if nargin < 5 || isempty(gamma), gamma = 3.3; end
if nargin < 6 || isempty(n),     n     = 80;  end

rng(seed);
wp  = 2*pi/Tp;
edges = linspace(0.6*wp, 3*wp, n+1)';
dw  = diff(edges);
om  = edges(1:end-1) + rand(n,1).*dw;            % one random frequency per band

% JONSWAP shape (unnormalised), then scaled to the requested Hs
sig = 0.07*(om <= wp) + 0.09*(om > wp);
S   = om.^-5 .* exp(-1.25*(wp./om).^4) .* gamma.^exp(-(om - wp).^2 ./ (2*sig.^2*wp^2));
S   = S * (Hs/4)^2 / sum(S.*dw);                 % m0 = (Hs/4)^2

wv.type = 'irregular';
wv.a  = sqrt(2*S.*dw);
wv.om = om;
wv.k  = om.^2/p.g;                               % deep-water dispersion relation
wv.ph = 2*pi*rand(n,1);
wv.Hs = Hs;  wv.Tp = Tp;  wv.gamma = gamma;  wv.seed = seed;
wv.S  = S;
wv.zeta_a = Hs/2;                                % comparable regular amplitude, for reference
wv.we = om + wv.k*p.U;                           % encounter frequencies at p.U
end
