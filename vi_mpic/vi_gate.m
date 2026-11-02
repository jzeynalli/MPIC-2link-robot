function [gam, g, v, dgam, dv] = vi_gate(m, phat, sig, ell, mode, gmeas)
%VI_GATE Contact gate and penetration moments along the wall normal.
%   Mirror of gateEval() in probfct_VI_MPIC.c.
%
%   m      planned depth of the desired frame [m]
%   phat   believed wall position, sig its standard deviation [m]
%   ell    deepest contact-free depth reached so far [m] (mode 2)
%   mode   0 measured gate (gmeas), 1 unconditioned Phi(z), 2 conditioned
%
%   gam    gate = contact probability, g = E[d+], v = Var[d+],
%   dgam   d gam / d m,  dv = d v / d m

if nargin < 6
    gmeas = 0;
end
gam = 0; g = 0; v = 0; dgam = 0; dv = 0;

if mode == 0
    gam = gmeas;
    v = gam * sig^2;
    return
end

b = (m - phat) / sig;
if mode == 2
    a = min((ell - phat) / sig, 30);
else
    a = -40;
end
if b <= a
    return
end

Qa   = 0.5 * erfc(a / sqrt(2));
dPhi = Qa - 0.5 * erfc(b / sqrt(2));
pa   = exp(-a^2 / 2) / sqrt(2 * pi);
pb   = exp(-b^2 / 2) / sqrt(2 * pi);
g    = sig * (b * dPhi + pb - pa) / Qa;
E2   = sig^2 * ((b^2 + 1) * dPhi + b * pb + (a - 2 * b) * pa) / Qa;

gam  = dPhi / Qa;
v    = max(E2 - g^2, 0);
dgam = pb / (sig * Qa);
dv   = 2 * g * (1 - gam);
end
