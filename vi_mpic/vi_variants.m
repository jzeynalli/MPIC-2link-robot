function V = vi_variants(P, names)
%VI_VARIANTS Controller variants of the VI-MPIC study.
%   V = vi_variants(P) returns all variants, V = vi_variants(P, names) only
%   those whose name is in the cell array NAMES.
%
%   Fields: name, family, gate (0 measured, 1 Phi(z), 2 conditioned),
%   psi (stiffness-rate term), stiff ('fixed', 'vi', 'sched_force',
%   'sched_dist'), k (fixed or initial stiffness), k_soft (schedules),
%   risk (stiffness cost eq. 30 on), cost (overrides of P.cost fields).

ks = [300 500 800 1200 2000 3000 5000 10000];
V = [];

% classical MPIC (Gold et al. 2023): fixed gains, measured gate
for k = ks
    V = [V, make(sprintf('mpic_k%d', k), 'MPIC, fixed gains', 0, 0, 'fixed', k)]; %#ok<AGROW>
end
% VI-MPIC as in the notes and its ablations
V = [V, make('vimpic',           'VI-MPIC',                         2, 1, 'vi', P.k_max)];
V = [V, make('vimpic_nopsi',     'VI-MPIC without w.*sdot term',    2, 0, 'vi', P.k_max)];
V = [V, make('vimpic_measgate',  'VI-MPIC with measured gate',      0, 1, 'vi', P.k_max)];
V = [V, make('vimpic_uncond',    'VI-MPIC with unconditioned gate', 1, 1, 'vi', P.k_max)];
V = [V, make('vimpic_rs1',       'VI-MPIC with r_s = 1',            2, 1, 'vi', P.k_max, NaN, struct('RS', 1))];
V = [V, make('vimpic_weakdrive', 'VI-MPIC with Q_v = 100',          2, 1, 'vi', P.k_max, NaN, struct('Qv', 100))];
% heuristic stiffness schedules along the wall normal
% (both on the measured gate: with the conditioned gate a soft spring whose
% reference is deep reads as "pressing" after a bounce, see docs/vi-mpic.md)
V = [V, make('sched_force', 'Schedule switched on contact force', 0, 0, 'sched_force', P.k_max, P.k_min)];
V = [V, make('sched_dist',  'Schedule on distance to the wall',  0, 0, 'sched_dist',  P.k_max, P.k_min)];

if nargin > 1
    keep = ismember({V.name}, names);
    V = V(keep);
end
end

function v = make(name, family, gate, psi, stiff, k, k_soft, cost)
if nargin < 7
    k_soft = NaN;
end
if nargin < 8
    cost = struct();
end
v.name   = name;
v.family = family;
v.gate   = gate;
v.psi    = psi;
v.stiff  = stiff;
v.k      = k;
v.k_soft = k_soft;
v.risk   = strcmp(stiff, 'vi');
v.cost   = cost;
end
