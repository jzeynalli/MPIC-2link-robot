function V = vi_variants(P, names)
%VI_VARIANTS Controller variants of the VI-MPIC study.
%   V = vi_variants(P) returns all variants, V = vi_variants(P, names) only
%   those whose name is in the cell array NAMES.
%
%   Fields: name, family, gate (0 measured, 1 Phi(z), 2 conditioned),
%   psi (stiffness-rate term), stiff ('fixed', 'vi', 'sched_force',
%   'sched_dist'), k (fixed or initial stiffness), k_soft (schedules),
%   risk (stiffness cost eq. 30 on), cost (overrides of P.cost fields),
%   P (overrides of top-level fields of P, e.g. k_min), opt (improvements):
%     contact_gate  'model' (first draft) or 'measured': gamma = 1 once in contact
%     k_hold        true: no stiffness actuation in the contact primitive
%     init_fix      true: force state 0, not kbar g, after a loss of contact
%     reflex        true: the 1 kHz loop stops reference and stiffness at contact
%   Names ending in '_rfx' are the same controller with the reflex.

ks = [300 500 800 1200 2000 3000 5000 10000];
V = [];

% classical MPIC (Gold et al. 2023): fixed gains, measured gate
for k = ks
    V = [V, make(sprintf('mpic_k%d', k), 'MPIC, fixed gains', 0, 0, 'fixed', k)]; %#ok<AGROW>
end
% VI-MPIC as in the notes (first draft) and its ablations
V = [V, make('vimpic',           'VI-MPIC',                         2, 1, 'vi', P.k_max)];
V = [V, make('vimpic_nopsi',     'VI-MPIC without w.*sdot term',    2, 0, 'vi', P.k_max)];
V = [V, make('vimpic_measgate',  'VI-MPIC with measured gate',      0, 1, 'vi', P.k_max)];
V = [V, make('vimpic_uncond',    'VI-MPIC with unconditioned gate', 1, 1, 'vi', P.k_max)];
V = [V, make('vimpic_rs1',       'VI-MPIC with r_s = 1',            2, 1, 'vi', P.k_max, NaN, struct('RS', 1))];
V = [V, make('vimpic_weakdrive', 'VI-MPIC with Q_v = 100',          2, 1, 'vi', P.k_max, NaN, struct('Qv', 100))];
% improvement ladder: each step adds one change to the previous one
cg     = struct('contact_gate', 'measured');
hold_  = struct('contact_gate', 'measured', 'k_hold', true);
floor_ = struct('k_min', 900);
% VI-MPIC v2: measured gate in contact; stiffness held for 0.2 s after
% contact, then at most 3 1/s; force risk only in contact, with sigma = 2 mm
% of unknown surface variation; force state reset after a loss of contact;
% contact-aware floor. Meant to run with the 1 kHz reflex ('vimpic_v2_rfx')
v2 = struct('contact_gate', 'measured', 'k_hold', true, 'k_hold_time', 0.2, 'k_rate', 3, ...
            'risk_contact_only', true, 'sigma_contact', 2e-3, 'init_fix', true);
V = [V, make('vimpic_cg',         'VI-MPIC + measured gate in contact', 2, 1, 'vi', P.k_max, NaN, struct(), cg)];
V = [V, make('vimpic_cg_hold',    '... + stiffness held in contact',    2, 1, 'vi', P.k_max, NaN, struct(), hold_)];
V = [V, make('vimpic_cg_hold_fl', '... + contact-aware floor 900 N/m',  2, 1, 'vi', P.k_max, NaN, struct(), hold_, floor_)];
V = [V, make('vimpic_v2',         'VI-MPIC v2',                         2, 1, 'vi', P.k_max, NaN, struct(), v2, floor_)];
% hand-designed counterpart of v2: stiff until contact, hold 0.2 s, then
% soften at 3 1/s to the floor
V = [V, make('sched_contact', 'Schedule: soften after contact', 0, 0, 'sched_force', P.k_max, 900, struct(), ...
             struct('k_hold', true, 'k_hold_time', 0.2, 'k_rate', 3))];
% heuristic stiffness schedules along the wall normal
% (both on the measured gate: with the conditioned gate a soft spring whose
% reference is deep reads as "pressing" after a bounce, see docs/vi-mpic.md)
V = [V, make('sched_force', 'Schedule switched on contact force', 0, 0, 'sched_force', P.k_max, P.k_min)];
V = [V, make('sched_dist',  'Schedule on distance to the wall',  0, 0, 'sched_dist',  P.k_max, P.k_min)];

% the same controllers with the 1 kHz contact reflex in the low-level loop
base = V;
for i = 1:numel(base)
    r = base(i);
    r.name = [r.name '_rfx'];
    r.family = [r.family ', reflex'];
    r.opt.reflex = true;
    V = [V, r]; %#ok<AGROW>
end

if nargin > 1
    keep = ismember({V.name}, names);
    V = V(keep);
end
end

function v = make(name, family, gate, psi, stiff, k, k_soft, cost, opt, Pover)
if nargin < 7
    k_soft = NaN;
end
if nargin < 8
    cost = struct();
end
if nargin < 9
    opt = struct();
end
if nargin < 10
    Pover = struct();
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
v.opt    = opt;
v.P      = Pover;
end
