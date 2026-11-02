function V = fc_variants(P, names)
%FC_VARIANTS Controller variants of the FC-MPIC study.
%   V = fc_variants(P) returns all variants, V = fc_variants(P, names) only
%   those whose name is in the cell array NAMES.
%
%   Fields: name, label, problem ('fc': probfct_FC_MPIC.c, 'vi':
%   probfct_VI_MPIC.c), model (0 Gold et al., 1 FC-MPIC, 2 pure sliding),
%   disc (discrete-time map), opt (coupling, cone, mu, estimate), and for
%   VI-MPIC the fields of vi_variants (gate, psi, stiff, k, risk, cost).
%   opt.mu = NaN uses the model friction coefficient P.fc.mu.

V = [];
% classical MPIC (Gold et al. 2023): friction frozen at its measured value
V = [V, make('mpic',      'MPIC (Gold et al.)',               'fc', 0, 0)];
V = [V, make('mpic_d',    'MPIC, discrete time',              'fc', 0, 1)];
% FC-MPIC and the ablations of the proposal (Sec. 5.2)
V = [V, make('fc',        'FC-MPIC',                          'fc', 1, 1)];
V = [V, make('fc_nc',     'FC-MPIC without coupling',         'fc', 1, 1, struct('coupling', false))];
V = [V, make('fc_nocone', 'FC-MPIC without consistent cone',  'fc', 1, 1, struct('cone', false))];
V = [V, make('slide',     'pure-sliding Coulomb (Kleff)',     'fc', 2, 1)];
% unknown friction coefficient (proposal Table 4)
V = [V, make('fc_mu03',     'FC-MPIC, mu = 0.3',              'fc', 1, 1, struct('mu', 0.3))];
V = [V, make('fc_mu03_est', 'FC-MPIC, mu = 0.3, estimated',   'fc', 1, 1, struct('mu', 0.3, 'estimate', true))];
V = [V, make('fc_mu08',     'FC-MPIC, mu = 0.8',              'fc', 1, 1, struct('mu', 0.8))];
V = [V, make('fc_mu08_est', 'FC-MPIC, mu = 0.8, estimated',   'fc', 1, 1, struct('mu', 0.8, 'estimate', true))];
% VI-MPIC as drafted from the notes (vi_variants 'vimpic'); it needs the
% Cartesian controller ('cart'), whose stiffness it plans
vi = vi_variants(P.vi, {'vimpic'});
v = make('vimpic', 'VI-MPIC (first draft)', 'vi', NaN, 0);
for f = {'family', 'gate', 'psi', 'stiff', 'k', 'k_soft', 'risk', 'cost'}
    v.(f{1}) = vi.(f{1});
end
V = [V, v];

if nargin > 1
    keep = ismember({V.name}, names);
    V = V(keep);
end
end

function v = make(name, label, problem, model, disc, opt)
if nargin < 6
    opt = struct();
end
o = struct('coupling', true, 'cone', true, 'mu', NaN, 'estimate', false);
for f = fieldnames(opt)'
    o.(f{1}) = opt.(f{1});
end
v.name    = name;
v.label   = label;
v.problem = problem;
v.model   = model;
v.disc    = disc;
v.opt     = o;
% fields of vi_variants, used by VI-MPIC only
v.family  = '';
v.gate    = 0;
v.psi     = 0;
v.stiff   = 'fixed';
v.k       = NaN;
v.k_soft  = NaN;
v.risk    = false;
v.cost    = struct();
end
