function T = run_fc_benchmark(set, iters, seeds)
%RUN_FC_BENCHMARK Closed-loop sanding runs of one benchmark set.
%   T = run_fc_benchmark(set, iters, seeds) runs the controllers of SET
%   with ITERS gradient iterations per MPC step for every friction case,
%   tracking weight Qp_y and noise seed, and writes
%   results/fc_<set>_it<iters>.csv (one row per run). Sets:
%     'joint'  joint PD of Gold et al. (kq = 120 Nm/rad): MPIC, MPIC in
%              discrete time, FC-MPIC and its ablations (no coupling, no
%              consistent cone, pure sliding)
%     'kq'     MPIC and FC-MPIC with softer and stiffer joints (60, 400 Nm/rad)
%     'cart'   controller of VI-MPIC: MPIC and FC-MPIC with fixed Cartesian
%              stiffness (300 ... 10000 N/m, and 300/10000 N/m along x/y),
%              and VI-MPIC (first draft), which plans the stiffness
%     'mu'     FC-MPIC with a wrong friction coefficient, with and without
%              the online estimate (nominal case)
%     'joint_hi'  as 'joint' (MPIC, FC-MPIC, ablation without coupling) with
%              larger tracking weights, to complete the trade-off curves
%     'decoupled' MPIC and FC-MPIC with a Cartesian controller that has the
%              stiffness of the joint PD along the wall normal and tangent
%              at the start pose but no coupling between them
%     'stroke' MPIC and FC-MPIC with a 3 cm stroke instead of 8 cm
%     'cart_d' MPIC in discrete time with the gains of 'cart'
%   The tracking weight Qp_y traces each controller's trade-off between
%   tangential tracking and normal-force regulation (the force weight is
%   fixed). fc_report summarises all sets.

if nargin < 3
    seeds = 1:3;
end
here = fileparts(mfilename('fullpath'));
old = cd(here);
cleanup = onCleanup(@() cd(old));
if ~exist(fullfile(here, '+CmexFC'), 'dir') || ~exist(fullfile(here, '+CmexVI'), 'dir')
    fc_build();
end

Qps = [5e3 2e4 1e5 5e5];
Qhi = [2e6 1e7];
cases = {'nominal', 'abrasive'};
label = set;                            % set name in the table
A = NaN;                                % stroke amplitude, NaN: fc_params
switch set
    case 'joint'
        jobs = makeJobs({'mpic', 'mpic_d', 'fc', 'fc_nc', 'fc_nocone', 'slide'}, 'joint', {120}, cases, Qps);
    case 'joint_hi'
        jobs = makeJobs({'mpic', 'mpic_d', 'fc', 'fc_nc'}, 'joint', {120}, cases, Qhi);
        label = 'joint';
    case 'kq'
        jobs = makeJobs({'mpic', 'mpic_d', 'fc'}, 'joint', {60, 400}, {'nominal'}, Qps);
    case 'cart'
        ks = {300, 1000, 3000, 10000, [300 10000]};
        jobs = [makeJobs({'mpic', 'fc'}, 'cart', ks, cases, Qps), makeJobs({'vimpic'}, 'cart', {NaN}, cases, Qps)];
    case 'cart_d'
        % MPIC in discrete time on the controller of VI-MPIC, the like-for-like
        % reference of FC-MPIC there
        jobs = makeJobs({'mpic_d'}, 'cart', {300, 1000, 3000, 10000, [300 10000]}, cases, Qps);
        label = 'cart';
    case 'mu'
        jobs = makeJobs({'fc_mu03', 'fc_mu03_est', 'fc_mu08', 'fc_mu08_est'}, 'joint', {120}, {'nominal'}, Qps);
    case 'decoupled'
        % diagonal of the joint PD's compliance C = J Kp^-1 J' at the start pose
        P = fc_params();
        q0 = vi_ik([P.wall.x + P.task.F_des / P.wall.ke; P.task.y0], P.robot);
        [~, J] = vi_kin(q0, P.robot);
        kc = round(1 ./ diag(J * (diag(P.ctrl.kq) \ J')))';
        jobs = makeJobs({'mpic', 'fc'}, 'cart', {kc}, cases, [Qps Qhi]);
    case 'stroke'
        jobs = makeJobs({'mpic', 'mpic_d', 'fc'}, 'joint', {120}, cases, [Qps Qhi]);
        A = 0.015;
    otherwise
        error('run_fc_benchmark: unknown set %s', set);
end

names = {'set', 'ctrl', 'gain', 'fcase', 'name', 'Qp', 'iterations', 'seed', 'et_rms', 'et_max', 'et_rev', 'et_mid', ...
         'efn_rms', 'efn_max', 'efn_rev', 'efn_mid', 'lost', 'fn_pos', 'fn_neg', 'ft_pos', 'ft_neg', 'rn', ...
         'mu_end', 'capped', 'cpu_med', 'cpu_max'};
rows = cell(numel(jobs) * numel(seeds), numel(names));
r = 0;
tic;
for i = 1:numel(jobs)
    jb = jobs(i);
    P = fc_params(jb.fcase);
    if ~isnan(A)
        P.task.A = A;
        P.task.y0 = -A;                 % strokes centred on y = 0
    end
    P.ctrl.type = jb.ctrl;
    if strcmp(jb.ctrl, 'joint')
        P.ctrl.kq = jb.gain * [1; 1];
        gain = sprintf('%g', jb.gain);
    elseif ~isnan(jb.gain(1))
        P.ctrl.kc = jb.gain(:) .* [1; 1];
        gain = strjoin(arrayfun(@(g) sprintf('%g', g), jb.gain, 'UniformOutput', false), '/');
    else
        gain = 'planned';
    end
    P.cost.Qp = jb.Qp * [1 1];
    P.mpc.MaxGradIter = iters;
    V = fc_variants(P, {jb.name});
    for sd = seeds
        R = fc_simulate(P, V, sd);
        m = R.metrics;
        r = r + 1;
        rows(r, :) = {label, jb.ctrl, gain, jb.fcase, jb.name, jb.Qp, iters, sd, m.et_rms, m.et_max, m.et_rev, m.et_mid, ...
                      m.efn_rms, m.efn_max, m.efn_rev, m.efn_mid, m.lost, m.fn_pos, m.fn_neg, m.ft_pos, m.ft_neg, m.rn, ...
                      m.mu_end, m.capped, m.cpu_med, m.cpu_max};
    end
    fprintf('%s it %d  %3d/%d  %-8s %-9s %-11s Qp %-6g  et %.3f mm  efn %.3f N  (%.0f s)\n', set, iters, i, numel(jobs), ...
            jb.fcase, gain, jb.name, jb.Qp, m.et_rms, m.efn_rms, toc);
end
T = cell2table(rows(1:r, :), 'VariableNames', names);
if ~exist(fullfile(here, 'results'), 'dir')
    mkdir(fullfile(here, 'results'));
end
writetable(T, fullfile(here, 'results', sprintf('fc_%s_it%d.csv', set, iters)));
end

function jobs = makeJobs(names, ctrl, gains, cases, Qps)
jobs = struct('name', {}, 'ctrl', {}, 'gain', {}, 'fcase', {}, 'Qp', {});
for c = 1:numel(cases)
    for g = 1:numel(gains)
        for n = 1:numel(names)
            for qp = Qps
                jobs(end + 1) = struct('name', names{n}, 'ctrl', ctrl, 'gain', gains{g}, 'fcase', cases{c}, 'Qp', qp); %#ok<AGROW>
            end
        end
    end
end
end
