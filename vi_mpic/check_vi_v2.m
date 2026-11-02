function check_vi_v2()
%CHECK_VI_V2 Diagnosis and robustness checks behind docs/vi-mpic-v2.md.
%   (1) impact diagnosis: reference velocity along the normal at the first two
%       MPC steps after contact, force peak and its time (0.2 m/s, 10 iterations)
%   (2) limit cycle of the converged MPC (50 iterations) in sliding contact:
%       writes docs/figures/vi2_limit_cycle.png
%   (3) does a larger reference-velocity weight R remove the limit cycle?
%   (4) optimum of the stiffness cost in steady contact (Prop. 3 of the notes)

here = fileparts(mfilename('fullpath'));
old = cd(here);
cleanup = onCleanup(@() cd(old));
addpath(fullfile(here, '..', 'external', 'grampc', 'matlab', 'mfiles'));
P0 = vi_params();
figdir = fullfile(here, '..', 'docs', 'figures');

% ---- (1) impact diagnosis
fprintf('(1) impact at 0.2 m/s, 10 iterations, 9 wall offsets\n');
P = P0;
P.task.v_app = 0.2;
n = round(P.mpc.dt / P.robot.h);
for nm = {'mpic_k300', 'mpic_k10000', 'vimpic', 'vimpic_v2_rfx'}
    V = vi_variants(P, nm);
    vr = zeros(numel(P.task.offsets), 2);
    pk = zeros(numel(P.task.offsets), 2);
    for j = 1:numel(P.task.offsets)
        Fn = vi_simulate(P, V, P.task.offsets(j)).fine;
        ic = find(Fn.F > 0, 1);
        w = Fn.t >= Fn.t(ic) & Fn.t <= Fn.t(ic) + 0.1;     % impact window
        [pk(j, 1), ip] = max(Fn.F(w));
        pk(j, 2) = 1e3 * (ip - 1) * P.robot.h;
        ks = ceil(Fn.t(ic) / P.mpc.dt + 1e-9);              % next MPC sample
        vr(j, :) = [Fn.vref(min(ks * n + 1, end)), Fn.vref(min((ks + 1) * n + 1, end))];
    end
    fprintf('    %-14s reference velocity 1st / 2nd step after contact %+.3f..%+.3f / %+.3f..%+.3f m/s, impact peak %.2f..%.2f N after %.1f..%.1f ms\n', ...
        nm{1}, min(vr(:, 1)), max(vr(:, 1)), min(vr(:, 2)), max(vr(:, 2)), min(pk(:, 1)), max(pk(:, 1)), min(pk(:, 2)), max(pk(:, 2)));
end

% ---- (2) limit cycle with 50 iterations in sliding contact
fprintf('(2) sliding, 50 iterations, wall at p_hat\n');
P = P0;
P.task.v_app = 0.2;
P.task.slide.enable = true;
P.task.T_sim = 4.0;
P.mpc.MaxGradIter = 50;
V = vi_variants(P, {'mpic_k3000_rfx', 'mpic_k10000_rfx'});
runs = {vi_simulate(P, V(1), 0), vi_simulate(P, V(2), 0)};
for i = 1:2
    Fn = runs{i}.fine;
    w = Fn.t > 3.0;
    fprintf('    %-16s force after 3 s %.2f..%.2f N, contact losses %d\n', V(i).name, min(Fn.F(w)), max(Fn.F(w)), sum(diff(Fn.F(w) > 0.01) == -1));
end
vi_plot_runs(runs, P, fullfile(figdir, 'vi2_limit_cycle.png'), [1.3 4.0]);

% ---- (3) larger reference-velocity weight
fprintf('(3) sliding, runs not settled at 5 N (of 3 offsets) for weights R on dqd/dt\n');
names = {'mpic_k3000_rfx', 'mpic_k10000_rfx', 'sched_contact_rfx', 'vimpic_v2_rfx'};
for it = [10 50]
    for RQ = [0.01 0.1 1]
        P.mpc.MaxGradIter = it;
        P.cost.RQ = RQ;
        V = vi_variants(P, names);
        [~, o] = ismember(names, {V.name});
        V = V(o);
        u = zeros(1, numel(V));
        for i = 1:numel(V)
            for d = 0.01 * [-2 0 2]
                m = vi_simulate(P, V(i), d).metrics;
                u(i) = u(i) + (m.F_rmse_end > 1 || ~m.contact);
            end
        end
        fprintf('    %2d iterations, R = %4.2f: %s\n', it, RQ, strjoin(compose('%s %d', string(names'), u'), ', '));
    end
end

% ---- (4) optimum of the stiffness cost in steady contact, Prop. 3 of the notes:
% k* = qp sigma_w^2 / (qF sigma^2 ke^3)
qp = P0.cost.qp;  sw = P0.cost.sigma_w;  qF = P0.cost.qF;  sg = 2e-3;
fprintf('(4) k* in contact (sigma = %.0f mm, sigma_w = %.0f N):', 1e3 * sg, sw);
for ke = [5000 2000 1000 500 300]
    fprintf('  ke %4d -> %.0f N/m;', ke, qp * sw^2 / (qF * sg^2 * ke^3));
end
fprintf('\n    floor 900 N/m: the optimum is the floor unless ke < %.0f N/m\n', (qp * sw^2 / (qF * sg^2 * 900))^(1/3));
end
