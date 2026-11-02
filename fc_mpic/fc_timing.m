function Tm = fc_timing()
%FC_TIMING GRAMPC time per MPC step of each controller, measured alone.
%   Tm = fc_timing() runs each controller once (nominal case, Qp_y = 1e5,
%   seed 1) with 10 and 50 gradient iterations, one run at a time, and
%   prints the median and the 95th percentile of the GRAMPC time per step
%   (the benchmark sets run in parallel processes, which inflates their
%   times). Run it with no other MATLAB jobs active.

here = fileparts(mfilename('fullpath'));
old = cd(here);
cleanup = onCleanup(@() cd(old));
runs = {'joint', 'mpic'; 'joint', 'mpic_d'; 'joint', 'fc'; 'joint', 'slide'; 'cart', 'mpic'; 'cart', 'fc'; 'cart', 'vimpic'};
rows = cell(0, 5);
for it = [10 50]
    for i = 1:size(runs, 1)
        P = fc_params('nominal');
        P.ctrl.type = runs{i, 1};
        P.cost.Qp = [1e5 1e5];
        P.mpc.MaxGradIter = it;
        V = fc_variants(P, runs(i, 2));
        fc_simulate(P, V, 1);                   % warm-up
        R = fc_simulate(P, V, 1);
        rows(end + 1, :) = {runs{i, 1}, runs{i, 2}, it, median(R.log.cpu), prctile(R.log.cpu, 95)}; %#ok<AGROW>
    end
end
Tm = cell2table(rows, 'VariableNames', {'ctrl', 'name', 'iterations', 'cpu_med_ms', 'cpu_p95_ms'});
fprintf('| controller | low-level controller | iterations | median [ms] | 95th percentile [ms] |\n|---|---|---|---|---|\n');
for i = 1:height(Tm)
    fprintf('| %s | %s | %d | %.2f | %.2f |\n', Tm.name{i}, Tm.ctrl{i}, Tm.iterations(i), Tm.cpu_med_ms(i), Tm.cpu_p95_ms(i));
end
end
