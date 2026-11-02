function [T, S] = run_vi_benchmark(speeds, iters, names)
%RUN_VI_BENCHMARK Sweep the controller variants over wall offsets, approach speeds and solver iterations.
%   [T, S] = run_vi_benchmark() runs every variant of vi_variants for every
%   wall offset P.task.offsets, approach speed 0.1, 0.2, 0.3 m/s and
%   10 or 50 gradient iterations per MPC step. T holds one row per run,
%   S one row per controller, speed and iteration count (worst case and
%   mean over the offsets). Writes results/*.csv, results/*.mat and the
%   figures docs/figures/vi_*.png.
%
%   run_vi_benchmark(speeds, iters, names) restricts the sweep.

here = fileparts(mfilename('fullpath'));
old = cd(here);
cleanup = onCleanup(@() cd(old));
addpath(fullfile(here, '..', 'external', 'grampc', 'matlab', 'mfiles'));
if ~exist(fullfile(here, '+CmexFiles'), 'dir')
    vi_build();
end

P0 = vi_params();
if nargin < 1 || isempty(speeds)
    speeds = [0.1 0.2 0.3];
end
if nargin < 2 || isempty(iters)
    iters = [10 50];
end
V = vi_variants(P0);
if nargin > 2
    V = vi_variants(P0, names);
end
offs = P0.task.offsets;
show = {'mpic_k300', 'mpic_k10000', 'vimpic', 'vimpic_nopsi', 'sched_dist'};

rows = cell(0, 16);
keep = {};
for it = iters
    for v = speeds
        P = P0;
        P.task.v_app = v;
        P.mpc.MaxGradIter = it;
        for i = 1:numel(V)
            for j = 1:numel(offs)
                R = vi_simulate(P, V(i), offs(j));
                m = R.metrics;
                rows(end + 1, :) = {V(i).name, V(i).family, v, it, 1e3 * offs(j), 1e3 * m.e_track_rms, ...
                    1e3 * m.e_track_max, m.contact, m.t_approach, m.F_peak, m.impact_peak, ...
                    m.t_settle, m.F_rmse_end, m.k_contact, m.cpu_med, m.cpu_max}; %#ok<AGROW>
                if v == 0.2 && it == iters(1) && ismember(V(i).name, show) && (j == 1 || j == numel(offs))
                    keep{end + 1} = R; %#ok<AGROW>
                end
            end
            fprintf('iter %d  v_app %.1f  %-18s done\n', it, v, V(i).name);
        end
    end
end

T = cell2table(rows, 'VariableNames', {'name', 'family', 'v_app', 'iterations', 'offset_mm', ...
    'e_track_rms_mm', 'e_track_max_mm', 'contact', 't_approach_s', 'F_peak_N', 'impact_peak_N', ...
    't_settle_s', 'F_rmse_end_N', 'k_contact', 'cpu_med_ms', 'cpu_max_ms'});
S = summarise(T);

if ~exist(fullfile(here, 'results'), 'dir')
    mkdir(fullfile(here, 'results'));
end
writetable(T, fullfile(here, 'results', 'vi_benchmark_runs.csv'));
writetable(S, fullfile(here, 'results', 'vi_benchmark_summary.csv'));
save(fullfile(here, 'results', 'vi_benchmark.mat'), 'T', 'S', 'P0');

figdir = fullfile(here, '..', 'docs', 'figures');
if ~exist(figdir, 'dir')
    mkdir(figdir);
end
plotPareto(S, P0, speeds, iters, numel(offs), fullfile(figdir, 'vi_pareto.png'));
if ~isempty(keep)
    early = keep(cellfun(@(r) r.offset < 0, keep));
    late  = keep(cellfun(@(r) r.offset > 0, keep));
    vi_plot_runs(early, P0, fullfile(figdir, 'vi_runs_early_wall.png'), [1.15 2.2]);
    vi_plot_runs(late,  P0, fullfile(figdir, 'vi_runs_late_wall.png'),  [1.35 2.4]);
end
printSummary(S, numel(offs));
end

function S = summarise(T)
[g, name, v, it] = findgroups(T.name, T.v_app, T.iterations);
fam = splitapply(@(x) x(1), T.family, g);
S = table(name, fam, v, it, 'VariableNames', {'name', 'family', 'v_app', 'iterations'});
S.e_track_rms_mm    = splitapply(@mean, T.e_track_rms_mm, g);
S.n_contact         = splitapply(@sum, T.contact, g);
S.n_lost            = splitapply(@(x) sum(x > 1), T.F_rmse_end_N, g);
S.F_peak_max_N      = splitapply(@max, T.F_peak_N, g);
S.F_peak_mean_N     = splitapply(@mean, T.F_peak_N, g);
S.t_approach_mean_s = splitapply(@(x) mean(x, 'omitnan'), T.t_approach_s, g);
S.k_contact_mean    = splitapply(@(x) mean(x, 'omitnan'), T.k_contact, g);
S.cpu_med_ms        = splitapply(@median, T.cpu_med_ms, g);
S.cpu_max_ms        = splitapply(@max, T.cpu_max_ms, g);
S = sortrows(S, {'iterations', 'v_app', 'family', 'name'});
end

function printSummary(S, n)
fprintf('\n| iter | v_app | controller | e_track RMS [mm] | contact | lost | F_peak worst [N] | F_peak mean [N] | t_approach [s] | k at contact [N/m] | GRAMPC median/max [ms] |\n');
fprintf('|---|---|---|---|---|---|---|---|---|---|---|\n');
for i = 1:height(S)
    fprintf('| %d | %.1f | %s | %.2f | %d/%d | %d | %.2f | %.2f | %.3f | %.0f | %.2f / %.2f |\n', S.iterations(i), ...
        S.v_app(i), S.name{i}, S.e_track_rms_mm(i), S.n_contact(i), n, S.n_lost(i), S.F_peak_max_N(i), ...
        S.F_peak_mean_N(i), S.t_approach_mean_s(i), S.k_contact_mean(i), S.cpu_med_ms(i), S.cpu_max_ms(i));
end
end

function plotPareto(S, P, speeds, iters, n, file)
fig = figure('Color', 'w', 'Position', [100 100 1300 380 * numel(iters)], 'Visible', 'off');
tl = tiledlayout(fig, numel(iters), numel(speeds), 'TileSpacing', 'compact', 'Padding', 'compact');
mk = {'mpic', 'vimpic', 'vimpic_nopsi', 'vimpic_measgate', 'vimpic_uncond', 'vimpic_rs1', 'vimpic_weakdrive', 'sched_force', 'sched_dist'};
sym = {'-o', 'p', 'd', 's', '^', 'v', 'x', '<', '>'};
col = [0 0 0; 0.85 0.1 0.1; 0.95 0.5 0.1; 0.5 0.2 0.7; 0.2 0.6 0.2; 0.9 0.3 0.6; 0.5 0.5 0.5; 0.1 0.45 0.8; 0.1 0.7 0.8];
lab = {'MPIC, fixed gains', 'VI-MPIC', 'VI-MPIC without w.*ds/dt', 'VI-MPIC, measured gate', ...
       'VI-MPIC, unconditioned gate', 'VI-MPIC, r_s = 1', 'VI-MPIC, Q_v = 100', 'schedule on force', 'schedule on distance'};
for it = iters
    for v = speeds
        ax = nexttile(tl);
        hold(ax, 'on'); grid(ax, 'on');
        Sv = S(S.v_app == v & S.iterations == it, :);
        h = gobjects(1, numel(mk));
        fixed = Sv(startsWith(Sv.name, 'mpic_k'), :);
        [~, o] = sort(fixed.e_track_rms_mm);
        fixed = fixed(o, :);
        h(1) = plot(ax, fixed.e_track_rms_mm, fixed.F_peak_max_N, sym{1}, 'Color', col(1, :), 'MarkerFaceColor', 'w');
        for i = 1:height(fixed)
            text(ax, fixed.e_track_rms_mm(i), fixed.F_peak_max_N(i), sprintf('  %s', erase(fixed.name{i}, 'mpic_k')), 'FontSize', 7);
        end
        for i = 2:numel(mk)
            r = Sv(strcmp(Sv.name, mk{i}), :);
            if isempty(r)
                continue
            end
            y = r.F_peak_max_N;
            if r.n_contact < n
                y = P.task.F_des;       % no contact in some runs: drawn on the target line
            end
            h(i) = plot(ax, r.e_track_rms_mm, y, sym{i}, 'Color', col(i, :), 'MarkerSize', 10, 'LineWidth', 1.5);
            if r.n_contact < n
                text(ax, r.e_track_rms_mm, y, sprintf('  no contact in %d/%d', n - r.n_contact, n), 'FontSize', 7, 'Color', col(i, :));
            end
        end
        yline(ax, P.task.F_des, 'k:');
        set(ax, 'XScale', 'log');
        xlabel(ax, 'free-motion tracking error, RMS [mm]');
        ylabel(ax, 'worst force peak over wall offsets [N]');
        title(ax, sprintf('approach %.1f m/s, %d iterations per step', v, it));
        valid = isgraphics(h);
    end
end
legend(ax, h(valid), lab(valid), 'Location', 'northeastoutside', 'Interpreter', 'none');
exportgraphics(fig, file, 'Resolution', 120);
end
