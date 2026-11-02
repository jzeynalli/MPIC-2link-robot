function [SI, SS] = vi_report_v2()
%VI_REPORT_V2 Tables and figures of the VI-MPIC v2 study.
%   Merges the first-draft runs (results/vi_benchmark_runs.csv: fixed gains,
%   vimpic and sched_dist without reflex, reproduced bit for bit by the
%   current code) with the v2 sweeps (results/v2_*.csv, see run_vi_sweep),
%   prints markdown tables and writes docs/figures/vi2_*.png.
%   SI: impact task per controller, speed and iteration count.
%   SS: sliding task per controller and iteration count.

here = fileparts(mfilename('fullpath'));
old = cd(here);
cleanup = onCleanup(@() cd(old));
addpath(fullfile(here, '..', 'external', 'grampc', 'matlab', 'mfiles'));
P = vi_params();
figdir = fullfile(here, '..', 'docs', 'figures');

% ---- impact task: first-draft rows + v2 sweeps
T1 = readtable(fullfile('results', 'vi_benchmark_runs.csv'), 'TextType', 'char');
T1 = T1(startsWith(T1.name, 'mpic_k') | ismember(T1.name, {'vimpic', 'sched_dist'}), :);
T1.task = repmat({'impact'}, height(T1), 1);
T1.reflex = zeros(height(T1), 1);
for c = {'F_rmse_slide_N', 'F_dev_slide_N', 'y_rmse_slide_mm', 'k_slide'}
    T1.(c{1}) = nan(height(T1), 1);
end
T2 = [readtable(fullfile('results', 'v2_impact_fixed_rfx.csv'), 'TextType', 'char');
      readtable(fullfile('results', 'v2_impact_ladder.csv'), 'TextType', 'char');
      readtable(fullfile('results', 'v2_impact_ladder_rfx.csv'), 'TextType', 'char')];
TI = [T1(:, T2.Properties.VariableNames); T2];
TS = readtable(fullfile('results', 'v2_slide.csv'), 'TextType', 'char');

SI = summarise(TI, {'name', 'v_app', 'iterations'});
SS = summarise(TS, {'name', 'iterations'});
writetable(SI, fullfile('results', 'v2_impact_summary.csv'));
writetable(SS, fullfile('results', 'v2_slide_summary.csv'));

% ---- tables
ladder = {'vimpic', 'vimpic_cg', 'vimpic_cg_hold', 'vimpic_cg_hold_fl', 'vimpic_v2'};
fprintf('\nImpact, worst force peak [N] at 0.1 / 0.2 / 0.3 m/s (unsettled runs of 27)\n');
fprintf('| controller | 10 it, no reflex | 10 it, reflex | 50 it, no reflex | 50 it, reflex |\n|---|---|---|---|---|\n');
rows = [ladder, {'sched_dist', 'sched_contact', 'mpic_k300', 'mpic_k800', 'mpic_k3000', 'mpic_k10000'}];
for i = 1:numel(rows)
    fprintf('| %s |', rows{i});
    for it = [10 50]
        for rf = [0 1]
            nm = rows{i};
            if rf
                nm = [nm '_rfx']; %#ok<AGROW>
            end
            fprintf(' %s |', cellPeak(SI, nm, it));
        end
    end
    fprintf('\n');
end

fprintf('\nImpact, free-motion tracking RMS [mm] at 0.2 m/s and median solve time [ms]\n');
for nm = {'mpic_k10000', 'mpic_k10000_rfx', 'vimpic', 'vimpic_v2', 'vimpic_v2_rfx', 'sched_contact_rfx'}
    for it = [10 50]
        r = SI(strcmp(SI.name, nm{1}) & SI.v_app == 0.2 & SI.iterations == it, :);
        if ~isempty(r)
            fprintf('  %-18s it %2d: e %.2f mm, t_approach %.3f s, solve %.2f ms\n', nm{1}, it, r.e_track_rms_mm, r.t_approach_mean_s, r.cpu_med_ms);
        end
    end
end

fprintf('\nSliding (0.2 m/s approach, 5 wall offsets)\n');
fprintf('| controller | it | tracking RMS [mm] | force peak worst [N] | sliding force RMS mean / worst [N] | worst deviation [N] | tangential RMS [mm] | k while sliding [N/m] | unsettled |\n');
fprintf('|---|---|---|---|---|---|---|---|---|\n');
for i = 1:height(SS)
    fprintf('| %s | %d | %.2f | %.2f | %.3f / %.3f | %.2f | %.2f | %.0f | %d |\n', SS.name{i}, SS.iterations(i), ...
        SS.e_track_rms_mm(i), SS.F_peak_max_N(i), SS.F_rmse_slide_mean_N(i), SS.F_rmse_slide_max_N(i), ...
        SS.F_dev_slide_max_N(i), SS.y_rmse_slide_mean_mm(i), SS.k_slide_mean(i), SS.n_unsettled(i));
end

plotImpact(SI, P, fullfile(figdir, 'vi2_impact_pareto.png'));
plotLadder(SI, ladder, fullfile(figdir, 'vi2_ladder.png'));
plotSlide(SS, fullfile(figdir, 'vi2_slide_pareto.png'));
plotSlideRuns(P, fullfile(figdir, 'vi2_slide_runs.png'));
end

function S = summarise(T, keys)
[g, tk] = findgroups(T(:, keys));
S = tk;
S.e_track_rms_mm      = splitapply(@mean, T.e_track_rms_mm, g);
S.n_runs              = splitapply(@numel, T.F_peak_N, g);
S.n_unsettled         = splitapply(@(c, f) sum(~c | f > 1), T.contact, T.F_rmse_end_N, g);
S.F_peak_max_N        = splitapply(@max, T.F_peak_N, g);
S.F_peak_mean_N       = splitapply(@mean, T.F_peak_N, g);
S.t_approach_mean_s   = splitapply(@(x) mean(x, 'omitnan'), T.t_approach_s, g);
S.cpu_med_ms          = splitapply(@median, T.cpu_med_ms, g);
S.F_rmse_slide_mean_N = splitapply(@(x) mean(x, 'omitnan'), T.F_rmse_slide_N, g);
S.F_rmse_slide_max_N  = splitapply(@max, T.F_rmse_slide_N, g);
S.F_dev_slide_max_N   = splitapply(@max, T.F_dev_slide_N, g);
S.y_rmse_slide_mean_mm = splitapply(@(x) mean(x, 'omitnan'), T.y_rmse_slide_mm, g);
S.k_slide_mean        = splitapply(@(x) mean(x, 'omitnan'), T.k_slide, g);
end

function s = cellPeak(SI, nm, it)
v = [0.1 0.2 0.3];
p = nan(1, 3);
u = 0;
for i = 1:3
    r = SI(strcmp(SI.name, nm) & SI.iterations == it & SI.v_app == v(i), :);
    if ~isempty(r)
        p(i) = r.F_peak_max_N;
        u = u + r.n_unsettled;
    end
end
if all(isnan(p))
    s = '-';
else
    s = sprintf('%.1f / %.1f / %.1f', p);
    if u > 0
        s = sprintf('%s (%d)', s, u);
    end
end
end

function plotImpact(SI, P, file)
fig = figure('Color', 'w', 'Position', [100 100 1300 760], 'Visible', 'off');
tl = tiledlayout(fig, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
mk = {'vimpic', 'vimpic_v2', 'vimpic_v2_rfx', 'sched_contact_rfx', 'sched_dist_rfx'};
sym = {'p', 'd', 'h', '>', '<'};
col = [0.85 0.1 0.1; 0.95 0.5 0.1; 0.1 0.6 0.2; 0.1 0.45 0.8; 0.5 0.2 0.7];
lab = {'MPIC fixed gains', 'MPIC fixed gains + reflex', 'VI-MPIC (first draft)', 'VI-MPIC v2', ...
       'VI-MPIC v2 + reflex', 'schedule after contact + reflex', 'schedule on distance + reflex'};
for it = [10 50]
    for v = [0.1 0.2 0.3]
        ax = nexttile(tl);
        hold(ax, 'on'); grid(ax, 'on');
        S = SI(SI.v_app == v & SI.iterations == it, :);
        h = gobjects(1, 7);
        f0 = S(startsWith(S.name, 'mpic_k') & ~endsWith(S.name, '_rfx'), :);
        f1 = S(startsWith(S.name, 'mpic_k') & endsWith(S.name, '_rfx'), :);
        f0 = sortrows(f0, 'e_track_rms_mm');
        f1 = sortrows(f1, 'e_track_rms_mm');
        h(1) = plot(ax, f0.e_track_rms_mm, f0.F_peak_max_N, '-o', 'Color', [0 0 0], 'MarkerFaceColor', 'w');
        h(2) = plot(ax, f1.e_track_rms_mm, f1.F_peak_max_N, '--s', 'Color', [0.4 0.4 0.4], 'MarkerFaceColor', [0.8 0.8 0.8]);
        for i = 1:numel(mk)
            r = S(strcmp(S.name, mk{i}), :);
            if ~isempty(r)
                h(2 + i) = plot(ax, r.e_track_rms_mm, r.F_peak_max_N, sym{i}, 'Color', col(i, :), 'MarkerSize', 11, 'LineWidth', 1.6);
            end
        end
        yline(ax, P.task.F_des, 'k:');
        set(ax, 'XScale', 'log');
        xlabel(ax, 'free-motion tracking error, RMS [mm]');
        ylabel(ax, 'worst force peak over wall offsets [N]');
        title(ax, sprintf('impact: %.1f m/s, %d iterations', v, it));
        valid = isgraphics(h);
    end
end
legend(ax, h(valid), lab(valid), 'Location', 'northeastoutside', 'Interpreter', 'none');
exportgraphics(fig, file, 'Resolution', 120);
end

function plotLadder(SI, ladder, file)
fig = figure('Color', 'w', 'Position', [100 100 1200 420], 'Visible', 'off');
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
lab = {'first draft', '+ measured gate in contact', '+ stiffness held in contact', '+ floor 900 N/m', 'v2'};
ref = {'mpic_k10000', 'mpic_k10000_rfx'};
for it = [10 50]
    ax = nexttile(tl);
    hold(ax, 'on'); grid(ax, 'on');
    Y = nan(numel(ladder), 2);
    for i = 1:numel(ladder)
        for rf = 1:2
            nm = ladder{i};
            if rf == 2
                nm = [nm '_rfx']; %#ok<AGROW>
            end
            r = SI(strcmp(SI.name, nm) & SI.v_app == 0.2 & SI.iterations == it, :);
            if ~isempty(r)
                Y(i, rf) = r.F_peak_max_N;
            end
        end
    end
    b = bar(ax, Y);
    b(1).FaceColor = [0.85 0.4 0.3];
    b(2).FaceColor = [0.3 0.55 0.85];
    for rf = 1:2
        r = SI(strcmp(SI.name, ref{rf}) & SI.v_app == 0.2 & SI.iterations == it, :);
        yline(ax, r.F_peak_max_N, '--', 'Color', b(rf).FaceColor, 'LineWidth', 1.2);
    end
    set(ax, 'XTick', 1:numel(ladder), 'XTickLabel', lab, 'XTickLabelRotation', 20);
    ylabel(ax, 'worst force peak at 0.2 m/s [N]');
    title(ax, sprintf('%d iterations per step (dashed: fixed 10 kN/m)', it));
    legend(ax, {'no reflex', 'with 1 kHz reflex'}, 'Location', 'northeast');
end
exportgraphics(fig, file, 'Resolution', 120);
end

function plotSlide(SS, file)
fig = figure('Color', 'w', 'Position', [100 100 1150 430], 'Visible', 'off');
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
mk = {'vimpic_rfx', 'vimpic_v2_rfx', 'sched_contact_rfx', 'vimpic_v2', 'sched_contact', 'mpic_k800', 'mpic_k10000'};
sym = {'p', 'h', '>', 'd', '<', 'o', 'o'};
col = [0.85 0.1 0.1; 0.1 0.6 0.2; 0.1 0.45 0.8; 0.95 0.5 0.1; 0.5 0.2 0.7; 0.3 0.3 0.3; 0 0 0];
lab = {'MPIC fixed gains + reflex', 'VI-MPIC (first draft) + reflex', 'VI-MPIC v2 + reflex', ...
       'schedule after contact + reflex', 'VI-MPIC v2, no reflex', 'schedule after contact, no reflex', ...
       'MPIC 800 N/m, no reflex', 'MPIC 10000 N/m, no reflex'};
for it = [10 50]
    ax = nexttile(tl);
    hold(ax, 'on'); grid(ax, 'on');
    S = SS(SS.iterations == it, :);
    h = gobjects(1, 8);
    f1 = sortrows(S(startsWith(S.name, 'mpic_k') & endsWith(S.name, '_rfx'), :), 'e_track_rms_mm');
    h(1) = plot(ax, f1.e_track_rms_mm, f1.F_rmse_slide_mean_N, '--s', 'Color', [0.4 0.4 0.4], 'MarkerFaceColor', [0.8 0.8 0.8]);
    for i = 1:height(f1)
        text(ax, f1.e_track_rms_mm(i), f1.F_rmse_slide_mean_N(i), sprintf('  %s', erase(erase(f1.name{i}, 'mpic_k'), '_rfx')), 'FontSize', 7);
    end
    for i = 1:numel(mk)
        r = S(strcmp(S.name, mk{i}), :);
        if ~isempty(r)
            h(1 + i) = plot(ax, r.e_track_rms_mm, r.F_rmse_slide_mean_N, sym{i}, 'Color', col(i, :), 'MarkerSize', 10, 'LineWidth', 1.6);
        end
    end
    set(ax, 'XScale', 'log');
    xlabel(ax, 'free-motion tracking error, RMS [mm]');
    ylabel(ax, 'normal force RMS error while sliding [N]');
    title(ax, sprintf('sliding over an unknown 2 mm waviness, %d iterations', it));
    valid = isgraphics(h);
end
legend(ax, h(valid), lab(valid), 'Location', 'northeastoutside', 'Interpreter', 'none');
exportgraphics(fig, file, 'Resolution', 120);
end

function plotSlideRuns(P, file)
P.task.v_app = 0.2;
P.task.slide.enable = true;
P.task.T_sim = 4.0;
names = {'mpic_k800_rfx', 'mpic_k10000_rfx', 'sched_contact_rfx', 'vimpic_v2_rfx'};
V = vi_variants(P, names);
[~, o] = ismember(names, {V.name});
V = V(o);
runs = cell(1, numel(V));
for i = 1:numel(V)
    runs{i} = vi_simulate(P, V(i), 0);
end
vi_plot_runs(runs, P, file, [1.2 3.4]);
end
