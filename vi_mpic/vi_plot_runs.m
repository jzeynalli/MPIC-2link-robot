function fig = vi_plot_runs(runs, P, file, tlim)
%VI_PLOT_RUNS Time histories of several runs: EE position, force, normal stiffness.
%   vi_plot_runs(runs, P, file, tlim) saves the figure as PNG when FILE is
%   given and limits the time axis to TLIM.

fig = figure('Color', 'w', 'Position', [100 100 900 760], 'Visible', 'off');
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
cols = lines(numel(runs));

ax1 = nexttile(tl); hold(ax1, 'on'); grid(ax1, 'on');
ax2 = nexttile(tl); hold(ax2, 'on'); grid(ax2, 'on');
ax3 = nexttile(tl); hold(ax3, 'on'); grid(ax3, 'on');
names = cell(1, numel(runs));
for i = 1:numel(runs)
    R = runs{i};
    names{i} = sprintf('%s, \\Delta = %+g mm', strrep(R.variant.name, '_', '\_'), 1e3 * R.offset);
    plot(ax1, R.fine.t, 1e3 * (R.fine.x - P.belief.p_hat), 'Color', cols(i, :));
    plot(ax2, R.fine.t, R.fine.F, 'Color', cols(i, :));
    stairs(ax3, R.log.t, R.log.k(:, 1), 'Color', cols(i, :));
end
walls = unique(cellfun(@(r) r.offset, runs));
yline(ax1, 1e3 * walls, 'k--');
yline(ax2, P.task.F_des, 'k:');
ylabel(ax1, 'EE x - believed wall [mm]');
ylabel(ax2, 'contact force [N]');
ylabel(ax3, 'normal stiffness [N/m]');
set(ax3, 'YScale', 'log');
ylim(ax3, [0.8 * P.k_min 1.2 * P.k_max]);
xlabel(ax3, 'time [s]');
legend(ax2, names, 'Location', 'northwest', 'Interpreter', 'tex');
title(ax1, 'EE position along the wall normal (dashed: true wall)');
if nargin > 3 && ~isempty(tlim)
    xlim([ax1 ax2 ax3], tlim);
end

if nargin > 2 && ~isempty(file)
    exportgraphics(fig, file, 'Resolution', 130);
end
end
