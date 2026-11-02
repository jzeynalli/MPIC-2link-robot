function fc_plot_runs(fcase, Qp, iters, file)
%FC_PLOT_RUNS Time histories of MPIC and FC-MPIC over one stroke period.
%   fc_plot_runs(fcase, Qp, iters, file) simulates MPIC and FC-MPIC with
%   the joint PD (and MPIC with the decoupled Cartesian controller of the
%   'decoupled' benchmark set) in friction case FCASE with tracking weight
%   QP and ITERS gradient iterations, and plots the tangential tracking
%   error, the normal force and the friction force over the last stroke
%   period. Dotted lines mark the stroke reversals. Saves FILE if given.

if nargin < 1
    fcase = 'nominal';
end
if nargin < 2
    Qp = 1e5;
end
if nargin < 3
    iters = 10;
end
here = fileparts(mfilename('fullpath'));
old = cd(here);
cleanup = onCleanup(@() cd(old));

P = fc_params(fcase);
P.cost.Qp = [Qp Qp];
P.mpc.MaxGradIter = iters;
runs = {fc_simulate(P, fc_variants(P, {'mpic'}), 1), fc_simulate(P, fc_variants(P, {'fc'}), 1)};
names = {'MPIC (joint PD)', 'FC-MPIC (joint PD)'};
% decoupled Cartesian controller with the diagonal stiffness of the joint PD
q0 = vi_ik([P.wall.x + P.task.F_des / P.wall.ke; P.task.y0], P.robot);
[~, J] = vi_kin(q0, P.robot);
Pd = P;
Pd.ctrl.type = 'cart';
Pd.ctrl.kc = round(1 ./ diag(J * (diag(P.ctrl.kq) \ J')));
runs{end + 1} = fc_simulate(Pd, fc_variants(Pd, {'mpic'}), 1);
names{end + 1} = 'MPIC (decoupled controller)';

Tk = P.task;
w = 2 * pi / Tk.T;
t_end = Tk.T_sim;
win = [t_end - Tk.T, t_end];
col = [0 0 0; 0 0.3 0.75; 0.1 0.6 0.3];
fig = figure('Color', 'w', 'Position', [100 100 900 720], 'Visible', 'off');
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = gobjects(1, 3);
for i = 1:3
    ax(i) = nexttile(tl);
    hold(ax(i), 'on');
    grid(ax(i), 'on');
end
for r = 1:numel(runs)
    Fn = runs{r}.fine;
    tau = Fn.t - Tk.t0;
    ys = Tk.y0 + (tau > 0) .* Tk.A .* (1 - cos(w * tau));
    k = Fn.t >= win(1) & Fn.t <= win(2);
    plot(ax(1), Fn.t(k), 1e3 * (Fn.p(k, 2) - ys(k)), 'Color', col(r, :), 'LineWidth', 1.2);
    plot(ax(2), Fn.t(k), Fn.F(k, 1), 'Color', col(r, :), 'LineWidth', 1.2);
    plot(ax(3), Fn.t(k), Fn.F(k, 2), 'Color', col(r, :), 'LineWidth', 1.2);
end
% reversals: dpy*/dt = 0
t_rev = Tk.t0 + (0:ceil(Tk.T_sim / Tk.T * 2)) * Tk.T / 2;
t_rev = t_rev(t_rev >= win(1) & t_rev <= win(2));
for i = 1:3
    xline(ax(i), t_rev, 'k:');
    xlim(ax(i), win);
end
yline(ax(2), Tk.F_des, 'k--');
ylabel(ax(1), 'tangential error [mm]');
ylabel(ax(2), 'normal force f_n [N]');
ylabel(ax(3), 'friction force f_t [N]');
xlabel(ax(3), 'time [s]');
legend(ax(1), names, 'Location', 'best');
title(ax(1), sprintf('%s friction, Q_p = %g, %d iterations; dotted: stroke reversals', fcase, Qp, iters));
if nargin > 3 && ~isempty(file)
    exportgraphics(fig, file, 'Resolution', 130);
end
end
