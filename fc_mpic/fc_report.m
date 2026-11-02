function S = fc_report(make_figures)
%FC_REPORT Tables and figures of the FC-MPIC study from results/fc_*_it*.csv.
%   S = fc_report() reads the runs written by run_fc_benchmark, averages
%   the noise seeds (S: one row per controller, gain, case, tracking weight
%   and iteration count) and prints the tables of docs/fc-mpic.md.
%
%   Each controller traces a trade-off between tangential tracking (et) and
%   normal-force regulation (efn) over the tracking weight Qp_y. A
%   controller is better than a reference only if its trade-off curve lies
%   below the reference's. The tables therefore compare the curves: at the
%   tracking error of every non-dominated point of the reference, the force
%   error of the controller's own non-dominated curve (log-log
%   interpolation) relative to the reference's, efn(et_ref) / efn_ref - 1.
%   They give the median change, its range over the points [min, max] and
%   the number of reference points within the controller's tracking range.
%   fc_report(true) also writes docs/figures/fc_pareto_*.png.

if nargin < 1
    make_figures = false;
end
here = fileparts(mfilename('fullpath'));
files = dir(fullfile(here, 'results', 'fc_*_it*.csv'));
T = [];
for i = 1:numel(files)
    f = fullfile(files(i).folder, files(i).name);
    o = detectImportOptions(f, 'VariableNamingRule', 'preserve');
    if ismember('case', o.VariableNames)
        o.VariableNames{strcmp(o.VariableNames, 'case')} = 'fcase';
    end
    o = setvartype(o, {'set', 'ctrl', 'gain', 'fcase', 'name'}, 'string');
    T = [T; readtable(f, o)]; %#ok<AGROW>
end
S = summarise(T);

fprintf('\n## Joint PD of Gold et al. (kq = 120 Nm/rad)\n');
for it = unique(S.iterations)'
    for fcase = ["nominal", "abrasive"]
        for metric = ["rms", "rev"]
            Sj = S(S.set == "joint" & S.fcase == fcase & S.iterations == it, :);
            tradeTable(Sj, ["mpic", "mpic_d", "fc", "fc_nc", "fc_nocone", "slide"], metric, ...
                       sprintf('%s, %d iterations, %s', fcase, it, metric));
        end
    end
end

fprintf('\n## Joint PD: change of the tracking error of FC-MPIC at the force error of MPIC\n');
fprintf('| case | iterations | vs MPIC, all | vs MPIC, reversals | vs MPIC (discrete), all | vs MPIC (discrete), reversals |\n|---|---|---|---|---|---|\n');
for it = unique(S.iterations)'
    for fcase = ["nominal", "abrasive"]
        Sj = S(S.set == "joint" & S.fcase == fcase & S.iterations == it, :);
        a = Sj(Sj.name == "fc", :);
        cells = strings(1, 4);
        k = 0;
        for rn = ["mpic", "mpic_d"]
            b = Sj(Sj.name == rn, :);
            for metric = ["rms", "rev"]
                k = k + 1;
                cells(k) = tchange(a.("et_" + metric), a.("efn_" + metric), b.("et_" + metric), b.("efn_" + metric));
            end
        end
        fprintf('| %s | %d | %s |\n', fcase, it, strjoin(cells, ' | '));
    end
end

fprintf('\n## Joint stiffness (nominal), change of the force error at MPIC''s tracking\n');
fprintf('| joints [Nm/rad] | iterations | FC vs MPIC, all | FC vs MPIC, reversals | FC vs MPIC (discrete), all | FC vs MPIC (discrete), reversals |\n|---|---|---|---|---|---|\n');
for it = unique(S.iterations)'
    for g = ["60", "120", "400"]
        Sk = S((S.set == "kq" | S.set == "joint") & S.gain == g & S.fcase == "nominal" & S.iterations == it, :);
        if isempty(Sk)
            continue
        end
        fprintf('| %s | %d | %s | %s | %s | %s |\n', g, it, ...
                change(Sk, "fc", "mpic", "rms"), change(Sk, "fc", "mpic", "rev"), ...
                change(Sk, "fc", "mpic_d", "rms"), change(Sk, "fc", "mpic_d", "rev"));
    end
end

fprintf('\n## Stroke length (joint PD), change of the force error at MPIC''s tracking\n');
fprintf('| stroke | case | iterations | FC vs MPIC, all | FC vs MPIC, reversals | FC vs MPIC (discrete), all | FC vs MPIC (discrete), reversals |\n|---|---|---|---|---|---|---|\n');
for it = unique(S.iterations)'
    for fcase = ["nominal", "abrasive"]
        for st = ["joint", "stroke"]
            Sk = S(S.set == st & S.gain == "120" & S.fcase == fcase & S.iterations == it, :);
            if isempty(Sk)
                continue
            end
            fprintf('| %s | %s | %d | %s | %s | %s | %s |\n', strokeText(st), fcase, it, ...
                    change(Sk, "fc", "mpic", "rms"), change(Sk, "fc", "mpic", "rev"), ...
                    change(Sk, "fc", "mpic_d", "rms"), change(Sk, "fc", "mpic_d", "rev"));
        end
    end
end

fprintf('\n## Decoupled controller: Cartesian stiffness with the diagonal of the joint PD, no coupling\n');
fprintf('| case | iterations | controller | best et [mm] | best efn [N] | vs MPIC (joint PD), all | reversals | vs FC-MPIC (joint PD), all | reversals |\n|---|---|---|---|---|---|---|---|---|\n');
for it = unique(S.iterations)'
    for fcase = ["nominal", "abrasive"]
        Sd = S(S.set == "decoupled" & S.fcase == fcase & S.iterations == it, :);
        Sj = S(S.set == "joint" & S.fcase == fcase & S.iterations == it, :);
        if isempty(Sd)
            continue
        end
        for nm = ["mpic", "fc"]
            row = Sd(Sd.name == nm, :);
            mj = Sj(Sj.name == "mpic", :);
            fj = Sj(Sj.name == "fc", :);
            fprintf('| %s | %d | %s, decoupled (k = %s) | %.3f | %.3f | %s | %s | %s | %s |\n', fcase, it, label(nm), row.gain(1), ...
                    min(row.et_rms), min(row.efn_rms), ...
                    fchange(row.et_rms, row.efn_rms, mj.et_rms, mj.efn_rms), ...
                    fchange(row.et_rev, row.efn_rev, mj.et_rev, mj.efn_rev), ...
                    fchange(row.et_rms, row.efn_rms, fj.et_rms, fj.efn_rms), ...
                    fchange(row.et_rev, row.efn_rev, fj.et_rev, fj.efn_rev));
        end
    end
end

fprintf('\n## Controller of VI-MPIC (Cartesian stiffness)\n');
for it = unique(S.iterations)'
    for fcase = ["nominal", "abrasive"]
        Sc = S(S.set == "cart" & S.fcase == fcase & S.iterations == it, :);
        if isempty(Sc)
            continue
        end
        fprintf('\n%s, %d iterations: change of the force error against MPIC\n', fcase, it);
        fprintf('| controller | reference | all | reversals | best et [mm] | best efn [N] |\n|---|---|---|---|---|---|\n');
        ref = Sc(Sc.name == "mpic", :);
        for g = unique(Sc.gain(Sc.name == "fc"))'
            Sg = Sc(Sc.gain == g, :);
            row = Sg(Sg.name == "fc", :);
            for rn = ["mpic", "mpic_d"]
                if any(Sg.name == rn)
                    fprintf('| FC-MPIC, k = %s | %s, same k | %s | %s | %.3f | %.3f |\n', g, label(rn), ...
                            change(Sg, "fc", rn, "rms"), change(Sg, "fc", rn, "rev"), min(row.et_rms), min(row.efn_rms));
                end
            end
        end
        for nm = ["fc", "vimpic"]
            % VI-MPIC's curve is short: the MPIC curve is evaluated at its points
            row = Sc(Sc.name == nm, :);
            own = nm == "vimpic";
            fprintf('| %s, all gains | MPIC, all fixed gains | %s | %s | %.3f | %.3f |\n', label(nm), ...
                    fchange(row.et_rms, row.efn_rms, ref.et_rms, ref.efn_rms, own), ...
                    fchange(row.et_rev, row.efn_rev, ref.et_rev, ref.efn_rev, own), min(row.et_rms), min(row.efn_rms));
        end
        fprintf('| MPIC, all fixed gains | | | | %.3f | %.3f |\n', min(ref.et_rms), min(ref.efn_rms));
    end
end

fprintf('\n## Worst normal-force error over the tracking weights (RMS [N] / largest deviation [N] / share of time without contact)\n');
fprintf('| low-level controller | case | iterations | gain | MPIC | MPIC, discrete time | FC-MPIC | VI-MPIC (first draft) |\n|---|---|---|---|---|---|---|---|\n');
for st = ["joint", "decoupled", "cart"]
    for it = unique(S.iterations)'
        for fcase = ["nominal", "abrasive"]
            Ss = S(S.set == st & S.fcase == fcase & S.iterations == it & S.Qp <= 5e5, :);
            gains = unique(Ss.gain(Ss.name ~= "vimpic"))';
            if st == "cart"
                gains = [gains, "planned"]; %#ok<AGROW>
            end
            for g = gains
                cells = strings(1, 4);
                nms = ["mpic", "mpic_d", "fc", "vimpic"];
                for i = 1:4
                    r = Ss(Ss.gain == g & Ss.name == nms(i), :);
                    if ~isempty(r)
                        cells(i) = sprintf('%.2f / %.1f / %.3f', max(r.efn_rms), max(r.efn_max), max(r.lost));
                    end
                end
                if any(cells ~= "")
                    fprintf('| %s | %s | %d | %s | %s |\n', st, fcase, it, g, strjoin(cells, ' | '));
                end
            end
        end
    end
end

fprintf('\n## Unknown friction coefficient (joint PD, nominal, true mu_c = 0.5)\n');
fprintf('| controller | iterations | Qp | final mu_hat | et RMS [mm] | et rev [mm] | efn RMS [N] | efn rev [N] |\n|---|---|---|---|---|---|---|---|\n');
for it = unique(S.iterations)'
    for qp = [2e4 1e5]
        for nm = ["mpic", "fc", "fc_mu03", "fc_mu03_est", "fc_mu08", "fc_mu08_est"]
            r = S((S.set == "mu" | S.set == "joint") & S.fcase == "nominal" & S.gain == "120" & S.name == nm & ...
                  S.iterations == it & S.Qp == qp, :);
            if isempty(r)
                continue
            end
            fprintf('| %s | %d | %g | %s | %.3f | %.3f | %.3f | %.3f |\n', label(nm), it, qp, muText(nm, r.mu_end), ...
                    r.et_rms, r.et_rev, r.efn_rms, r.efn_rev);
        end
    end
end

fprintf('\n## GRAMPC time per step [ms], median over all runs (benchmarks ran in parallel)\n');
fprintf('| controller | 10 iterations | 50 iterations |\n|---|---|---|\n');
for nm = ["mpic", "mpic_d", "fc", "slide", "vimpic"]
    c = arrayfun(@(it) median(T.cpu_med(T.name == nm & T.iterations == it)), [10 50]);
    fprintf('| %s | %.2f | %.2f |\n', label(nm), c);
end

if make_figures
    figdir = fullfile(here, '..', 'docs', 'figures');
    plotJoint(S, "rms", fullfile(figdir, 'fc_pareto_joint.png'));
    plotJoint(S, "rev", fullfile(figdir, 'fc_pareto_joint_rev.png'));
    plotCart(S, fullfile(figdir, 'fc_pareto_cart.png'));
end
end

function S = summarise(T)
keys = {'set', 'ctrl', 'gain', 'fcase', 'name', 'Qp', 'iterations'};
[g, K] = findgroups(T(:, keys));
S = K;
vals = {'et_rms', 'et_max', 'et_rev', 'et_mid', 'efn_rms', 'efn_max', 'efn_rev', 'efn_mid', 'lost', 'rn', ...
        'fn_pos', 'fn_neg', 'ft_pos', 'ft_neg', 'mu_end', 'cpu_med'};
for v = vals
    S.(v{1}) = splitapply(@mean, T.(v{1}), g);
end
S.et_std  = splitapply(@std, T.et_rms, g);
S.efn_std = splitapply(@std, T.efn_rms, g);
S.n_seeds = splitapply(@numel, T.et_rms, g);
end

function tradeTable(Sj, names, metric, ttl)
% trade-off points per tracking weight and the change against MPIC
et = "et_" + metric;  ef = "efn_" + metric;
qps = unique(Sj.Qp)';
fprintf('\n%s: tangential error [mm] / normal-force error [N]\n', ttl);
fprintf('| controller |%s vs MPIC | vs MPIC (discrete) |\n', sprintf(' Qp = %g |', qps));
fprintf('|---|%s---|---|\n', repmat('---|', 1, numel(qps)));
for nm = names
    r = Sj(Sj.name == nm, :);
    if isempty(r)
        continue
    end
    cells = strings(1, numel(qps));
    for i = 1:numel(qps)
        k = r.Qp == qps(i);
        if any(k)
            cells(i) = sprintf('%.3f / %.3f', r.(et)(k), r.(ef)(k));
        end
    end
    fprintf('| %s | %s | %s | %s |\n', label(nm), strjoin(cells, ' | '), change(Sj, nm, "mpic", metric), ...
            change(Sj, nm, "mpic_d", metric));
end
end

function s = change(Sx, nm, ref, metric)
% change of the force error of NM against REF at REF's tracking (see header)
if nm == ref
    s = "-";
    return
end
a = Sx(Sx.name == nm, :);
b = Sx(Sx.name == ref, :);
if isempty(a) || isempty(b)
    s = "";
    return
end
s = fchange(a.("et_" + metric), a.("efn_" + metric), b.("et_" + metric), b.("efn_" + metric));
end

function s = fchange(et, ef, et_ref, ef_ref, at_own)
% formatted change; at_own = true evaluates the reference curve at the
% controller's own points instead (for a controller with a short curve)
if nargin < 5
    at_own = false;
end
if at_own
    [d, n, lo, hi] = frontChange(et_ref, ef_ref, et, ef);
    [d, lo, hi] = deal(1 / (1 + d) - 1, 1 / (1 + hi) - 1, 1 / (1 + lo) - 1);
else
    [d, n, lo, hi] = frontChange(et, ef, et_ref, ef_ref);
end
if isnan(d)
    s = "n/a";
else
    s = sprintf('%+.0f %% [%+.0f, %+.0f] (%d)', 100 * d, 100 * lo, 100 * hi, n);
end
end

function s = tchange(et, ef, et_ref, ef_ref)
% the dual measure: the controller's tracking error at the force error of the
% reference's non-dominated points, et(efn_ref) / et_ref - 1 (median [min, max] (n))
[xt, yt] = front(et, ef);
[xr, yr] = front(et_ref, ef_ref);
in = yr >= min(yt) & yr <= max(yt);
s = "n/a";
if sum(in) == 0 || numel(xt) < 2
    return
end
[ys, o] = sort(yt);
r = exp(interp1(log(ys), log(xt(o)), log(yr(in)))) ./ xr(in) - 1;
s = sprintf('%+.0f %% [%+.0f, %+.0f] (%d)', 100 * median(r), 100 * min(r), 100 * max(r), numel(r));
end

function [d, n, lo, hi] = frontChange(et, ef, et_ref, ef_ref)
% the controller's non-dominated curve evaluated at the tracking errors of
% the reference's non-dominated points: median, number, min and max of
% efn(et_ref) / efn_ref - 1
[xt, yt] = front(et, ef);
[xr, yr] = front(et_ref, ef_ref);
in = xr >= min(xt) & xr <= max(xt);
n = sum(in);
d = NaN;  lo = NaN;  hi = NaN;
if n == 0 || numel(xt) < 2
    return
end
r = exp(interp1(log(xt), log(yt), log(xr(in)))) ./ yr(in) - 1;
d = median(r);
lo = min(r);
hi = max(r);
end

function [x, y] = front(et, ef)
% non-dominated points (smaller is better in both), sorted by et
keep = true(size(et));
for i = 1:numel(et)
    keep(i) = ~any(et <= et(i) & ef <= ef(i) & (et < et(i) | ef < ef(i)));
end
[x, o] = sort(et(keep));
y = ef(keep);
y = y(o);
end

function s = pct(d)
if isnan(d)
    s = "n/a";
else
    s = sprintf('%+.0f %%', 100 * d);
end
end

function s = strokeText(st)
if st == "stroke"
    s = "3 cm";
else
    s = "8 cm";
end
end

function s = muText(nm, mu_end)
if endsWith(nm, "_est")
    s = sprintf('%.3f', mu_end);
else
    s = "-";
end
end

function s = label(nm)
switch nm
    case "mpic",        s = "MPIC (Gold et al.)";
    case "mpic_d",      s = "MPIC, discrete time";
    case "fc",          s = "FC-MPIC";
    case "fc_nc",       s = "FC-MPIC without coupling";
    case "fc_nocone",   s = "FC-MPIC without consistent cone";
    case "slide",       s = "pure sliding (Kleff et al.)";
    case "vimpic",      s = "VI-MPIC (first draft)";
    case "fc_mu03",     s = "FC-MPIC, mu = 0.3";
    case "fc_mu03_est", s = "FC-MPIC, mu = 0.3, estimated";
    case "fc_mu08",     s = "FC-MPIC, mu = 0.8";
    case "fc_mu08_est", s = "FC-MPIC, mu = 0.8, estimated";
    otherwise,          s = nm;
end
end

function plotJoint(S, metric, file)
names = ["mpic", "mpic_d", "fc", "fc_nc", "fc_nocone", "slide"];
col = [0 0 0; 0.55 0.55 0.55; 0 0.3 0.75; 0.3 0.6 0.9; 0.55 0.35 0.8; 0.85 0.5 0.1];
sym = {'-o', '--s', '-d', '-.^', ':v', '--x'};
its = unique(S.iterations)';
fig = figure('Color', 'w', 'Position', [100 100 1150 400 * numel(its)], 'Visible', 'off');
tl = tiledlayout(fig, numel(its), 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for it = its
    for fcase = ["nominal", "abrasive"]
        ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
        Sj = S(S.set == "joint" & S.fcase == fcase & S.iterations == it, :);
        h = gobjects(1, numel(names));
        for n = 1:numel(names)
            r = sortrows(Sj(Sj.name == names(n), :), 'Qp');
            if isempty(r)
                continue
            end
            h(n) = plot(ax, r.("et_" + metric), r.("efn_" + metric), sym{n}, 'Color', col(n, :), 'LineWidth', 1.3, ...
                        'MarkerFaceColor', 'w');
            if names(n) == "mpic" || names(n) == "fc"
                for i = 1:height(r)
                    text(ax, r.("et_" + metric)(i), r.("efn_" + metric)(i), sprintf('  %g', r.Qp(i)), 'FontSize', 7, ...
                         'Color', col(n, :));
                end
            end
        end
        % MPIC with a Cartesian controller that removes the coupling
        r = sortrows(S(S.set == "decoupled" & S.fcase == fcase & S.iterations == it & S.name == "mpic", :), 'Qp');
        if ~isempty(r)
            hd = plot(ax, r.("et_" + metric), r.("efn_" + metric), '-o', 'Color', [0.1 0.6 0.3], 'LineWidth', 1.3, ...
                      'MarkerFaceColor', [0.1 0.6 0.3]);
        end
        set(ax, 'XScale', 'log');
        xlabel(ax, sprintf('tangential tracking error, %s [mm]', ttlMetric(metric)));
        ylabel(ax, sprintf('normal-force error, %s [N]', ttlMetric(metric)));
        title(ax, sprintf('%s friction, %d iterations per step', fcase, it));
    end
end
valid = isgraphics(h);
labs = arrayfun(@label, names(valid));
if exist('hd', 'var')
    h = [h(valid), hd];
    labs = [labs, "MPIC, decoupled Cartesian controller"];
else
    h = h(valid);
end
legend(ax, h, labs, 'Location', 'northeastoutside');
exportgraphics(fig, file, 'Resolution', 120);
end

function plotCart(S, file)
its = unique(S.iterations)';
fig = figure('Color', 'w', 'Position', [100 100 1150 400 * numel(its)], 'Visible', 'off');
tl = tiledlayout(fig, numel(its), 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for it = its
    for fcase = ["nominal", "abrasive"]
        ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
        Sc = S(S.set == "cart" & S.fcase == fcase & S.iterations == it, :);
        h = gobjects(1, 4);
        for g = unique(Sc.gain(Sc.name == "mpic"))'
            r = sortrows(Sc(Sc.name == "mpic" & Sc.gain == g, :), 'Qp');
            h(1) = plot(ax, r.et_rms, r.efn_rms, '-o', 'Color', [0.6 0.6 0.6], 'MarkerFaceColor', 'w');
            text(ax, r.et_rms(1), r.efn_rms(1), sprintf('  %s', g), 'FontSize', 7, 'Color', [0.4 0.4 0.4]);
            r = sortrows(Sc(Sc.name == "fc" & Sc.gain == g, :), 'Qp');
            h(2) = plot(ax, r.et_rms, r.efn_rms, '-d', 'Color', [0.3 0.6 0.9], 'MarkerFaceColor', 'w');
        end
        ref = Sc(Sc.name == "mpic", :);
        [x, y] = front(ref.et_rms, ref.efn_rms);
        h(3) = plot(ax, x, y, 'k-', 'LineWidth', 2);
        r = sortrows(Sc(Sc.name == "vimpic", :), 'Qp');
        h(4) = plot(ax, r.et_rms, r.efn_rms, '-p', 'Color', [0.85 0.1 0.1], 'MarkerSize', 9, 'LineWidth', 1.5);
        set(ax, 'XScale', 'log');
        xlabel(ax, 'tangential tracking error, RMS [mm]');
        ylabel(ax, 'normal-force error, RMS [N]');
        title(ax, sprintf('%s friction, %d iterations per step', fcase, it));
    end
end
legend(ax, h, {'MPIC, fixed k (labels: k [N/m])', 'FC-MPIC, fixed k', 'MPIC front', 'VI-MPIC (first draft)'}, ...
       'Location', 'northeastoutside');
exportgraphics(fig, file, 'Resolution', 120);
end

function s = ttlMetric(metric)
if metric == "rev"
    s = 'RMS at reversals';
else
    s = 'RMS';
end
end
