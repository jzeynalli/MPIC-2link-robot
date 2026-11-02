function out = fc_prediction(make_figure)
%FC_PREDICTION Open-loop prediction accuracy of the force models over the MPC horizon (proposal Sec. 3.2).
%   Along closed-loop sanding runs (joint PD, Qp_y = 1e5, 10 iterations,
%   driven by FC-MPIC and by MPIC), each model is rolled out from the
%   state at t_k with the inputs that were actually applied afterwards and
%   compared with what the plant did over the next 0.5 s: tangential tool
%   position, friction force and normal force. Models: Gold et al. (MPIC),
%   pure sliding, FC-MPIC without coupling, FC-MPIC; all in discrete time
%   with the true friction coefficient. Reversal windows: |dpy*/dt| at t_k
%   below 40 % of its peak.
%   Both MPIC and FC-MPIC freeze J0 = J(q_k) over the horizon. The rows
%   "J(q) updated" re-linearise the model along the predicted joints
%   (fc_prestep at every predicted step, mu_k of t_k): an analysis tool,
%   not a controller, that separates the friction model from the frozen
%   Jacobian.
%   out = fc_prediction(true) also saves docs/figures/fc_prediction.png.

if nargin < 1
    make_figure = false;
end
here = fileparts(mfilename('fullpath'));
old = cd(here);
cleanup = onCleanup(@() cd(old));

models = {'mpic_d', 'slide', 'fc_nc', 'fc', 'mpic_d', 'fc'};
labels = {'Gold et al.', 'pure sliding', 'FC-nc', 'FC', 'Gold, J(q) upd.', 'FC, J(q) upd.'};
keys   = {'gold', 'slide', 'fc_nc', 'fc', 'gold_J', 'fc_J'};
upd    = [false false false false true true];
Np = 50;                                % 0.5 s
at = [10 30 50];                        % report after 0.1, 0.3, 0.5 s
out = struct();
for fcase = {'nominal', 'abrasive'}
    for driver = {'fc', 'mpic'}
        P = fc_params(fcase{1});
        P.cost.Qp = [1e5 1e5];
        R = fc_simulate(P, fc_variants(P, driver), 1);
        L = R.log;
        N = numel(L.t);
        k0 = find(L.t >= P.task.t_metric, 1);
        ks = k0:(N - Np);
        tau = L.t(ks) - P.task.t0;
        w = 2 * pi / P.task.T;
        rev = abs(P.task.A * w * sin(w * tau)) < P.task.rev_frac * P.task.A * w;
        E = struct();
        for m = 1:numel(models)
            V = fc_variants(P, models(m));
            ey = zeros(numel(ks), Np);  et = ey;  en = ey;
            for j = 1:numel(ks)
                k = ks(j);
                q = L.q(k, :)';
                [~, J0] = vi_kin(q, P.robot);
                KP = reshape(L.KP(k, :), 2, 2)';
                Fk = L.F(k, :)';                    % true (fn, ft) at t_k
                Md = fc_prestep(P, V, J0, KP, KP / P.omega, Fk, P.fc.mu);
                x = [q; L.qd(k, :)'; Fk(2); Fk(1)];
                for i = 1:Np
                    if upd(m) && i > 1
                        [~, Jp] = vi_kin(x(1:2), P.robot);
                        mu_k = Md.mu;
                        Md = fc_prestep(P, V, Jp, KP, KP / P.omega, Fk, P.fc.mu);
                        Md.mu = mu_k;
                    end
                    x = fc_model_step(x, L.u(k + i - 1, :)', Md);
                    pp = vi_kin(x(1:2), P.robot);
                    ey(j, i) = 1e3 * (pp(2) - L.p(k + i, 2));
                    et(j, i) = x(5) - L.F(k + i, 2);
                    en(j, i) = x(6) - L.F(k + i, 1);
                end
            end
            rms = @(a, msk) sqrt(mean(a(msk, :).^2, 1));
            E.(keys{m}) = struct('y_all', rms(ey, true(size(rev))), 'y_rev', rms(ey, rev), ...
                                   'ft_all', rms(et, true(size(rev))), 'ft_rev', rms(et, rev), ...
                                   'fn_all', rms(en, true(size(rev))), 'fn_rev', rms(en, rev));
        end
        out.(fcase{1}).(driver{1}) = E;
        fprintf('\n%s, closed loop driven by %s: RMS prediction error after %s s (all samples | reversal windows)\n', ...
                fcase{1}, driver{1}, strjoin(arrayfun(@(a) sprintf('%.1f', a / 100), at, 'UniformOutput', false), ' / '));
        for m = 1:numel(models)
            e = E.(keys{m});
            fprintf('  %-16s y [mm] %s | %s   f_t [N] %s | %s   f_n [N] %s | %s\n', labels{m}, ...
                    num(e.y_all(at)), num(e.y_rev(at)), num(e.ft_all(at)), num(e.ft_rev(at)), num(e.fn_all(at)), num(e.fn_rev(at)));
        end
    end
end
if make_figure
    plotPrediction(out, keys, labels, fullfile(here, '..', 'docs', 'figures', 'fc_prediction.png'));
end
end

function s = num(v)
s = strjoin(arrayfun(@(a) sprintf('%5.2f', a), v, 'UniformOutput', false), ' ');
end

function plotPrediction(out, keys, labels, file)
fig = figure('Color', 'w', 'Position', [100 100 1150 620], 'Visible', 'off');
tl = tiledlayout(fig, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
col = [0.85 0.5 0.1; 0.4 0.4 0.4; 0.3 0.6 0.9; 0 0.3 0.75; 0.85 0.5 0.1; 0 0.3 0.75];
sty = {'-', '--', '-.', '-', ':', ':'};
fields = {'y_rev', 'ft_rev', 'fn_rev'};
ylab = {'tool position p_t [mm]', 'friction force f_t [N]', 'normal force f_n [N]'};
cases = {'nominal', 'abrasive'};
for c = 1:2
    E = out.(cases{c}).fc;
    for f = 1:3
        ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
        for m = 1:numel(keys)
            v = E.(keys{m}).(fields{f});
            plot(ax, 10 * (1:numel(v)), v, sty{m}, 'Color', col(m, :), 'LineWidth', 1.4);
        end
        xlabel(ax, 'prediction time [ms]');
        title(ax, sprintf('%s: %s', cases{c}, ylab{f}));
        if f == 1
            ylabel(ax, 'RMS prediction error, reversal windows');
        end
    end
end
legend(ax, labels, 'Location', 'northwest');
exportgraphics(fig, file, 'Resolution', 130);
end
