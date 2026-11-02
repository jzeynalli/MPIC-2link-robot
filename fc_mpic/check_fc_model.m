function out = check_fc_model(make_figure)
%CHECK_FC_MODEL Checks of the FC-MPIC force model against the plant and against its claims.
%   1. E1, friction loop (proposal Sec. 3.1, Prop. 2.4): the normal
%      reference is held and the tangential reference moves back and forth
%      (+-20 mm at 1 cm/s) on the plant, open loop. The plant's friction
%      and normal force are compared with the FC model (exact return map on
%      the reference increments) and with Gold's model (constant), and the
%      reversal travel and normal-force ratio with eq. (2.11)-(2.12).
%   2. Prop. 2.1: FC-MPIC with mu = 0 predicts the normal force of Gold et al.
%   3. Prop. 2.3: one exact return-map step equals the continuous
%      elasto-plastic solution (fine sub-steps) for a constant input.
%   4. fc_model_step (MATLAB) equals the compiled ffct for all models.
%   5. Classical MPIC in probfct_FC_MPIC.c (continuous time) gives the same
%      closed loop as the fixed-gain MPIC of probfct_VI_MPIC.c.
%   out = check_fc_model(true) also saves docs/figures/fc_friction_loop.png.

if nargin < 1
    make_figure = false;
end
here = fileparts(mfilename('fullpath'));
old = cd(here);
cleanup = onCleanup(@() cd(old));
out = struct();

for fcase = {'nominal', 'abrasive'}
    P = fc_params(fcase{1});
    E = frictionLoop(P);
    out.(fcase{1}) = E;
    fprintf(['E1 %-8s Kh = [%.0f %.0f; %.0f %.0f] N/m | f_t RMS error: FC %.2f N | f_n RMS error: FC %.2f N, ' ...
             'Gold %.2f N\n'], fcase{1}, E.Kh(1, 1), E.Kh(1, 2), E.Kh(2, 1), E.Kh(2, 2), E.ft_err_fc, E.fn_err_fc, E.fn_err_gold);
    fprintf(['   sliding levels: plant f_t %+.2f / %+.2f N, f_n %.2f / %.2f N (r_n %.3f) | FC model f_n %.2f / %.2f N ' ...
             '(r_n %.3f) | eq. (2.12) r_n %.3f\n'], E.plant.ft, E.plant.fn, E.plant.rn, E.model.fn, E.model.rn, E.rn_eq);
    fprintf('   reversal travel (95%% of the friction reversal): plant %.2f / %.2f mm, FC model %.2f / %.2f mm, eq. (2.11) %.2f mm\n', ...
            E.plant.lrev, E.model.lrev, E.lrev_eq);
end
if make_figure
    plotLoop(out.nominal, fullfile(here, '..', 'docs', 'figures', 'fc_friction_loop.png'));
end

% ---- 2. Prop. 2.1: mu = 0 gives Gold's normal force
P = fc_params('nominal');
Md = prestepAt(P, 'fc');
Mg = Md;  Mg.model = 0;
Md.mu = 0;
rng(2);
xf = [0; 0; 0; 0; 0; 10];  xg = xf;
dmax = 0;
for i = 1:300
    u = 0.2 * randn(2, 1);
    xf = fc_model_step(xf, u, Md);
    xg = fc_model_step(xg, u, Mg);
    dmax = max(dmax, abs(xf(6) - xg(6)));
end
fprintf('Prop. 2.1: mu = 0, 300 random steps: |fn(FC) - fn(Gold)| <= %.3g N, |ft(FC)| = %.3g N\n', dmax, abs(xf(5)));
out.prop21 = dmax;

% ---- 3. Prop. 2.3: exact return map against fine sub-steps
Kh = Md.Kh;  mu = 0.5;
phi0 = [4.9; 10];                       % inside the cone
dd = [3e-3; -0.5e-3];                   % reference increment that leaves it
phi1 = returnMap(phi0, dd, Kh, mu);
phi = phi0;
for i = 1:20000
    phi = returnMap(phi, dd / 20000, Kh, mu);
end
fprintf('Prop. 2.3: one step (%.6f, %.6f) N, 20000 sub-steps (%.6f, %.6f) N\n', phi1, phi);
out.prop23 = max(abs(phi1 - phi));

% ---- 4. MATLAB mirror against the compiled ffct
I = fc_index();
V = fc_variants(P, {'fc'});
[grampc, C] = fc_init_grampc(P, V, [0; 0; 0; 0; 0; 10], fc_vi_params(P));
up = grampc.userparam;
emax = 0;
for model = 0:2
    Mm = prestepAt(P, 'fc');
    Mm.model = model;
    up(I.model) = model;  up(I.disc) = 1;  up(I.gate) = 1;
    up(I.W) = rowMajor(Mm.W);  up(I.G) = rowMajor(Mm.G);  up(I.BJ) = rowMajor(Mm.BJ);
    up(I.Kh) = rowMajor(Mm.Kh);  up(I.kbn) = Mm.kbn;  up(I.mu) = Mm.mu;
    for i = 1:200
        x = [-0.7 + 0.1 * randn; 1.4 + 0.1 * randn; -0.7 + 0.1 * randn; 1.4 + 0.1 * randn; 6 * randn; 10 + 3 * randn];
        u = 0.5 * randn(2, 1);
        xc = C.ffct(0, x, u, [], grampc.param, up);
        emax = max(emax, max(abs(xc(:) - fc_model_step(x, u, Mm))));
    end
end
fprintf('fc_model_step vs compiled ffct (600 random states): max difference %.3g\n', emax);
out.mirror = emax;

% ---- 5. MPIC in probfct_FC_MPIC.c against the fixed-gain MPIC of probfct_VI_MPIC.c
P = fc_params('nominal');
P.ctrl.type = 'cart';
P.ctrl.kc = [3000; 3000];
P.task.T_sim = 3;
Ra = fc_simulate(P, fc_variants(P, {'mpic'}), 1);
Vb = fc_variants(P, {'vimpic'});
vb = vi_variants(P.vi, {'mpic_k3000'});
for f = {'family', 'gate', 'psi', 'stiff', 'k', 'k_soft', 'risk', 'cost'}
    Vb.(f{1}) = vb.(f{1});
end
Rb = fc_simulate(P, Vb, 1);
de = max(abs(Ra.fine.p(:) - Rb.fine.p(:)));
dF = max(abs(Ra.fine.F(:) - Rb.fine.F(:)));
fprintf('MPIC, FC problem vs VI problem with fixed gains (3 s): max |dp| = %.3g m, max |dF| = %.3g N\n', de, dF);
out.equiv = [de dF];
end

function E = frictionLoop(P)
% open loop on the plant: normal reference held, tangential reference
% +-amp at vref after 0.3 s (as hysteresis.py of the proposal, which uses
% +-12 mm; +-20 mm completes the longer reversal of the abrasive case)
amp = 0.02;  vref = 0.01;  ncyc = 2;  t_on = 0.3;
dt = P.mpc.dt;
period = 4 * amp / vref;
N = round((t_on + ncyc * period) / dt);
Fd = P.task.F_des;
p0 = [P.wall.x + Fd / P.wall.ke; 0];
S.q = vi_ik(p0, P.robot);  S.dq = [0; 0];  S.z = 0;  S.fn = Fd;  S.ft = 0;
[~, J0] = vi_kin(S.q, P.robot);
KP = diag(P.ctrl.kq);
qd = S.q + KP \ (J0' * [Fd; 0]);
V = fc_variants(P, {'fc'});
Md = fc_prestep(P, V, J0, KP, KP / P.omega, [Fd; 0], P.fric.mu_c);
Kh = Md.Kh;  mu = P.fric.mu_c;
phi = [0; Fd];                          % FC model state (ft, fn)
yr = 0;
T = zeros(N, 1);  Y = zeros(N, 1);  Fp = zeros(N, 2);  Fm = zeros(N, 2);
for k = 1:N
    t = (k - 1) * dt;
    tau = t - t_on;
    if tau < 0
        v = 0;
    elseif mod(tau, period) < period / 4 || mod(tau, period) >= 3 * period / 4
        v = vref;
    else
        v = -vref;
    end
    uq = J0 \ [0; v];
    T(k) = t;  Y(k) = yr;  Fp(k, :) = [S.ft S.fn];  Fm(k, :) = phi';
    [S, Fl] = fc_plant(S, qd, uq, J0, KP, [NaN; NaN], [0; 0], P); %#ok<ASGLU>
    qd = qd + dt * uq;
    yr = yr + dt * v;
    phi = returnMap(phi, Md.BJ * uq * dt, Kh, mu);
end
E.t = T;  E.y = 1e3 * Y;  E.plant_F = Fp;  E.model_F = Fm;  E.Kh = Kh;
t_cyc = t_on + (ncyc - 1) * period;     % start of the last cycle
sec = T >= t_cyc;
E.ft_err_fc   = sqrt(mean((Fp(sec, 1) - Fm(sec, 1)).^2));
E.fn_err_fc   = sqrt(mean((Fp(sec, 2) - Fm(sec, 2)).^2));
E.fn_err_gold = sqrt(mean((Fp(sec, 2) - Fd).^2));
E.plant = levels(T, Y, Fp, t_cyc, period);
E.model = levels(T, Y, Fm, t_cyc, period);
% Prop. 2.4: the first loading from ft = 0 sets the sliding normal force in +y,
% fn+ = Fd ktt / (ktt - mu knt); eq. (2.12) and (2.11) follow
E.rn_eq   = (Kh(1, 1) + mu * Kh(2, 1)) / (Kh(1, 1) - mu * Kh(2, 1));
fn_pos    = Fd * Kh(1, 1) / (Kh(1, 1) - mu * Kh(2, 1));
E.lrev_eq = 1e3 * 2 * mu * fn_pos / (Kh(1, 1) + mu * Kh(2, 1));
end

function Lv = levels(T, Y, F, t_cyc, period)
% sliding levels in +y and -y (mid-stroke of the last cycle) and the
% reference travel that completes 95 % of each friction reversal: top
% (+y -> -y) at a quarter of the cycle, bottom (-y -> +y) at three quarters
ph = T - t_cyc;
up = ph > 0.05 * period & ph < 0.2 * period;
dn = ph > 0.55 * period & ph < 0.7 * period;
Lv.ft = [mean(F(up, 1)) mean(F(dn, 1))];
Lv.fn = [mean(F(up, 2)) mean(F(dn, 2))];
Lv.rn = Lv.fn(1) / Lv.fn(2);
lr = zeros(1, 2);
t_rev = t_cyc + period * [0.25 0.75];
f_to = Lv.ft([2 1]);
for i = 1:2
    k0 = find(T >= t_rev(i) - 1e-9, 1);
    target = F(k0, 1) + 0.95 * (f_to(i) - F(k0, 1));
    k1 = k0 - 1 + find(sign(f_to(i) - F(k0, 1)) * (F(k0:end, 1) - target) >= 0, 1);
    if isempty(k1)
        lr(i) = NaN;
    else
        lr(i) = abs(1e3 * (Y(k1) - Y(k0)));
    end
end
Lv.lrev = lr;
end

function phi = returnMap(phi, dd, Kh, mu)
% exact elastic predictor / plastic corrector, eq. (2.4)-(2.7)
tr = phi + Kh * dd;
exc = abs(tr(1)) - mu * tr(2);
if exc > 0
    e = sign(tr(1));
    tr = tr - Kh(:, 1) * e * exc / (Kh(1, 1) - mu * e * Kh(2, 1));
end
phi = tr;
end

function Md = prestepAt(P, name)
% model data at the start pose of the sanding task
Fd = P.task.F_des;
q = vi_ik([P.wall.x + Fd / P.wall.ke; P.task.y0], P.robot);
[~, J0] = vi_kin(q, P.robot);
KP = diag(P.ctrl.kq);
Md = fc_prestep(P, fc_variants(P, {name}), J0, KP, KP / P.omega, [Fd; 0], P.fc.mu);
end

function plotLoop(E, file)
fig = figure('Color', 'w', 'Position', [100 100 1000 380], 'Visible', 'off');
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
sec = E.t >= E.t(end) - 4 * 0.02 / 0.01;
ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
plot(ax, E.y(sec), E.plant_F(sec, 1), 'r-', 'LineWidth', 1.2);
plot(ax, E.y(sec), E.model_F(sec, 1), 'b--', 'LineWidth', 1.2);
xlabel(ax, 'tangential reference [mm]');  ylabel(ax, 'friction force f_t [N]');
legend(ax, {'plant (LuGre)', 'FC model'}, 'Location', 'northwest');
ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
plot(ax, E.y(sec), E.plant_F(sec, 2), 'r-', 'LineWidth', 1.2);
plot(ax, E.y(sec), E.model_F(sec, 2), 'b--', 'LineWidth', 1.2);
yline(ax, E.model_F(1, 2), 'Color', [0.85 0.5 0.1], 'LineStyle', '-.', 'LineWidth', 1.2);
xlabel(ax, 'tangential reference [mm]');  ylabel(ax, 'normal force f_n [N]');
legend(ax, {'plant (LuGre)', 'FC model', 'Gold et al. (constant)'}, 'Location', 'northwest');
exportgraphics(fig, file, 'Resolution', 130);
end

function r = rowMajor(A)
r = reshape(A.', 1, []);
end
