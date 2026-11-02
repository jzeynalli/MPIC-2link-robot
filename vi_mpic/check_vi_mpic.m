function C = check_vi_mpic()
%CHECK_VI_MPIC Model checks of the VI-MPIC draft.
%   (1) conditioned gate: g, Var and gamma against Monte Carlo
%   (2) overdamping floors k >= 4 omega^2 m_eff in free space and in contact
%   (3) E1: force under a stiffness ramp on the rigid-body plant, compared
%       with the force row of the model with and without the w.*sdot term
%   (4) E5: cost after the real-time iterations (10) versus 200 more
%   Writes docs/figures/vi_e1_force_model.png and returns the numbers in C.

here = fileparts(mfilename('fullpath'));
old = cd(here);
cleanup = onCleanup(@() cd(old));
addpath(fullfile(here, '..', 'external', 'grampc', 'matlab', 'mfiles'));
P = vi_params();
rng(0);

% ---- (1) conditioned gate against Monte Carlo
fprintf('(1) conditioned gate, sigma = %.0f mm\n', 1e3 * P.belief.sigma0);
sig = P.belief.sigma0;
phat = P.belief.p_hat;
C.gate = [];
for a = [-1 0 1]
    ell = phat + a * sig;
    po = phat + sig * randn(4e6, 1);
    po = po(po >= ell);
    for adv = [2e-3 10e-3]
        m = ell + adv;
        d = max(m - po, 0);
        [gam, g, v] = vi_gate(m, phat, sig, ell, 2);
        h = 1e-7;
        [~, gp] = vi_gate(m + h, phat, sig, ell, 2);
        [~, gm] = vi_gate(m - h, phat, sig, ell, 2);
        fprintf('    a = %+d, advance %4.1f mm: g %.4f mm (MC %.4f), sqrt(Var) %.4f mm (MC %.4f), gamma %.5f (FD of g %.5f)\n', ...
            a, 1e3 * adv, 1e3 * g, 1e3 * mean(d), 1e3 * sqrt(v), 1e3 * std(d), gam, (gp - gm) / (2 * h));
        C.gate(end + 1, :) = [a, adv, g, mean(d), sqrt(v), std(d), gam, (gp - gm) / (2 * h)];
    end
end

% ---- (2) overdamping floors along the approach line
xs = linspace(P.task.pA(1), P.task.p_cap, 12);
meff = zeros(size(xs));
for i = 1:numel(xs)
    q = vi_ik([xs(i); 0], P.robot);
    [~, J, M] = vi_kin(q, P.robot);
    meff(i) = 1 / ([1 0] * (J / M * J') * [1; 0]);
end
mmax = max(meff);
k_free = 4 * P.omega^2 * mmax;
% in contact the wall spring adds stiffness: (kx/omega)^2 >= 4 m (kx + ke)
k_contact = 2 * P.omega^2 * mmax * (1 + sqrt(1 + P.wall.ke / (P.omega^2 * mmax)));
fprintf('(2) m_eff along the normal: %.3f..%.3f kg; floor 4 omega^2 m_eff = %.0f N/m; in contact with ke = %.0f N/m: %.0f N/m (k_min = %.0f)\n', ...
    min(meff), mmax, k_free, P.wall.ke, k_contact, P.k_min);
C.meff = [min(meff) mmax];
C.k_floor = [k_free k_contact];

% ---- (3) E1: stiffness ramp with the reference held in contact
fprintf('(3) E1: force under a stiffness ramp, reference held 2 mm into the wall\n');
fig = figure('Color', 'w', 'Position', [100 100 900 360], 'Visible', 'off');
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
C.e1 = [];
for sdot = [-8 -40]
    [t, F, Fpsi, F0] = stiffnessRamp(P, sdot);
    err_psi = max(abs(F - Fpsi));
    err_frz = max(abs(F - F0));
    tail = t > t(end) - 0.05;
    fprintf('    sdot = %3d 1/s: force %.2f -> %.2f N; max error with w.*sdot %.2f N (end %.3f N), frozen Kbar %.2f N\n', ...
        sdot, F(1), F(end), err_psi, mean(abs(F(tail) - Fpsi(tail))), err_frz);
    C.e1(end + 1, :) = [sdot, F(1), F(end), err_psi, err_frz];
    ax = nexttile(tl);
    hold(ax, 'on'); grid(ax, 'on');
    plot(ax, t, F, 'k', 'LineWidth', 1.5);
    plot(ax, t, Fpsi, 'r--', 'LineWidth', 1.2);
    plot(ax, t, F0 * ones(size(t)), 'b:', 'LineWidth', 1.2);
    xlabel(ax, 'time [s]');
    ylabel(ax, 'contact force [N]');
    title(ax, sprintf('k_x 5000 -> 1000 N/m at ds/dt = %d 1/s, reference held', sdot), 'Interpreter', 'none');
    legend(ax, {'rigid-body plant', 'model with w.*ds/dt (notes eq. 12)', 'frozen Kbar (classical MPIC)'}, ...
        'Location', 'east', 'Interpreter', 'none');
end
figdir = fullfile(here, '..', 'docs', 'figures');
if ~exist(figdir, 'dir')
    mkdir(figdir);
end
exportgraphics(fig, fullfile(figdir, 'vi_e1_force_model.png'), 'Resolution', 130);

% ---- (4) E5: convergence of the real-time iteration
% share of the cost after the 10 real-time iterations that 200 further
% iterations from the same point remove: (J10 - J210) / J10
fprintf('(4) E5: cost after 10 iterations versus 10 + 200, v_app = 0.2 m/s, wall at the believed position\n');
P2 = P;
P2.task.v_app = 0.2;
ts = [0.5 1.2 1.3 1.35 1.4 1.5 1.6 1.8 2.2 2.8];
C.e5 = [];
for name = {'mpic_k3000', 'vimpic', 'vimpic_nopsi'}
    V = vi_variants(P2, name);
    J = zeros(numel(ts), 2);
    for i = 1:numel(ts)
        R = vi_simulate(P2, V, 0, ts(i));
        g = R.grampc;
        J(i, 1) = g.sol.J(1);
        g = CmexFiles.grampc_setopt_Cmex(g, 'ShiftControl', 'off');
        g = CmexFiles.grampc_setopt_Cmex(g, 'MaxGradIter', 200);
        g = CmexFiles.grampc_run_Cmex(g);
        J(i, 2) = g.sol.J(1);
    end
    share = (J(:, 1) - J(:, 2)) ./ max(J(:, 1), 1e-12);
    fprintf('    %-12s removable share of the cost: median %4.1f %%, max %4.1f %%  (J10: %s)\n', ...
        name{1}, 100 * median(share), 100 * max(share), mat2str(J(:, 1)', 2));
    C.e5(end + 1, :) = [median(share), max(share)];
end
end

function [t, F, Fpsi, F0] = stiffnessRamp(P, sdot)
% rigid-body plant in contact with the reference held fixed while the
% normal stiffness ramps from 5000 to 1000 N/m; same torque law as vi_simulate
R = P.robot;
h = R.h;
k0 = 5000;  k1 = 1000;  ke = P.wall.ke;  ce = P.wall.ce;  pw = P.belief.p_hat;
F0 = 5;
kb0 = ke * k0 / (ke + k0);
q = vi_ik([pw + F0 / ke; 0], R);
qd = vi_ik([pw + F0 / kb0; 0], R);
[~, J0] = vi_kin(q, R);
dq = [0; 0];
T = 0.1 + log(k0 / k1) / abs(sdot) + 0.25;
N = round(T / h);
t = (0:N - 1)' * h;
F = zeros(N, 1);
s = log(k0);
for n = 1:N
    tn = t(n);
    if tn > 0.1
        s = max(log(k1), log(k0) + sdot * (tn - 0.1));
    end
    kt = [exp(s); k0];
    KP = J0' * diag(kt) * J0;
    KD = KP / P.omega;
    [p, J, M] = vi_kin(q, R);
    d = p(1) - pw;
    Fc = 0;
    if d > 0
        Fc = max(0, ke * d + ce * (J(1, :) * dq));
    end
    F(n) = Fc;
    dq = (M + h * KD) \ (M * dq + h * (KP * (qd - q) + J' * [-Fc; 0]));
    q = q + h * dq;
end
% force row of the model with qd fixed: dF/dt = kbar w sdot, i.e. F = kbar(s(t)) d
st = log(k0) + sdot * max(t - 0.1, 0);
st = max(st, log(k1));
kbt = ke * exp(st) ./ (ke + exp(st));
Fpsi = F0 * kbt / kb0;
end
