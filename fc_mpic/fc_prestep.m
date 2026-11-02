function M = fc_prestep(P, V, J0, KP, KD, Fm, mu_hat)
%FC_PRESTEP Frozen model data of one MPC step of probfct_FC_MPIC.c.
%   M = fc_prestep(P, V, J0, KP, KD, Fm, mu_hat) is the prestep of the
%   proposal (Table 5) for variant V: J0 = J(q_k) frozen, KP and KD the
%   gains of the low-level joint PD, Fm = (fn, ft) the measured contact
%   force and mu_hat the friction estimate. M holds
%     W = Kd^-1 Kp, G = Kd^-1 J0' B, BJ = B' J0      (joint row, reference rate)
%     Kh = (B' C B + D)^-1, C = J0 Kp^-1 J0'         (stick stiffness, eq. 2.3)
%     kbn = 1 / (n' C n + 1/k_n)                     (Gold et al., Prop. 2.1)
%     mu                                             (mu_k of eq. 2.9)
%   and the model settings used by fc_model_step. Without coupling
%   (V.opt.coupling = false) B' C B is made diagonal (ablation FC-nc).

B = [0 1; 1 0];                         % phi = (ft, fn) = B' F, F in base axes
C = J0 * (KP \ J0');                    % controller compliance, Gold et al. eq. (11)
Ch = B' * C * B;
if ~V.opt.coupling
    Ch = diag(diag(Ch));
end
Kh = inv(Ch + diag([1 / P.fc.k_t, 1 / P.ke_model]));
M.Kh  = (Kh + Kh') / 2;
M.kbn = 1 / (Ch(2, 2) + 1 / P.ke_model);
M.W   = KD \ KP;
M.G   = KD \ (J0' * B);
M.BJ  = B' * J0;
M.J0  = J0;

mu = mu_hat;
% measurement-consistent cone, eq. (2.9): the measured force lies in the cone
if V.model == 1 && V.opt.cone && Fm(1) > P.fc.f_min
    mu = max(mu, abs(Fm(2)) / Fm(1));
end
% Prop. 2.3 needs ktt > mu |knt|; mu_k is kept below this bound (safeguard,
% counted by fc_simulate)
M.capped = false;
if V.model == 1 && M.Kh(2, 1) ~= 0
    mu_max = P.fc.painleve * M.Kh(1, 1) / abs(M.Kh(2, 1));
    M.capped = mu > mu_max;
    mu = min(mu, mu_max);
end
M.mu    = mu;
M.model = V.model;
M.epsf  = P.fc.eps_f;
M.beta  = P.fc.beta;
M.epsv  = P.fc.eps_v;
M.dt    = P.mpc.dt;
end
