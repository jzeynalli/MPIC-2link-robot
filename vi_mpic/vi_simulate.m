function R = vi_simulate(P, V, dP, t_stop)
%VI_SIMULATE Closed loop of one controller variant against the rigid-body plant.
%   R = vi_simulate(P, V, dP) runs variant V (see vi_variants) with the wall
%   dP metres behind its believed position P.belief.p_hat and returns the
%   logs (R.log, R.fine) and the metrics (R.metrics).
%
%   Task: hold P.task.pA while a disturbance force acts, then approach the
%   wall along +x with the desired frame moving at P.task.v_app (guarded
%   approach) and press with P.task.F_des once contact is measured.
%
%   Plant (not the MPC model): rigid-body 2-link arm in the horizontal
%   plane with the low-level torque law of the notes, eq. (38),
%     M(q) ddq = Kp(t) (qd(t) - q) + Kd(t) (dqd_k - dq) + J(q)' (F_dist - F_c)
%   Kp(t) = J0' diag(exp(s(t))) J0, Kd = Kp / omega, Coriolis terms
%   compensated, qd(t) = qd_k + dqd_k (t - t_k), s(t) = s_k + ds_k (t - t_k).
%   The wall is a spring-damper F_c = max(0, ke d + ce dd) for d > 0.
%
%   R = vi_simulate(P, V, dP, t_stop) stops after the MPC step at t_stop and
%   returns the GRAMPC struct of that step in R.grampc (for debugging).

if nargin < 4
    t_stop = inf;
end
% per-variant cost overrides
f = fieldnames(V.cost);
for i = 1:numel(f)
    P.cost.(f{i}) = V.cost.(f{i});
end
I = vi_index();
dt = P.mpc.dt;  h = P.robot.h;  nsub = round(dt / h);
N = round(P.task.T_sim / dt);
Rb = P.robot;
l1 = Rb.l(1);  l2 = Rb.l(2);
a1 = Rb.I(1) + Rb.I(2) + Rb.m(1) * Rb.r(1)^2 + Rb.m(2) * (l1^2 + Rb.r(2)^2);
a2 = Rb.m(2) * l1 * Rb.r(2);
a3 = Rb.I(2) + Rb.m(2) * Rb.r(2)^2;
p_wall = P.belief.p_hat + dP;
ke = P.wall.ke;  ce = P.wall.ce;  om = P.omega;
s_min = log(P.k_min);  s_max = log(P.k_max);
Fdist = P.task.dist_F;  tdist = P.task.dist_window;

% initial state: at rest at pA, reference on the robot
q  = vi_ik(P.task.pA, Rb);
dq = [0; 0];
qd = q;
s  = log(V.k) * [1; 1];
grampc = vi_init_grampc(P, V, [q; qd; 0; 0; s]);

% surface belief, analysis Table 5
phat = P.belief.p_hat;  sig = P.belief.sigma0;  ell = -1e3;
had_contact = false;

L.t = (0:N-1)' * dt;
L.q = zeros(N, 2);  L.qd = zeros(N, 2);  L.p = zeros(N, 2);  L.pdes = zeros(N, 2);
L.F = zeros(N, 1);  L.Fmodel = zeros(N, 1);  L.k = zeros(N, 2);  L.u = zeros(N, 4);
L.gamma = zeros(N, 1);  L.cpu = zeros(N, 1);  L.J = zeros(N, 1);
L.phat = zeros(N, 1);  L.sigma = zeros(N, 1);  L.depth = zeros(N, 1);
Fn.t = (0:N * nsub - 1)' * h;  Fn.F = zeros(N * nsub, 1);  Fn.x = zeros(N * nsub, 1);

for k = 1:N
    t = (k - 1) * dt;

    % ---- measurement (perfect force sensor)
    [p, J] = vi_kin(q, Rb);
    Fc = wallForce(p(1), J(1, :) * dq);
    in_contact = Fc > P.F_contact;

    % ---- belief update
    if in_contact
        had_contact = true;
        phat = p(1) - Fc / P.ke_model;
        sig = P.belief.sigma_meas;
        ell = -1e3;
    elseif had_contact
        sig = P.belief.sigma_lost;
        ell = p(1);
    else
        ell = max(ell, p(1));
    end

    % ---- primitive: guarded approach until contact, then force. With the
    % measured gate the approach resumes after a loss of contact; with the
    % probabilistic gates the force cost itself drives the return
    force_prim = in_contact || (had_contact && V.gate > 0);

    % ---- heuristic schedules set the normal stiffness before the solve
    pd_k = vi_kin(qd, Rb);
    m_k = pd_k(1);
    switch V.stiff
        case 'sched_force'
            lam = double(had_contact);
            s(1) = schedule(s(1), lam, V, P);
        case 'sched_dist'
            x_far  = P.belief.p_hat - 3 * P.belief.sigma0;
            x_near = P.belief.p_hat - P.belief.sigma0;
            lam = min(max((m_k - x_far) / (x_near - x_far), 0), 1);
            if had_contact
                lam = 1;
            end
            s(1) = schedule(s(1), lam, V, P);
    end

    % ---- prestep: frozen model data in userparam
    J0 = J;
    Ji = inv(J0);
    up = grampc.userparam;
    up(I.J0)    = [J0(1, 1) J0(1, 2) J0(2, 1) J0(2, 2)];
    up(I.J0inv) = [Ji(1, 1) Ji(1, 2) Ji(2, 1) Ji(2, 2)];
    up(I.phat)  = phat;
    up(I.sigma) = sig;
    up(I.ell)   = ell;
    up(I.mk)    = m_k;
    up(I.qdk)   = qd';
    up(I.gmeas) = double(in_contact);
    % hold: EE position; guarded approach: velocity v_app of the desired
    % frame along the normal, the same for every variant; force primitive:
    % force instead of position along the normal. The force cost is off
    % before contact, otherwise the expected-force model pulls the reference
    % towards the believed wall (no approach-speed constraint, notes eq. 32,
    % in this draft)
    up(I.Qr) = [0 0];
    up(I.vapp) = P.task.v_app;
    if force_prim
        up(I.Qp) = [0 P.cost.Qp(2)];
        up(I.QF) = P.cost.QF;
        up(I.Qv) = 0;
    elseif t >= P.task.t_app
        up(I.Qp) = [0 P.cost.Qp(2)];
        up(I.QF) = [0 0];
        up(I.Qv) = P.cost.Qv;
    else
        up(I.Qp) = P.cost.Qp;
        up(I.QF) = [0 0];
        up(I.Qv) = 0;
    end
    % EE target: pA while holding. Its ramp only matters in the hold phase,
    % where the horizon already sees the start of the approach (and the MPC
    % starts to move up to Thor early); afterwards the normal position
    % weights are zero
    pdes_x = min(P.task.pA(1) + P.task.v_app * max(0, t - P.task.t_app), P.task.p_cap);
    up(I.pdes)   = [pdes_x P.task.pA(2)];
    up(I.vdes)   = P.task.v_app;
    up(I.tdelay) = max(0, P.task.t_app - t);
    grampc.userparam = up;

    % ---- force state: measured in contact; before contact the conditioned
    % model starts from its own expectation kbar g(m_k) (analysis 3.2, remark)
    kx = exp(s(1));
    kb = P.ke_model * kx / (P.ke_model + kx);
    [gam_k, g_k] = vi_gate(m_k, phat, sig, ell, V.gate, double(in_contact));
    if in_contact
        F0 = [Fc; 0];
    elseif V.gate == 2
        F0 = [kb * g_k; 0];
    else
        F0 = [0; 0];
    end

    grampc = CmexFiles.grampc_setparam_Cmex(grampc, 'x0', [q; qd; F0; s]);
    grampc = CmexFiles.grampc_setparam_Cmex(grampc, 't0', t);
    [grampc, cpu] = CmexFiles.grampc_run_Cmex(grampc);
    u = grampc.sol.unext(:);
    if ~strcmp(V.stiff, 'vi')
        u(3:4) = 0;
    end

    L.q(k, :) = q';  L.qd(k, :) = qd';  L.p(k, :) = p';  L.pdes(k, :) = [pdes_x P.task.pA(2)];
    L.F(k) = Fc;  L.Fmodel(k) = F0(1);  L.k(k, :) = exp(s)';  L.u(k, :) = u';
    L.gamma(k) = gam_k;  L.cpu(k) = cpu;  L.J(k) = grampc.sol.J(1);
    L.phat(k) = phat;  L.sigma(k) = sig;  L.depth(k) = m_k;
    if t >= t_stop - 1e-9
        R.grampc = grampc;
        R.log = L;
        R.p_wall = p_wall;
        return
    end

    % ---- plant over [t, t + dt], semi-implicit Euler (damping implicit)
    uq = u(1:2);  us = u(3:4);
    for j = 1:nsub
        tau = (j - 1) * h;
        qdt = qd + uq * tau;
        kt = exp(min(max(s + us * tau, s_min), s_max));
        KP = J0' * [kt(1) 0; 0 kt(2)] * J0;
        KD = KP / om;

        c2 = cos(q(2));
        M = [a1 + 2 * a2 * c2, a3 + a2 * c2; a3 + a2 * c2, a3];
        s1 = sin(q(1));  c1 = cos(q(1));
        s12 = sin(q(1) + q(2));  c12 = cos(q(1) + q(2));
        px = l1 * c1 + l2 * c12;
        Jq = [-l1 * s1 - l2 * s12, -l2 * s12; l1 * c1 + l2 * c12, l2 * c12];
        Fcj = wallForce(px, Jq(1, :) * dq);
        if t + tau >= tdist(1) && t + tau < tdist(2)
            Fe = Fdist - [Fcj; 0];
        else
            Fe = [-Fcj; 0];
        end

        dq = (M + h * KD) \ (M * dq + h * (KP * (qdt - q) + KD * uq + Jq' * Fe));
        q = q + h * dq;

        idx = (k - 1) * nsub + j;
        Fn.F(idx) = Fcj;
        Fn.x(idx) = px;
    end

    % ---- poststep: remember the reference and stiffness the low-level controller ran with
    qd = qd + dt * uq;
    s = min(max(s + dt * us, s_min), s_max);
end

R.variant = V;
R.offset = dP;
R.log = L;
R.fine = Fn;
R.metrics = vi_metrics(L, Fn, P);

    function F = wallForce(px, vx)
        d = px - p_wall;
        if d > 0
            F = max(0, ke * d + ce * vx);
        else
            F = 0;
        end
    end
end

function s = schedule(s, lam, V, P)
% log-linear blend between the stiff and the soft value; softening is
% immediate, stiffening respects the rate bound of the analysis (Prop. 3.4)
s_goal = (1 - lam) * log(V.k) + lam * log(V.k_soft);
if s_goal < s
    s = s_goal;
else
    s = min(s_goal, s + P.sdot_max * P.mpc.dt);
end
end
