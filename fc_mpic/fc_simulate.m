function R = fc_simulate(P, V, seed)
%FC_SIMULATE Closed loop of one controller variant in the sanding task.
%   R = fc_simulate(P, V, seed) runs variant V (see fc_variants) against
%   the rigid-body plant with a frictional wall (fc_plant) and returns the
%   logs (R.log at the MPC rate, R.fine at 1 kHz) and the metrics
%   (R.metrics). SEED sets the force-measurement noise: runs with the same
%   seed see the same noise sequence.
%
%   Task: the EE starts in static contact, pressing P.task.F_des on the
%   wall x = P.wall.x, and slides back and forth along the wall,
%   py* = y0 + A (1 - cos(2 pi (t - t0) / T)), while it keeps F_des.
%
%   Low-level controller: joint PD with Kp = diag(kq) (P.ctrl.type =
%   'joint', Gold et al. eq. 4) or Kp = J0' diag(k) J0 with J0 frozen per
%   MPC step ('cart', the controller of VI-MPIC, which plans k), Kd = Kp/omega.
%   The contact force is measured with Gaussian noise at the MPC rate, the
%   joints exactly.

if nargin < 3
    seed = 1;
end
vi = strcmp(V.problem, 'vi');
if vi && ~strcmp(P.ctrl.type, 'cart')
    error('fc_simulate: VI-MPIC plans a Cartesian stiffness, use P.ctrl.type = ''cart''');
end
I = fc_index();
Iv = vi_index();
Pv = fc_vi_params(P);
dt = P.mpc.dt;
N = round(P.task.T_sim / dt);
Rb = P.robot;
xw = P.wall.x;  om = P.omega;
Tk = P.task;  Fd = Tk.F_des;
s_min = log(Pv.k_min);  s_max = log(Pv.k_max);
noise = RandStream('mt19937ar', 'Seed', seed);
mu_hat = P.fc.mu;
if ~isnan(V.opt.mu)
    mu_hat = V.opt.mu;
end

% initial state: static contact at (x_wall + F_des/ke, y0), the reference
% deflected so that the joint PD holds F_des
p0 = [xw + Fd / P.wall.ke; Tk.y0];
S.q = vi_ik(p0, Rb);
S.dq = [0; 0];
S.z = 0;                                % LuGre bristle deflection
S.fn = Fd;                              % contact force the robot exerts on the wall
S.ft = 0;
[~, J] = vi_kin(S.q, Rb);
switch P.ctrl.type
    case 'joint'
        s = [NaN; NaN];
        KP = diag(P.ctrl.kq);
    case 'cart'
        if vi
            s = log(V.k) * [1; 1];
        else
            s = log(P.ctrl.kc(:));
        end
        KP = J' * diag(exp(s)) * J;
end
qd = S.q + KP \ (J' * [Fd; 0]);
if vi
    x0 = [S.q; qd; Fd; 0; s];
else
    x0 = [S.q; qd; 0; Fd];
end
[grampc, C] = fc_init_grampc(P, V, x0, Pv);

% surface belief of VI-MPIC (vi_simulate); the run starts in contact
phat = xw;  sig = Pv.belief.sigma_meas;  ell = -1e3;
had_contact = true;

L.t = (0:N-1)' * dt;
L.q = zeros(N, 2);  L.qd = zeros(N, 2);  L.dq = zeros(N, 2);  L.p = zeros(N, 2);
L.u = zeros(N, 2);  L.us = zeros(N, 2);  L.F = zeros(N, 2);  L.Fm = zeros(N, 2);
L.KP = zeros(N, 4);  L.k = zeros(N, 2);  L.mu = nan(N, 1);  L.muhat = zeros(N, 1);
L.contact = false(N, 1);  L.capped = false(N, 1);  L.cpu = zeros(N, 1);  L.J = zeros(N, 1);
nf = round(dt / 1e-3);                  % 1 kHz samples per MPC interval
Fn.t = (0:N * nf - 1)' * 1e-3;
Fn.p = zeros(N * nf, 2);  Fn.F = zeros(N * nf, 2);  Fn.v = zeros(N * nf, 2);

for k = 1:N
    t = (k - 1) * dt;
    q = S.q;
    [p, J] = vi_kin(q, Rb);

    % ---- measurement: contact force with noise, joints exact
    Fm = [S.fn; S.ft] + P.noise_F * randn(noise, 2, 1);
    in_contact = Fm(1) > P.F_contact;

    % ---- online friction estimate while sliding, eq. (2.10)
    if V.opt.estimate && abs(J(2, :) * S.dq) > P.fc.v_min && Fm(1) > P.fc.f_min
        mu_hat = mu_hat + P.fc.gain * (abs(Fm(2)) / Fm(1) - mu_hat);
    end

    % ---- low-level gains of this step, J0 frozen
    J0 = J;
    if strcmp(P.ctrl.type, 'cart')
        KP = J0' * diag(exp(s)) * J0;
    end
    KD = KP / om;

    if vi
        % ---- VI-MPIC, first draft (vi_simulate): belief, gate, force state
        if in_contact
            had_contact = true;
            phat = p(1) - Fm(1) / Pv.ke_model;
            sig = Pv.belief.sigma_meas;
            ell = -1e3;
        elseif had_contact
            sig = Pv.belief.sigma_lost;
            ell = p(1);
        else
            ell = max(ell, p(1));
        end
        force_prim = in_contact || (had_contact && V.gate > 0);
        pd_k = vi_kin(qd, Rb);
        m_k = pd_k(1);
        up = grampc.userparam;
        up(Iv.J0)    = rowMajor(J0);
        up(Iv.J0inv) = rowMajor(inv(J0));
        up(Iv.phat)  = phat;
        up(Iv.sigma) = sig;
        up(Iv.ell)   = ell;
        up(Iv.mk)    = m_k;
        up(Iv.qdk)   = qd';
        up(Iv.gmeas) = double(in_contact);
        up(Iv.Qr)    = [0 0];
        up(Iv.vapp)  = Pv.task.v_app;
        up(Iv.Qp)    = [0 Pv.cost.Qp(2)];
        if force_prim
            up(Iv.QF) = Pv.cost.QF;
            up(Iv.Qv) = 0;
        else
            up(Iv.QF) = [0 0];
            up(Iv.Qv) = Pv.cost.Qv;
        end
        up(Iv.pdes)   = [p0(1) Tk.y0];
        up(Iv.vdes)   = 0;
        up(Iv.tdelay) = 0;
        up(Iv.yt0)    = Tk.t0 - t;
        grampc.userparam = up;
        % in contact the force state is the measured force, now including
        % the friction along the wall, which the model holds (as Gold et al.)
        kx = exp(s(1));
        kb = Pv.ke_model * kx / (Pv.ke_model + kx);
        [~, g_k] = vi_gate(m_k, phat, sig, ell, V.gate, double(in_contact));
        if in_contact
            F0 = Fm;
        elseif V.gate == 2
            F0 = [kb * g_k; 0];
        else
            F0 = [0; 0];
        end
        x0 = [q; qd; F0; s];
    else
        % ---- prestep of the FC-MPIC problem: frozen J0, Kh, kbar_n, mu_k
        Md = fc_prestep(P, V, J0, KP, KD, Fm, mu_hat);
        up = grampc.userparam;
        up(I.gate)  = double(in_contact);
        up(I.W)     = rowMajor(Md.W);
        up(I.G)     = rowMajor(Md.G);
        up(I.BJ)    = rowMajor(Md.BJ);
        up(I.Kh)    = rowMajor(Md.Kh);
        up(I.kbn)   = Md.kbn;
        up(I.mu)    = Md.mu;
        up(I.J0)    = rowMajor(J0);
        up(I.J0inv) = rowMajor(inv(J0));
        up(I.yt0)   = Tk.t0 - t;
        % force along the normal in contact; after a loss of contact the
        % guarded approach of vi_simulate brings the reference back
        up(I.Qp) = [0 P.cost.Qp(2)];
        if in_contact
            up(I.QF) = P.cost.QF;
            up(I.Qv) = 0;
            x0 = [q; qd; Fm(2); Fm(1)];
        else
            up(I.QF) = 0;
            up(I.Qv) = P.cost.Qv;
            x0 = [q; qd; 0; 0];
        end
        grampc.userparam = up;
        L.mu(k) = Md.mu;
        L.capped(k) = Md.capped;
    end

    grampc = C.setparam(grampc, 'x0', x0);
    grampc = C.setparam(grampc, 't0', t);
    [grampc, cpu] = C.run(grampc);
    u = grampc.sol.unext(:);
    uq = u(1:2);
    us = [0; 0];
    if vi
        us = u(3:4);
    end

    L.q(k, :) = q';  L.qd(k, :) = qd';  L.dq(k, :) = S.dq';  L.p(k, :) = p';
    L.u(k, :) = uq';  L.us(k, :) = us';  L.F(k, :) = [S.fn S.ft];  L.Fm(k, :) = Fm';
    L.KP(k, :) = rowMajor(KP);  L.k(k, :) = exp(s)';  L.muhat(k) = mu_hat;
    L.contact(k) = in_contact;  L.cpu(k) = cpu;  L.J(k) = grampc.sol.J(1);

    % ---- plant over [t, t + dt]
    [S, Fl] = fc_plant(S, qd, uq, J0, KP, s, us, P);
    rows = (k - 1) * nf + (1:nf);
    Fn.p(rows, :) = Fl.p;
    Fn.F(rows, :) = Fl.F;
    Fn.v(rows, :) = Fl.v;

    % ---- poststep: remember the reference (and stiffness) the low-level loop ran with
    qd = qd + dt * uq;
    if vi
        s = min(max(s + dt * us, s_min), s_max);
    end
end

R.variant = V;
R.seed = seed;
R.log = L;
R.fine = Fn;
R.metrics = fc_metrics(L, Fn, P);
end

function r = rowMajor(A)
r = reshape(A.', 1, []);
end
