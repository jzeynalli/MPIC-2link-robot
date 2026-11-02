function [S, Fl] = fc_plant(S, qd, uq, J0, KP, s, us, P)
%FC_PLANT Rigid-body 2-link arm with a frictional wall over one MPC interval.
%   [S, Fl] = fc_plant(S, qd, uq, J0, KP, s, us, P) integrates the plant
%   state S (q, dq, z, fn, ft) over P.mpc.dt with the reference
%   qd(tau) = qd + uq tau of the low-level joint PD (Gold et al. eq. 4),
%     M(q) ddq = Kp (qd(tau) - q) + Kd (uq - dq) - J(q)' [fn; ft],  Kd = Kp/omega,
%   Coriolis terms compensated, semi-implicit Euler with step P.robot.h.
%   Kp = KP, or Kp = J0' diag(exp(s + us tau)) J0 when the stiffness moves
%   (VI-MPIC, us ~= 0). The wall x = P.wall.x is a spring-damper along the
%   normal, fn = max(0, ke d + ce dd) for d > 0, with LuGre friction and a
%   Stribeck effect along it (Canudas de Wit et al. 1995),
%     g = mu_c fn + (mu_s - mu_c) fn exp(-(v/v_s)^2),  dz = v - sigma0 |v| z / g,
%     ft = sigma0 z + sigma1 dz + sigma2 v,
%   as in the prototype of the proposal (App. B). fn and ft are the force the
%   robot exerts on the wall. Fl logs the interval at 1 kHz (p, F, v).

h = P.robot.h;
nsub = round(P.mpc.dt / h);
nf = round(1e-3 / h);
Rb = P.robot;
l1 = Rb.l(1);  l2 = Rb.l(2);
a1 = Rb.I(1) + Rb.I(2) + Rb.m(1) * Rb.r(1)^2 + Rb.m(2) * (l1^2 + Rb.r(2)^2);
a2 = Rb.m(2) * l1 * Rb.r(2);
a3 = Rb.I(2) + Rb.m(2) * Rb.r(2)^2;
xw = P.wall.x;  ke = P.wall.ke;  ce = P.wall.ce;  Fr = P.fric;  om = P.omega;
vary = any(us ~= 0);
s_min = log(P.vi.k_min);  s_max = log(P.vi.k_max);
KD = KP / om;

q = S.q;  dq = S.dq;  z = S.z;  fn = S.fn;  ft = S.ft;
Fl.p = zeros(nsub / nf, 2);  Fl.F = zeros(nsub / nf, 2);  Fl.v = zeros(nsub / nf, 2);
for j = 1:nsub
    tau = (j - 1) * h;
    qdt = qd + uq * tau;
    if vary
        kt = exp(min(max(s + us * tau, s_min), s_max));
        KP = J0' * [kt(1) 0; 0 kt(2)] * J0;
        KD = KP / om;
    end
    c2 = cos(q(2));
    Mq = [a1 + 2 * a2 * c2, a3 + a2 * c2; a3 + a2 * c2, a3];
    s1 = sin(q(1));  c1 = cos(q(1));
    s12 = sin(q(1) + q(2));  c12 = cos(q(1) + q(2));
    px = l1 * c1 + l2 * c12;
    py = l1 * s1 + l2 * s12;
    Jq = [-l1 * s1 - l2 * s12, -l2 * s12; l1 * c1 + l2 * c12, l2 * c12];
    v = Jq * dq;

    d = px - xw;
    if d > 0
        fn = max(0, ke * d + ce * v(1));
    else
        fn = 0;
    end
    if fn > 0
        g = Fr.mu_c * fn + (Fr.mu_s - Fr.mu_c) * fn * exp(-(v(2) / Fr.v_s)^2);
        zd = v(2) - Fr.sigma0 * abs(v(2)) * z / g;
        z = z + h * zd;
        zmax = Fr.mu_s * fn / Fr.sigma0;
        z = min(max(z, -zmax), zmax);
        ft = Fr.sigma0 * z + Fr.sigma1 * zd + Fr.sigma2 * v(2);
    else
        z = 0;
        ft = 0;
    end

    dq = (Mq + h * KD) \ (Mq * dq + h * (KP * (qdt - q) + KD * uq - Jq' * [fn; ft]));
    q = q + h * dq;

    if mod(j - 1, nf) == 0
        i = (j - 1) / nf + 1;
        Fl.p(i, :) = [px py];
        Fl.F(i, :) = [fn ft];
        Fl.v(i, :) = v';
    end
end
S.q = q;  S.dq = dq;  S.z = z;  S.fn = fn;  S.ft = ft;
end
