/* Problem description for GRAMPC v2.3 (https://github.com/grampc/grampc),
 * based on the GRAMPC probfct template (BSD-3-Clause, see LICENSE).
 *
 * VI-MPIC: variable-impedance model predictive interaction control of the
 * 2-link planar robot, following the VI-MPIC formulation notes (eq. 19-21,
 * 26-30) and the IROS 2027 analysis (conditioned gate, Prop. 3.2; stiffness
 * rate bound, Prop. 3.4). See docs/vi-mpic.md.
 *
 * The same file runs classical MPIC (Gold et al., T-RO 2023): set UP_VARK = 0
 * (and umin = umax = 0 on the stiffness rates) and use the measured contact
 * gate (gate mode 0). With UP_VARK = 0 the stiffness rates drop out of the
 * model and their gradient is zero; a pinned input with a non-zero gradient
 * spoils the step size of GRAMPC's explicit line search.
 *
 *   x = [q1 q2 | qd1 qd2 | Fx Fy | sx sy]      u = [dqd1 dqd2 | dsx dsy]
 *
 *   dq/dt   = u_q + omega (qd - q) - omega J0^{-1} w,   w = F ./ kx,  kx = exp(s)
 *   dqd/dt  = u_q
 *   dF_i/dt = kbar_i ( gamma_i (J0 u_q)_i + psi w_i ds_i/dt ),  kbar = ke kx / (ke + kx)
 *   ds/dt   = u_s
 *
 * The Cartesian stiffness diag(kx) is realised by the joint PD
 * Kp = J0^T diag(kx) J0, Kd = Kp / omega (notes Lemma 3, n = 2, no null space).
 * Axis x is the wall normal n_c = +e_x. Its gate gamma_x depends on the
 * planned depth of the desired frame, m(qd) = m_k + J0(1,:) (qd - qd_k).
 * Axis y has no environment (ke_y = 0).
 *
 * Gate modes (userparam UP_GMODE):
 *   0  measured gate of Gold et al. eq. (24), frozen over the horizon
 *   1  unconditioned probabilistic gate Phi(z) of the notes, eq. (27)
 *   2  gate conditioned on "no contact up to depth ell_k", analysis eq. (3.4)-(3.5)
 *
 * Stage cost: notes eq. (9) (EE position, force, input) plus the
 * uncertainty-aware stiffness cost eq. (30). Constraints: s_min <= s <= s_max.
 * The guarded approach is specified as a velocity of the desired frame along
 * the normal, 0.5 QV (J0(1,:) dqd/dt - VAPP)^2, not as a position of the EE:
 * the force state is the expected force, so before contact the model pushes
 * q back by the expected deflection J0^{-1} F/kx, and a cost on FK(q) drives
 * the reference ahead while the real arm, which has no contact yet, follows
 * it. A time-indexed position ramp makes the reference sprint after any
 * delay. A position term on the desired frame FK(qd) (weights UP_QR) is
 * available as well.
 */

#include <math.h>
#include "probfct.h"

#define POW2(a) ((a)*(a))

/* link lengths of the robot [m], must match vi_params.m */
#define LINK1 0.2
#define LINK2 0.2

/* userparam layout (0-based C index), mirrored by vi_index.m */
#define UP_OMEGA   0   /* PD bandwidth omega = Kp / Kd [1/s]                       */
#define UP_KE      1   /* environment stiffness ke_x, ke_y [N/m]                   */
#define UP_J0      3   /* frozen Jacobian J0 = J(q_k), row major                   */
#define UP_J0INV   7   /* inverse of J0, row major                                 */
#define UP_PHAT   11   /* surface belief: mean position along n_c = +e_x [m]       */
#define UP_SIGMA  12   /* surface belief: standard deviation [m]                   */
#define UP_ELL    13   /* deepest contact-free depth ell_k [m], -1e3 if none       */
#define UP_MK     14   /* depth of the desired frame at t_k, x of FK(qd_k) [m]     */
#define UP_QDK    15   /* qd_k, linearisation point of m(qd)                       */
#define UP_GMODE  17   /* gate mode 0, 1 or 2, see above                           */
#define UP_GMEAS  18   /* measured gate for mode 0 (0 or 1)                        */
#define UP_PSI    19   /* 1: keep the stiffness-rate term w .* ds/dt               */
#define UP_QP     20   /* EE position weights (x, y)                               */
#define UP_PDES   22   /* EE position target at the start of the horizon (x, y)    */
#define UP_VDES   24   /* optional position ramp: target velocity along x [m/s]    */
#define UP_TDELAY 25   /* optional position ramp: horizon time it starts [s]       */
#define UP_PXMAX  26   /* optional position ramp: end along x [m]                  */
#define UP_QF     27   /* force weights (x, y)                                     */
#define UP_FDES   29   /* desired force (x, y) [N]                                 */
#define UP_RQ     31   /* weight on the reference velocity dqd/dt                  */
#define UP_RS     32   /* weight on the stiffness rate ds/dt                       */
#define UP_QFR    33   /* q_F of the force-risk term, notes eq. (30)               */
#define UP_QPR    34   /* q_p of the disturbance-risk term, notes eq. (30)         */
#define UP_SIGW   35   /* disturbance standard deviation sigma_w [N]               */
#define UP_QS     36   /* weight on s - s_ref                                      */
#define UP_SREF   37   /* s_ref (x, y)                                             */
#define UP_SMIN   39   /* lower log-stiffness bound (x, y)                         */
#define UP_SMAX   41   /* upper log-stiffness bound (x, y)                         */
#define UP_VSCALE 43   /* terminal cost = VSCALE * state part of the stage cost    */
#define UP_QR     44   /* weights on the desired-frame position FK(qd) (x, y)      */
#define UP_VARK   46   /* 1: ds/dt is a decision variable (VI-MPIC), 0: fixed gains */
#define UP_QV     47   /* weight on the normal reference velocity J0(1,:) dqd/dt    */
#define UP_VAPP   48   /* desired normal reference velocity (guarded approach) [m/s] */
#define UP_VDESY  49   /* velocity of the EE target along y (sliding) [m/s]        */

#define UP(i) (((const double *)userparam)[i])

/* contact gate and penetration moments along the wall normal */
typedef struct
{
    double gamma;   /* gate = contact probability                     */
    double var;     /* Var[d+], penetration variance                  */
    double dgamma;  /* d gamma / d m                                  */
    double dvar;    /* d Var[d+] / d m                                */
} Gate;

static double normpdf(double z) { return 0.39894228040143267794 * exp(-0.5 * z * z); }
/* upper tail 1 - Phi(z), accurate for large z */
static double normsf(double z) { return 0.5 * erfc(0.70710678118654752440 * z); }

/* series stiffness of controller and environment, notes eq. (18) */
static double kbar(double ke, double kx) { return ke > 0.0 ? ke * kx / (ke + kx) : 0.0; }

/* planned depth of the desired frame along n_c = +e_x, linearised with J0 */
static double depth(ctypeRNum *x, typeUSERPARAM *userparam)
{
    return UP(UP_MK) + UP(UP_J0) * (x[2] - UP(UP_QDK)) + UP(UP_J0 + 1) * (x[3] - UP(UP_QDK + 1));
}

/* gamma, Var[d+] and their derivatives with respect to the depth m.
 * Mode 1 is the rectified Gaussian of the notes (a -> -inf). Mode 2
 * conditions the prior on p_o >= ell: with a = (ell - p_hat)/sigma and
 * b = (m - p_hat)/sigma, for b > a
 *   gamma = (Phi(b) - Phi(a)) / (1 - Phi(a))
 *   g     = sigma (b (Phi(b) - Phi(a)) + phi(b) - phi(a)) / (1 - Phi(a))
 *   E2    = sigma^2 ((b^2 + 1)(Phi(b) - Phi(a)) + b phi(b) + (a - 2b) phi(a)) / (1 - Phi(a))
 *   Var = E2 - g^2,  dg/dm = gamma,  dE2/dm = 2 g,  dVar/dm = 2 g (1 - gamma). */
static void gateEval(Gate *G, double m, typeUSERPARAM *userparam)
{
    const int mode = (int)UP(UP_GMODE);
    const double sig = UP(UP_SIGMA);
    double a, b, Qa, dPhi, pa, pb, g, E2;

    G->gamma = 0.0;
    G->var = 0.0;
    G->dgamma = 0.0;
    G->dvar = 0.0;

    if (mode == 0)
    {
        G->gamma = UP(UP_GMEAS);
        G->var = G->gamma * sig * sig;
        return;
    }

    b = (m - UP(UP_PHAT)) / sig;
    if (mode == 2)
    {
        a = (UP(UP_ELL) - UP(UP_PHAT)) / sig;
        if (a > 30.0)
        {
            a = 30.0;   /* 30 sigma past the mean without contact: keep 1 - Phi(a) > 0 */
        }
    }
    else
    {
        a = -40.0;      /* unconditioned: Phi(a) = 0, phi(a) = 0 */
    }
    if (b <= a)
    {
        return;
    }

    Qa = normsf(a);
    dPhi = Qa - normsf(b);
    pa = normpdf(a);
    pb = normpdf(b);
    g = sig * (b * dPhi + pb - pa) / Qa;
    E2 = sig * sig * ((b * b + 1.0) * dPhi + b * pb + (a - 2.0 * b) * pa) / Qa;

    G->gamma = dPhi / Qa;
    G->var = E2 - g * g;
    if (G->var < 0.0)
    {
        G->var = 0.0;
    }
    G->dgamma = pb / (sig * Qa);
    G->dvar = 2.0 * g * (1.0 - G->gamma);
}

/* forward kinematics and Jacobian of the end effector (true kinematics at q) */
static void fkin(double *pe, double *J, ctypeRNum *q)
{
    const double s1 = sin(q[0]), c1 = cos(q[0]);
    const double s12 = sin(q[0] + q[1]), c12 = cos(q[0] + q[1]);

    pe[0] = LINK1 * c1 + LINK2 * c12;
    pe[1] = LINK1 * s1 + LINK2 * s12;
    J[0] = -LINK1 * s1 - LINK2 * s12;
    J[1] = -LINK2 * s12;
    J[2] = LINK1 * c1 + LINK2 * c12;
    J[3] = LINK2 * c12;
}

/* EE position target at horizon time t: hold, optional ramp along x, and a
 * target moving along y while sliding on the wall */
static void target(double *pd, double t, typeUSERPARAM *userparam)
{
    const double tm = t - UP(UP_TDELAY);

    pd[0] = UP(UP_PDES);
    pd[1] = UP(UP_PDES + 1) + UP(UP_VDESY) * t;
    if (UP(UP_VDES) > 0.0 && tm > 0.0)
    {
        pd[0] += UP(UP_VDES) * tm;
        if (pd[0] > UP(UP_PXMAX))
        {
            pd[0] = UP(UP_PXMAX);
        }
    }
}

/* uncertainty-aware stiffness cost, notes eq. (30), per Cartesian axis:
 *   0.5 qF kbar^2 Var[d+] + 0.5 qp sigma_w^2 ((1 - gamma)/kx^2 + gamma/(kx + ke)^2)
 *   + 0.5 rs ds^2 + 0.5 qs (s - s_ref)^2
 * The contact probability Phi of the notes is the gate gamma (analysis 3.2 (i)). */
static double stiffnessCost(ctypeRNum *x, ctypeRNum *u, const Gate *G, typeUSERPARAM *userparam)
{
    const double sw2 = POW2(UP(UP_SIGW));
    double c = 0.0, ke, kx, kb, gam, var;
    int i;

    for (i = 0; i < 2; i++)
    {
        ke = UP(UP_KE + i);
        kx = exp(x[6 + i]);
        kb = kbar(ke, kx);
        gam = (i == 0) ? G->gamma : 0.0;
        var = (i == 0) ? G->var : 0.0;
        c += 0.5 * UP(UP_QFR) * kb * kb * var
           + 0.5 * UP(UP_QPR) * sw2 * ((1.0 - gam) / (kx * kx) + gam / POW2(kx + ke))
           + 0.5 * UP(UP_RS) * POW2(u[2 + i])
           + 0.5 * UP(UP_QS) * POW2(x[6 + i] - UP(UP_SREF + i));
    }
    return c;
}


/** OCP dimensions: states (Nx), controls (Nu), parameters (Np), equalities (Ng),
    inequalities (Nh), terminal equalities (NgT), terminal inequalities (NhT) **/
void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT, typeUSERPARAM *userparam)
{
    *Nx = 8;
    *Nu = 4;
    *Np = 0;
    *Ng = 0;
    *Nh = 4;
    *NgT = 0;
    *NhT = 0;
}


/** System function f(t,x,u,p,param,userparam)
    ------------------------------------ **/
void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    const double om = UP(UP_OMEGA), psi = UP(UP_PSI);
    const double *J0 = (const double *)userparam + UP_J0;
    const double *Ji = (const double *)userparam + UP_J0INV;
    const double us[2] = {UP(UP_VARK) * u[2], UP(UP_VARK) * u[3]};
    double kx[2], w[2], kb[2], gam[2], r[2];
    Gate G;
    int i;

    gateEval(&G, depth(x, userparam), userparam);
    gam[0] = G.gamma;
    gam[1] = 0.0;
    for (i = 0; i < 2; i++)
    {
        kx[i] = exp(x[6 + i]);
        w[i] = x[4 + i] / kx[i];
        kb[i] = kbar(UP(UP_KE + i), kx[i]);
    }
    r[0] = J0[0] * u[0] + J0[1] * u[1];
    r[1] = J0[2] * u[0] + J0[3] * u[1];

    /* quasi-static joint PD with Kd = Kp / omega, pushed back by the contact */
    out[0] = u[0] + om * (x[2] - x[0]) - om * (Ji[0] * w[0] + Ji[1] * w[1]);
    out[1] = u[1] + om * (x[3] - x[1]) - om * (Ji[2] * w[0] + Ji[3] * w[1]);
    out[2] = u[0];
    out[3] = u[1];
    /* expected-force dynamics with time-varying stiffness, notes eq. (19) / (29) */
    for (i = 0; i < 2; i++)
    {
        out[4 + i] = kb[i] * (gam[i] * r[i] + psi * w[i] * us[i]);
    }
    out[6] = us[0];
    out[7] = us[1];
}

/** Jacobian df/dx multiplied by vector vec, i.e. (df/dx)^T*vec, notes eq. (20) **/
void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    const double om = UP(UP_OMEGA), psi = UP(UP_PSI);
    const double *J0 = (const double *)userparam + UP_J0;
    const double *Ji = (const double *)userparam + UP_J0INV;
    const double us[2] = {UP(UP_VARK) * u[2], UP(UP_VARK) * u[3]};
    double kx[2], w[2], kb[2], gam[2], r[2], chi[2], v, c;
    Gate G;
    int i;

    gateEval(&G, depth(x, userparam), userparam);
    gam[0] = G.gamma;
    gam[1] = 0.0;
    for (i = 0; i < 2; i++)
    {
        kx[i] = exp(x[6 + i]);
        w[i] = x[4 + i] / kx[i];
        kb[i] = kbar(UP(UP_KE + i), kx[i]);
    }
    r[0] = J0[0] * u[0] + J0[1] * u[1];
    r[1] = J0[2] * u[0] + J0[3] * u[1];
    /* chi = J0^{-T} lambda_q */
    chi[0] = Ji[0] * vec[0] + Ji[2] * vec[1];
    chi[1] = Ji[1] * vec[0] + Ji[3] * vec[1];

    out[0] = -om * vec[0];
    out[1] = -om * vec[1];
    /* the gate depends on qd through the planned depth m(qd) */
    c = G.dgamma * kb[0] * r[0] * vec[4];
    out[2] = om * vec[0] + c * J0[0];
    out[3] = om * vec[1] + c * J0[1];
    for (i = 0; i < 2; i++)
    {
        v = gam[i] * r[i] + psi * w[i] * us[i];
        out[4 + i] = -om * chi[i] / kx[i] + kb[i] * psi * us[i] * vec[4 + i] / kx[i];
        out[6 + i] = om * w[i] * chi[i]
                   + (kb[i] * kb[i] / kx[i] * v - kb[i] * psi * w[i] * us[i]) * vec[4 + i];
    }
}

/** Jacobian df/du multiplied by vector vec, i.e. (df/du)^T*vec, notes eq. (21) **/
void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    const double psi = UP(UP_PSI);
    const double *J0 = (const double *)userparam + UP_J0;
    double kx[2], w[2], kb[2], gam[2], lf[2];
    Gate G;
    int i;

    gateEval(&G, depth(x, userparam), userparam);
    gam[0] = G.gamma;
    gam[1] = 0.0;
    for (i = 0; i < 2; i++)
    {
        kx[i] = exp(x[6 + i]);
        w[i] = x[4 + i] / kx[i];
        kb[i] = kbar(UP(UP_KE + i), kx[i]);
        lf[i] = kb[i] * gam[i] * vec[4 + i];
    }

    out[0] = vec[0] + vec[2] + J0[0] * lf[0] + J0[2] * lf[1];
    out[1] = vec[1] + vec[3] + J0[1] * lf[0] + J0[3] * lf[1];
    out[2] = UP(UP_VARK) * (kb[0] * psi * w[0] * vec[4] + vec[6]);
    out[3] = UP(UP_VARK) * (kb[1] * psi * w[1] * vec[5] + vec[7]);
}

/** Jacobian df/dp multiplied by vector vec, i.e. (df/dp)^T*vec or vec^T*(df/dp) **/
void dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}


/** Integral cost l(t,x(t),u(t),p,param,userparam)
    -------------------------------------------------- **/
void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    double pe[2], J[4], pr[2], Jr[4], pd[2];
    const double rn = UP(UP_J0) * u[0] + UP(UP_J0 + 1) * u[1];
    Gate G;

    fkin(pe, J, x);
    fkin(pr, Jr, x + 2);
    target(pd, t, userparam);
    gateEval(&G, depth(x, userparam), userparam);

    out[0] = 0.5 * (UP(UP_QP) * POW2(pe[0] - pd[0]) + UP(UP_QP + 1) * POW2(pe[1] - pd[1])
                  + UP(UP_QR) * POW2(pr[0] - pd[0]) + UP(UP_QR + 1) * POW2(pr[1] - pd[1])
                  + UP(UP_QF) * POW2(x[4] - UP(UP_FDES)) + UP(UP_QF + 1) * POW2(x[5] - UP(UP_FDES + 1))
                  + UP(UP_RQ) * (POW2(u[0]) + POW2(u[1]))
                  + UP(UP_QV) * POW2(rn - UP(UP_VAPP)))
           + stiffnessCost(x, u, &G, userparam);
}
/** Gradient dl/dx **/
void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    const double *J0 = (const double *)userparam + UP_J0;
    const double sw2 = POW2(UP(UP_SIGW));
    double pe[2], J[4], pr[2], Jr[4], pd[2], e0, e1, r0, r1, kx, ke, kb, gam, var, dm;
    Gate G;
    int i;

    fkin(pe, J, x);
    fkin(pr, Jr, x + 2);
    target(pd, t, userparam);
    gateEval(&G, depth(x, userparam), userparam);

    /* EE position error through the true kinematics */
    e0 = UP(UP_QP) * (pe[0] - pd[0]);
    e1 = UP(UP_QP + 1) * (pe[1] - pd[1]);
    out[0] = J[0] * e0 + J[2] * e1;
    out[1] = J[1] * e0 + J[3] * e1;

    /* risk terms depend on qd through the planned depth m(qd) */
    ke = UP(UP_KE);
    kx = exp(x[6]);
    kb = kbar(ke, kx);
    dm = 0.5 * UP(UP_QFR) * kb * kb * G.dvar
       + 0.5 * UP(UP_QPR) * sw2 * (1.0 / POW2(kx + ke) - 1.0 / (kx * kx)) * G.dgamma;
    /* desired-frame position error */
    r0 = UP(UP_QR) * (pr[0] - pd[0]);
    r1 = UP(UP_QR + 1) * (pr[1] - pd[1]);
    out[2] = dm * J0[0] + Jr[0] * r0 + Jr[2] * r1;
    out[3] = dm * J0[1] + Jr[1] * r0 + Jr[3] * r1;

    out[4] = UP(UP_QF) * (x[4] - UP(UP_FDES));
    out[5] = UP(UP_QF + 1) * (x[5] - UP(UP_FDES + 1));

    /* d kbar / d s = kbar^2 / kx */
    for (i = 0; i < 2; i++)
    {
        ke = UP(UP_KE + i);
        kx = exp(x[6 + i]);
        kb = kbar(ke, kx);
        gam = (i == 0) ? G.gamma : 0.0;
        var = (i == 0) ? G.var : 0.0;
        out[6 + i] = UP(UP_QFR) * kb * kb * kb * var / kx
                   - UP(UP_QPR) * sw2 * ((1.0 - gam) / (kx * kx) + gam * kx / POW2(kx + ke) / (kx + ke))
                   + UP(UP_QS) * (x[6 + i] - UP(UP_SREF + i));
    }
}
/** Gradient dl/du **/
void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    const double ev = UP(UP_QV) * (UP(UP_J0) * u[0] + UP(UP_J0 + 1) * u[1] - UP(UP_VAPP));

    out[0] = UP(UP_RQ) * u[0] + ev * UP(UP_J0);
    out[1] = UP(UP_RQ) * u[1] + ev * UP(UP_J0 + 1);
    out[2] = UP(UP_VARK) * UP(UP_RS) * u[2];
    out[3] = UP(UP_VARK) * UP(UP_RS) * u[3];
}
/** Gradient dl/dp **/
void dldp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}


/** Terminal cost V(T,x(T),p,param,userparam)
    ---------------------------------------- **/
void Vfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    double pe[2], J[4], pr[2], Jr[4], pd[2];

    fkin(pe, J, x);
    fkin(pr, Jr, x + 2);
    target(pd, T, userparam);
    out[0] = 0.5 * UP(UP_VSCALE) * (UP(UP_QP) * POW2(pe[0] - pd[0]) + UP(UP_QP + 1) * POW2(pe[1] - pd[1])
                                  + UP(UP_QR) * POW2(pr[0] - pd[0]) + UP(UP_QR + 1) * POW2(pr[1] - pd[1])
                                  + UP(UP_QF) * POW2(x[4] - UP(UP_FDES)) + UP(UP_QF + 1) * POW2(x[5] - UP(UP_FDES + 1)));
}
/** Gradient dV/dx **/
void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    const double vs = UP(UP_VSCALE);
    double pe[2], J[4], pr[2], Jr[4], pd[2], e0, e1, r0, r1;

    fkin(pe, J, x);
    fkin(pr, Jr, x + 2);
    target(pd, T, userparam);
    e0 = UP(UP_QP) * (pe[0] - pd[0]);
    e1 = UP(UP_QP + 1) * (pe[1] - pd[1]);
    r0 = UP(UP_QR) * (pr[0] - pd[0]);
    r1 = UP(UP_QR + 1) * (pr[1] - pd[1]);

    out[0] = vs * (J[0] * e0 + J[2] * e1);
    out[1] = vs * (J[1] * e0 + J[3] * e1);
    out[2] = vs * (Jr[0] * r0 + Jr[2] * r1);
    out[3] = vs * (Jr[1] * r0 + Jr[3] * r1);
    out[4] = vs * UP(UP_QF) * (x[4] - UP(UP_FDES));
    out[5] = vs * UP(UP_QF + 1) * (x[5] - UP(UP_FDES + 1));
    out[6] = 0.0;
    out[7] = 0.0;
}
/** Gradient dV/dp **/
void dVdp(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Gradient dV/dT **/
void dVdT(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}


/** Equality constraints g(t,x(t),u(t),p,param,userparam) = 0
    --------------------------------------------------- **/
void gfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/dx multiplied by vector vec, i.e. (dg/dx)^T*vec or vec^T*(dg/dx) **/
void dgdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/du multiplied by vector vec, i.e. (dg/du)^T*vec or vec^T*(dg/du) **/
void dgdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/dp multiplied by vector vec, i.e. (dg/dp)^T*vec or vec^T*(dg/dp) **/
void dgdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}


/** Inequality constraints h(t,x(t),u(t),p,param,userparam) <= 0
    ------------------------------------------------------ **/
/* log-stiffness bounds s_min <= s <= s_max, s_min from the overdamping floor, notes eq. (15) */
void hfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    out[0] = UP(UP_SMIN) - x[6];
    out[1] = x[6] - UP(UP_SMAX);
    out[2] = UP(UP_SMIN + 1) - x[7];
    out[3] = x[7] - UP(UP_SMAX + 1);
}
/** Jacobian dh/dx multiplied by vector vec, i.e. (dh/dx)^T*vec or vec^T*(dg/dx) **/
void dhdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    int i;
    for (i = 0; i < 6; i++)
    {
        out[i] = 0.0;
    }
    out[6] = vec[1] - vec[0];
    out[7] = vec[3] - vec[2];
}
/** Jacobian dh/du multiplied by vector vec, i.e. (dh/du)^T*vec or vec^T*(dg/du) **/
void dhdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    out[0] = 0.0;
    out[1] = 0.0;
    out[2] = 0.0;
    out[3] = 0.0;
}
/** Jacobian dh/dp multiplied by vector vec, i.e. (dh/dp)^T*vec or vec^T*(dg/dp) **/
void dhdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}


/** Terminal equality constraints gT(T,x(T),p,param,userparam) = 0
    -------------------------------------------------------- **/
void gTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dx multiplied by vector vec, i.e. (dgT/dx)^T*vec or vec^T*(dgT/dx) **/
void dgTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dp multiplied by vector vec, i.e. (dgT/dp)^T*vec or vec^T*(dgT/dp) **/
void dgTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dT multiplied by vector vec, i.e. (dgT/dT)^T*vec or vec^T*(dgT/dT) **/
void dgTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}


/** Terminal inequality constraints hT(T,x(T),p,param,userparam) <= 0
    ----------------------------------------------------------- **/
void hTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dhT/dx multiplied by vector vec, i.e. (dhT/dx)^T*vec or vec^T*(dhT/dx) **/
void dhTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dhT/dp multiplied by vector vec, i.e. (dhT/dp)^T*vec or vec^T*(dhT/dp) **/
void dhTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dhT/dT multiplied by vector vec, i.e. (dhT/dT)^T*vec or vec^T*(dhT/dT) **/
void dhTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}


/** Additional functions required for semi-implicit systems
    M*dx/dt(t) = f(t,x(t),u(t),p,param,userparam) using the solver RODAS
    ------------------------------------------------------- **/
/** Jacobian df/dx in vector form (column-wise) **/
void dfdx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian df/dx in vector form (column-wise) **/
void dfdxtrans(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian df/dt **/
void dfdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian d(dH/dx)/dt  **/
void dHdxdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Mass matrix in vector form (column-wise, either banded or full matrix) **/
void Mfct(typeRNum *out, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Transposed mass matrix in vector form (column-wise, either banded or full matrix) **/
void Mtrans(typeRNum *out, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
