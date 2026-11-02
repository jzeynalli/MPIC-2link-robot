/* Problem description for GRAMPC v2.3 (https://github.com/grampc/grampc),
 * based on the GRAMPC probfct template (BSD-3-Clause, see LICENSE).
 *
 * FC-MPIC: friction-consistent model predictive interaction control of the
 * 2-link planar robot, following the FC-MPIC proposal (2 Nov 2026): contact
 * force updated by a return map, eq. (2.4)-(2.8), OCP (2.13) and the
 * discrete adjoint of its Appendix A. See docs/fc-mpic.md.
 *
 * The same file runs classical MPIC (Gold et al., T-RO 2023) as model 0,
 * in continuous time (the original formulation) or as a discrete-time map
 * with the discretisation of FC-MPIC, and the pure-sliding Coulomb model of
 * Kleff et al. as model 2. All models share the joint row and the cost.
 *
 *   x = [q1 q2 | qd1 qd2 | ft fn]      u = dqd/dt
 *
 * phi = (ft, fn) = B' F is the force the robot exerts on the wall in contact
 * coordinates, B = [T, -n], tangent T = +e_y, outward wall normal n = -e_x:
 * fn = Fx >= 0 is the normal force and ft = Fy the friction force.
 *
 * Joint row: quasi-static joint PD (Gold et al. eq. 15) with J0 = J(q_k)
 * frozen, for any stiffness Kp and damping Kd of the low-level controller,
 *   dq/dt = u + W (qd - q) - G phi,   W = Kd^-1 Kp,   G = Kd^-1 J0' B.
 * The reference of the contact point moves at r = B' J0 u = (r_t, r_n).
 *
 * Force row (UP_MODEL), active only with the measured contact gate (UP_GATE)
 * of Gold et al. eq. (24); without contact phi is held:
 *   0  Gold et al.: series spring along the normal, friction frozen at its
 *      measured value, dfn/dt = kbar_n r_n, dft/dt = 0,
 *      kbar_n = 1 / (n' C n + 1/k_n),  C = J0 Kp^-1 J0'
 *   1  FC-MPIC: elastic predictor phi_tr = phi + Kh r dt with the stick
 *      stiffness Kh = (B' C B + D)^-1, then the smooth plastic corrector
 *      phi+ = phi_tr - Kh e1 s(phi_tr) of eq. (2.8)
 *   2  pure-sliding Coulomb friction (Kleff et al.): fn as in model 0,
 *      ft = mu fn tanh(r_t / eps_v)
 * UP_DISC = 1 gives the map x+ = f(x, u) for GRAMPC's Integrator =
 * 'discrete' (explicit Euler in q, exact in qd, return map in phi), which
 * models 1 and 2 require. UP_DISC = 0 gives the ODE of model 0.
 *
 * Cost, with the terms of probfct_VI_MPIC.c (eq. 9 of the VI-MPIC notes):
 *   l = cs/2 (Qp_x (px - px*)^2 + Qp_y (py - py*(t))^2 + QF (fn - fn*)^2
 *             + RQ |u - u*(t)|^2 + Qv (r_n - v_app)^2)
 * with cs = dt in discrete time, so that the sum approximates the integral of
 * the continuous-time cost, and cs = 1 in continuous time. py*(t) is the
 * sanding stroke along the wall and u* = J0^-1 (0, d(py*)/dt) its velocity
 * feed-forward (Gold et al. Sec. III-D). The friction force is not weighted.
 */

#include <math.h>
#include "probfct.h"

#define POW2(a) ((a)*(a))

/* link lengths of the robot [m], must match fc_params.m */
#define LINK1 0.2
#define LINK2 0.2

/* userparam layout (0-based C index), mirrored by fc_index.m */
#define UP_MODEL   0   /* force model: 0 Gold et al., 1 FC-MPIC, 2 pure sliding       */
#define UP_DISC    1   /* 1: discrete-time map, 0: continuous-time ODE (model 0)      */
#define UP_GATE    2   /* measured contact gate, frozen over the horizon (0 or 1)     */
#define UP_W       3   /* W = Kd^-1 Kp, row major                                     */
#define UP_G       7   /* G = Kd^-1 J0' B, row major                                  */
#define UP_BJ     11   /* B' J0, row major: row 0 tangential, row 1 normal            */
#define UP_KH     15   /* stick stiffness Kh, row major (tt, tn, nt, nn) [N/m]        */
#define UP_KBN    19   /* normal series stiffness kbar_n of Gold et al. [N/m]         */
#define UP_MU     20   /* friction coefficient mu_k of the MPC step                   */
#define UP_EPSF   21   /* eps_f of the smooth sign in the corrector [N]               */
#define UP_BETA   22   /* sharpness beta of the softplus [1/N]                        */
#define UP_EPSV   23   /* eps_v of the pure-sliding model [m/s]                       */
#define UP_J0     24   /* frozen Jacobian J0 = J(q_k), row major                      */
#define UP_J0INV  28   /* its inverse, row major                                      */
#define UP_QP     32   /* EE position weights (x, y)                                  */
#define UP_PDES   34   /* EE target (x, y) at the start of the stroke [m]             */
#define UP_YA     36   /* stroke amplitude A: py* = pdes_y + A (1 - cos(w (t - t0)))  */
#define UP_YW     37   /* stroke angular frequency w [rad/s]                          */
#define UP_YT0    38   /* horizon time t0 at which the stroke starts [s]              */
#define UP_FF     39   /* 1: input cost on u - u*, with the stroke feed-forward u*    */
#define UP_QF     40   /* normal force weight                                         */
#define UP_FDES   41   /* desired normal force [N]                                    */
#define UP_RQ     42   /* input weight                                                */
#define UP_QV     43   /* weight on the normal reference velocity r_n                 */
#define UP_VAPP   44   /* desired normal reference velocity (guarded approach) [m/s]  */
#define UP_VSCALE 45   /* terminal cost = VSCALE * state part of the stage cost       */

#define UP(i) (((const double *)userparam)[i])

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

/* EE target and input feed-forward at horizon time t: the sanding stroke
 * py* = pdes_y + A (1 - cos(w (t - t0))) for t > t0, u* = J0^-1 (0, d(py*)/dt) */
static void target(double *pd, double *us, double t, typeUSERPARAM *userparam)
{
    const double tau = t - UP(UP_YT0);
    const double w = UP(UP_YW);
    double dy = 0.0, vy = 0.0;

    if (tau > 0.0)
    {
        dy = UP(UP_YA) * (1.0 - cos(w * tau));
        vy = UP(UP_YA) * w * sin(w * tau);
    }
    pd[0] = UP(UP_PDES);
    pd[1] = UP(UP_PDES + 1) + dy;
    us[0] = UP(UP_FF) * UP(UP_J0INV + 1) * vy;
    us[1] = UP(UP_FF) * UP(UP_J0INV + 3) * vy;
}

/* reference rate of the contact point in contact coordinates, r = B' J0 u */
static void refRate(double *r, ctypeRNum *u, typeUSERPARAM *userparam)
{
    r[0] = UP(UP_BJ) * u[0] + UP(UP_BJ + 1) * u[1];
    r[1] = UP(UP_BJ + 2) * u[0] + UP(UP_BJ + 3) * u[1];
}

/* joint rate of the quasi-static joint PD, u + W (qd - q) - G phi */
static void jointRate(double *dq, ctypeRNum *x, ctypeRNum *u, typeUSERPARAM *userparam)
{
    const double *W = (const double *)userparam + UP_W;
    const double *G = (const double *)userparam + UP_G;
    int i;

    for (i = 0; i < 2; i++)
    {
        dq[i] = u[i] + W[2 * i] * (x[2] - x[0]) + W[2 * i + 1] * (x[3] - x[1])
              - G[2 * i] * x[4] - G[2 * i + 1] * x[5];
    }
}

/* softplus sp(x) = log(1 + exp(beta x)) / beta and its derivative, overflow-safe */
static double softplus(double *dsp, double x, double beta)
{
    const double bx = beta * x;
    double e;

    if (bx > 0.0)
    {
        e = exp(-bx);
        *dsp = 1.0 / (1.0 + e);
        return x + log1p(e) / beta;
    }
    e = exp(bx);
    *dsp = e / (1.0 + e);
    return log1p(e) / beta;
}

/* smooth plastic corrector, proposal eq. (2.8), and its gradient, eq. (A.5):
 *   s = e sp(e ftr - mu fnr+) / (ktt - mu e knt),   e = tanh(ftr / eps_f)
 * ds = (ds/dftr, ds/dfnr). The prototype of the proposal clips the normal
 * force at zero inside the cone function, fnr+ = max(fnr, 0). */
static double corrector(double *ds, double ftr, double fnr, typeUSERPARAM *userparam)
{
    const double mu = UP(UP_MU), epsf = UP(UP_EPSF);
    const double ktt = UP(UP_KH), knt = UP(UP_KH + 2);
    const double e = tanh(ftr / epsf);
    const double de = (1.0 - e * e) / epsf;
    const double fnp = fnr > 0.0 ? fnr : 0.0;
    const double den = ktt - mu * e * knt;
    double S, dS;

    S = softplus(&dS, e * ftr - mu * fnp, UP(UP_BETA));
    ds[0] = (de * S + e * dS * (e + de * ftr)) / den + mu * e * de * S * knt / (den * den);
    ds[1] = fnr > 0.0 ? -mu * e * dS / den : 0.0;
    return e * S / den;
}

/* elastic predictor phi_tr = phi + Kh r dt */
static void predictor(double *phitr, ctypeRNum *x, const double *r, double dt, typeUSERPARAM *userparam)
{
    const double *Kh = (const double *)userparam + UP_KH;

    phitr[0] = x[4] + dt * (Kh[0] * r[0] + Kh[1] * r[1]);
    phitr[1] = x[5] + dt * (Kh[2] * r[0] + Kh[3] * r[1]);
}


/** OCP dimensions: states (Nx), controls (Nu), parameters (Np), equalities (Ng),
    inequalities (Nh), terminal equalities (NgT), terminal inequalities (NhT) **/
void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT, typeUSERPARAM *userparam)
{
    *Nx = 6;
    *Nu = 2;
    *Np = 0;
    *Ng = 0;
    *Nh = 0;
    *NgT = 0;
    *NhT = 0;
}


/** System function f(t,x,u,p,param,userparam): the ODE (UP_DISC = 0) or the
    map x+ = f(x, u) over one sampling interval (UP_DISC = 1)
    ------------------------------------ **/
void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    const int model = (int)UP(UP_MODEL);
    const double dt = param->dt;
    const double *Kh = (const double *)userparam + UP_KH;
    double r[2], dq[2], phitr[2], ds[2], s, fn;

    refRate(r, u, userparam);
    jointRate(dq, x, u, userparam);

    if (UP(UP_DISC) == 0.0)
    {
        /* continuous time, model 0: classical MPIC */
        out[0] = dq[0];
        out[1] = dq[1];
        out[2] = u[0];
        out[3] = u[1];
        out[4] = 0.0;
        out[5] = UP(UP_GATE) * UP(UP_KBN) * r[1];
        return;
    }

    out[0] = x[0] + dt * dq[0];
    out[1] = x[1] + dt * dq[1];
    out[2] = x[2] + dt * u[0];
    out[3] = x[3] + dt * u[1];
    out[4] = x[4];
    out[5] = x[5];
    if (UP(UP_GATE) == 0.0)
    {
        return;
    }
    if (model == 1)
    {
        /* return map, eq. (2.4)-(2.8) */
        predictor(phitr, x, r, dt, userparam);
        s = corrector(ds, phitr[0], phitr[1], userparam);
        out[4] = phitr[0] - Kh[0] * s;
        out[5] = phitr[1] - Kh[2] * s;
    }
    else
    {
        fn = x[5] + dt * UP(UP_KBN) * r[1];
        out[5] = fn;
        if (model == 2)
        {
            out[4] = UP(UP_MU) * (fn > 0.0 ? fn : 0.0) * tanh(r[0] / UP(UP_EPSV));
        }
    }
}

/** Jacobian df/dx multiplied by vector vec, i.e. (df/dx)^T*vec, eq. (A.1)-(A.3) **/
void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    const int model = (int)UP(UP_MODEL);
    const double dt = param->dt;
    const double *W = (const double *)userparam + UP_W;
    const double *G = (const double *)userparam + UP_G;
    const double *Kh = (const double *)userparam + UP_KH;
    /* W' lambda_q and G' lambda_q */
    const double wq0 = W[0] * vec[0] + W[2] * vec[1];
    const double wq1 = W[1] * vec[0] + W[3] * vec[1];
    const double gq0 = G[0] * vec[0] + G[2] * vec[1];
    const double gq1 = G[1] * vec[0] + G[3] * vec[1];
    double r[2], phitr[2], ds[2], a, fn;

    if (UP(UP_DISC) == 0.0)
    {
        out[0] = -wq0;
        out[1] = -wq1;
        out[2] = wq0;
        out[3] = wq1;
        out[4] = -gq0;
        out[5] = -gq1;
        return;
    }

    out[0] = vec[0] - dt * wq0;
    out[1] = vec[1] - dt * wq1;
    out[2] = dt * wq0 + vec[2];
    out[3] = dt * wq1 + vec[3];
    out[4] = -dt * gq0 + vec[4];
    out[5] = -dt * gq1 + vec[5];
    if (UP(UP_GATE) == 0.0)
    {
        return;
    }
    refRate(r, u, userparam);
    if (model == 1)
    {
        /* (d phi+/d phi)' lambda_phi = lambda_phi - grad s (Kh e1)' lambda_phi, a rank-one update */
        predictor(phitr, x, r, dt, userparam);
        corrector(ds, phitr[0], phitr[1], userparam);
        a = Kh[0] * vec[4] + Kh[2] * vec[5];
        out[4] -= ds[0] * a;
        out[5] -= ds[1] * a;
    }
    else if (model == 2)
    {
        /* ft+ depends on fn only */
        fn = x[5] + dt * UP(UP_KBN) * r[1];
        out[4] -= vec[4];
        out[5] += (fn > 0.0 ? UP(UP_MU) * tanh(r[0] / UP(UP_EPSV)) : 0.0) * vec[4];
    }
}

/** Jacobian df/du multiplied by vector vec, i.e. (df/du)^T*vec, eq. (A.4) **/
void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    const int model = (int)UP(UP_MODEL);
    const double dt = param->dt;
    const double *BJ = (const double *)userparam + UP_BJ;
    const double *Kh = (const double *)userparam + UP_KH;
    const double kbn = UP(UP_KBN), mu = UP(UP_MU), epsv = UP(UP_EPSV);
    double r[2], phitr[2], ds[2], a, g[2], b[2], fn, th, cn, ct;
    int j;

    if (UP(UP_DISC) == 0.0)
    {
        for (j = 0; j < 2; j++)
        {
            out[j] = vec[j] + vec[2 + j] + UP(UP_GATE) * kbn * vec[5] * BJ[2 + j];
        }
        return;
    }

    for (j = 0; j < 2; j++)
    {
        out[j] = dt * (vec[j] + vec[2 + j]);
    }
    if (UP(UP_GATE) == 0.0)
    {
        return;
    }
    refRate(r, u, userparam);
    if (model == 1)
    {
        /* dt (B' J0)' Kh' (d phi+/d phi_tr)' lambda_phi */
        predictor(phitr, x, r, dt, userparam);
        corrector(ds, phitr[0], phitr[1], userparam);
        a = Kh[0] * vec[4] + Kh[2] * vec[5];
        g[0] = vec[4] - ds[0] * a;
        g[1] = vec[5] - ds[1] * a;
        b[0] = Kh[0] * g[0] + Kh[2] * g[1];
        b[1] = Kh[1] * g[0] + Kh[3] * g[1];
        for (j = 0; j < 2; j++)
        {
            out[j] += dt * (BJ[j] * b[0] + BJ[2 + j] * b[1]);
        }
    }
    else
    {
        /* fn+ = fn + dt kbar_n r_n; pure sliding: ft+ = mu fn+ tanh(r_t / eps_v) */
        cn = vec[5];
        ct = 0.0;
        if (model == 2)
        {
            fn = x[5] + dt * kbn * r[1];
            th = tanh(r[0] / epsv);
            if (fn > 0.0)
            {
                cn += mu * th * vec[4];
                ct = mu * fn * (1.0 - th * th) / epsv * vec[4];
            }
        }
        for (j = 0; j < 2; j++)
        {
            out[j] += dt * kbn * cn * BJ[2 + j] + ct * BJ[j];
        }
    }
}

/** Jacobian df/dp multiplied by vector vec, i.e. (df/dp)^T*vec or vec^T*(df/dp) **/
void dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}


/** Integral cost l(t,x(t),u(t),p,param,userparam)
    -------------------------------------------------- **/
void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    const double cs = UP(UP_DISC) != 0.0 ? param->dt : 1.0;
    const double rn = UP(UP_BJ + 2) * u[0] + UP(UP_BJ + 3) * u[1];
    double pe[2], J[4], pd[2], us[2];

    fkin(pe, J, x);
    target(pd, us, t, userparam);
    out[0] = 0.5 * cs * (UP(UP_QP) * POW2(pe[0] - pd[0]) + UP(UP_QP + 1) * POW2(pe[1] - pd[1])
                       + UP(UP_QF) * POW2(x[5] - UP(UP_FDES))
                       + UP(UP_RQ) * (POW2(u[0] - us[0]) + POW2(u[1] - us[1]))
                       + UP(UP_QV) * POW2(rn - UP(UP_VAPP)));
}
/** Gradient dl/dx **/
void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    const double cs = UP(UP_DISC) != 0.0 ? param->dt : 1.0;
    double pe[2], J[4], pd[2], us[2], e0, e1;

    fkin(pe, J, x);
    target(pd, us, t, userparam);
    e0 = UP(UP_QP) * (pe[0] - pd[0]);
    e1 = UP(UP_QP + 1) * (pe[1] - pd[1]);
    out[0] = cs * (J[0] * e0 + J[2] * e1);
    out[1] = cs * (J[1] * e0 + J[3] * e1);
    out[2] = 0.0;
    out[3] = 0.0;
    out[4] = 0.0;
    out[5] = cs * UP(UP_QF) * (x[5] - UP(UP_FDES));
}
/** Gradient dl/du **/
void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    const double cs = UP(UP_DISC) != 0.0 ? param->dt : 1.0;
    const double ev = UP(UP_QV) * (UP(UP_BJ + 2) * u[0] + UP(UP_BJ + 3) * u[1] - UP(UP_VAPP));
    double pd[2], us[2];

    target(pd, us, t, userparam);
    out[0] = cs * (UP(UP_RQ) * (u[0] - us[0]) + ev * UP(UP_BJ + 2));
    out[1] = cs * (UP(UP_RQ) * (u[1] - us[1]) + ev * UP(UP_BJ + 3));
}
/** Gradient dl/dp **/
void dldp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}


/** Terminal cost V(T,x(T),p,param,userparam)
    ---------------------------------------- **/
void Vfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    double pe[2], J[4], pd[2], us[2];

    fkin(pe, J, x);
    target(pd, us, T, userparam);
    out[0] = 0.5 * UP(UP_VSCALE) * (UP(UP_QP) * POW2(pe[0] - pd[0]) + UP(UP_QP + 1) * POW2(pe[1] - pd[1])
                                  + UP(UP_QF) * POW2(x[5] - UP(UP_FDES)));
}
/** Gradient dV/dx **/
void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    const double vs = UP(UP_VSCALE);
    double pe[2], J[4], pd[2], us[2], e0, e1;

    fkin(pe, J, x);
    target(pd, us, T, userparam);
    e0 = UP(UP_QP) * (pe[0] - pd[0]);
    e1 = UP(UP_QP + 1) * (pe[1] - pd[1]);
    out[0] = vs * (J[0] * e0 + J[2] * e1);
    out[1] = vs * (J[1] * e0 + J[3] * e1);
    out[2] = 0.0;
    out[3] = 0.0;
    out[4] = 0.0;
    out[5] = vs * UP(UP_QF) * (x[5] - UP(UP_FDES));
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
void hfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dh/dx multiplied by vector vec, i.e. (dh/dx)^T*vec or vec^T*(dg/dx) **/
void dhdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}
/** Jacobian dh/du multiplied by vector vec, i.e. (dh/du)^T*vec or vec^T*(dg/du) **/
void dhdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
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
