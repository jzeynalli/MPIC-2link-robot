/* This file is part of GRAMPC - (https://github.com/grampc/grampc)
 *
 * GRAMPC -- A software framework for embedded nonlinear model predictive
 * control using a gradient-based augmented Lagrangian approach
 *
 * Copyright 2014-2025 by Knut Graichen, Andreas Voelz, Thore Wietzke,
 * Tobias Englert (<v2.3), Felix Mesmer (<v2.3), Soenke Rhein (<v2.3),
 * Bartosz Kaepernick (<v2.0), Tilman Utz (<v2.0).
 *
 * GRAMPC is distributed under the BSD-3-Clause license, see LICENSE.txt
 *
 *
 *
 *
 *
 *
 *
 *
 * This probfct-file describes the single-axis ball on plate problem from
 * Richter, S.: Computational complexity certifcation of gradient methods
 * for real-time model predictive control. Ph.D. thesis, ETH Zürich (2012).
 *  
 *                                           _T
 *                                          /
 *      min    J(u,p,T;x0) = V(T,x(T),p) + / l(t,x(t),u(t),p) dt
 *   u(.),p,T                            _/
 *                                      0
 *             .
 *      s.t.   x(t) = f(t0+t,x(t),u(t),p), x(0) = x0
 *             x_min <= x(t) <= x_max
 *             u_min <= u(t) <= u_max
 *
 */


#include "probfct.h"

 /* square macro */
#define POW2(a) ((a)*(a))

/** OCP dimensions: states (Nx), controls (Nu), parameters (Np), equalities (Ng), 
    inequalities (Nh), terminal equalities (NgT), terminal inequalities (NhT) **/
void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT, typeUSERPARAM *userparam)
{
	*Nx = 6;
	*Nu = 2;
	*Np = 0;
	*Nh = 0;
	*Ng = 0;
	*NgT = 0;
	*NhT = 0;
}


/** System function f(t,x,u,p,param,userparam) 
    ------------------------------------ **/
void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{

    // Fixed link lengths
    const ctypeRNum l1 = 0.2;
    const ctypeRNum l2 = 0.2;
    
    const ctypeRNum kp1 = ((ctypeRNum*)userparam)[18];
    const ctypeRNum kp2 = ((ctypeRNum*)userparam)[19];
    const ctypeRNum kd1 = ((ctypeRNum*)userparam)[20];
    const ctypeRNum kd2 = ((ctypeRNum*)userparam)[21];
    const ctypeRNum kex = ((ctypeRNum*)userparam)[22];
    const ctypeRNum kexy = ((ctypeRNum*)userparam)[23];
    const ctypeRNum keyx = ((ctypeRNum*)userparam)[24];
    const ctypeRNum key = ((ctypeRNum*)userparam)[25];
    
    // J(q)
    const double J11 = -l1*sin(x[0]) - l2*sin(x[0] + x[1]);
    const double J12 = -l2*sin(x[0] + x[1]);
    const double J21 =  l1*cos(x[0]) + l2*cos(x[0] + x[1]);
    const double J22 =  l2*cos(x[0] + x[1]);
    
    out[0] = u[0] + (kp1/kd1)*(x[2]-x[0])- (J11*x[4] + J21*x[5])/kd1;
    out[1] = u[1] + (kp2/kd2)*(x[3]-x[1])- (J12*x[4] + J22*x[5])/kd2;
    out[2] = u[0];
    out[3] = u[1];
    out[4] = kex*(J11*u[0] + J12*u[1]) + kexy*(J21*u[0] + J22*u[1]);
    out[5] = keyx*(J11*u[0] + J12*u[1]) + key*(J21*u[0] + J22*u[1]);

}

/** Jacobian df/dx multiplied by vector vec, i.e. (df/dx)^T*vec or vec^T*(df/dx) **/
void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{

    const ctypeRNum kp1 = ((ctypeRNum*)userparam)[18];
    const ctypeRNum kp2 = ((ctypeRNum*)userparam)[19];
    const ctypeRNum kd1 = ((ctypeRNum*)userparam)[20];
    const ctypeRNum kd2 = ((ctypeRNum*)userparam)[21];
    const ctypeRNum kex = ((ctypeRNum*)userparam)[22];
    const ctypeRNum kexy = ((ctypeRNum*)userparam)[23];
    const ctypeRNum keyx = ((ctypeRNum*)userparam)[24];
    const ctypeRNum key = ((ctypeRNum*)userparam)[25];

    out[0] = (vec[1]*((x[4]*cos(x[0] + x[1]))/5 + (x[5]*sin(x[0] + x[1]))/5))/kd2 - vec[4]*(kex*(u[0]*(cos(x[0] + x[1])/5 + cos(x[0])/5) + (u[1]*cos(x[0] + x[1]))/5) + kexy*((u[1]*sin(x[0] + x[1]))/5 + u[0]*(sin(x[0] + x[1])/5 + sin(x[0])/5))) - vec[5]*(keyx*(u[0]*(cos(x[0] + x[1])/5 + cos(x[0])/5) + (u[1]*cos(x[0] + x[1]))/5) + key*((u[1]*sin(x[0] + x[1]))/5 + u[0]*(sin(x[0] + x[1])/5 + sin(x[0])/5))) - vec[0]*(kp1/kd1 - (x[4]*(cos(x[0] + x[1])/5 + cos(x[0])/5) + x[5]*(sin(x[0] + x[1])/5 + sin(x[0])/5))/kd1);         
    out[1] = (vec[0]*((x[4]*cos(x[0] + x[1]))/5 + (x[5]*sin(x[0] + x[1]))/5))/kd1 - vec[4]*(kex*((u[0]*cos(x[0] + x[1]))/5 + (u[1]*cos(x[0] + x[1]))/5) + kexy*((u[0]*sin(x[0] + x[1]))/5 + (u[1]*sin(x[0] + x[1]))/5)) - vec[5]*(keyx*((u[0]*cos(x[0] + x[1]))/5 + (u[1]*cos(x[0] + x[1]))/5) + key*((u[0]*sin(x[0] + x[1]))/5 + (u[1]*sin(x[0] + x[1]))/5)) - vec[1]*(kp2/kd2 - ((x[4]*cos(x[0] + x[1]))/5 + (x[5]*sin(x[0] + x[1]))/5)/kd2);
    out[2] = (kp1*vec[0])/kd1;
    out[3] = (kp2*vec[1])/kd2;
    out[4] = (vec[1]*sin(x[0] + x[1]))/(5*kd2) + (vec[0]*(sin(x[0] + x[1])/5 + sin(x[0])/5))/kd1;
    out[5] = - (vec[0]*(cos(x[0] + x[1])/5 + cos(x[0])/5))/kd1 - (vec[1]*cos(x[0] + x[1]))/(5*kd2);

    }

/** Jacobian df/du multiplied by vector vec, i.e. (df/du)^T*vec or vec^T*(df/du) **/
void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    const ctypeRNum kp1 = ((ctypeRNum*)userparam)[18];
    const ctypeRNum kp2 = ((ctypeRNum*)userparam)[19];
    const ctypeRNum kd1 = ((ctypeRNum*)userparam)[20];
    const ctypeRNum kd2 = ((ctypeRNum*)userparam)[21];
    const ctypeRNum kex = ((ctypeRNum*)userparam)[22];
    const ctypeRNum kexy = ((ctypeRNum*)userparam)[23];
    const ctypeRNum keyx = ((ctypeRNum*)userparam)[24];
    const ctypeRNum key = ((ctypeRNum*)userparam)[25];

	out[0] = vec[0] + vec[2] + vec[4]*(kexy*(cos(x[0] + x[1])/5 + cos(x[0])/5) - kex*(sin(x[0] + x[1])/5 + sin(x[0])/5)) + vec[5]*(key*(cos(x[0] + x[1])/5 + cos(x[0])/5) - keyx*(sin(x[0] + x[1])/5 + sin(x[0])/5));
    out[1] = vec[1] + vec[3] + vec[4]*((kexy*cos(x[0] + x[1]))/5 - (kex*sin(x[0] + x[1]))/5) + vec[5]*((key*cos(x[0] + x[1]))/5 - (keyx*sin(x[0] + x[1]))/5);


}
/** Jacobian df/dp multiplied by vector vec, i.e. (df/dp)^T*vec or vec^T*(df/dp) **/
void dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}


/** Integral cost l(t,x(t),u(t),p,param,userparam) 
    -------------------------------------------------- **/
void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
	ctypeRNum* pCost = (ctypeRNum*)userparam;
    ctypeRNum* xdes = param->xdes;
    ctypeRNum* udes = param->udes;

    const ctypeRNum px_d  = 0.2 * cos(xdes[0]) + 0.2*cos(xdes[0]+xdes[1]);
    const ctypeRNum py_d  = 0.2 * sin(xdes[0]) + 0.2*sin(xdes[0]+xdes[1]);

    const ctypeRNum px = 0.2*cos(x[0]) + 0.2*cos(x[0]+x[1]);
    const ctypeRNum py = 0.2*sin(x[0]) + 0.2*sin(x[0]+x[1]);

    out[0] = (pCost[0] * POW2(x[0] - xdes[0])
            + pCost[1] * POW2(x[1] - xdes[1])
            + pCost[2] * POW2(x[2] - xdes[2])
            + pCost[3] * POW2(x[3] - xdes[3])
            + pCost[4] * POW2(x[4] - xdes[4])
            + pCost[5] * POW2(x[5] - xdes[5])
            + pCost[6] * POW2(u[0] - udes[0])
            + pCost[7] * POW2(u[1] - udes[1])
            + pCost[26] * POW2(px - px_d)
            + pCost[27] * POW2(py - py_d))
            / 2.0;
}
/** Gradient dl/dx **/
void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
	ctypeRNum* pCost = (ctypeRNum*)userparam;
    ctypeRNum* xdes = param->xdes;
    ctypeRNum* udes = param->udes;

    
	out[0] = pCost[0] * (x[0] - xdes[0]) - pCost[26]*(sin(x[0] + x[1])/5 + sin(x[0])/5)*(cos(x[0] + x[1])/5 - cos(xdes[0] + xdes[1])/5 + cos(x[0])/5 - cos(xdes[0])/5) + pCost[27]*(cos(x[0] + x[1])/5 + cos(x[0])/5)*(sin(x[0] + x[1])/5 - sin(xdes[0] + xdes[1])/5 + sin(x[0])/5 - sin(xdes[0])/5);
    out[1] = pCost[1] * (x[1] - xdes[1]) - (pCost[26]*sin(x[0] + x[1])*(cos(x[0] + x[1])/5 - cos(xdes[0] + xdes[1])/5 + cos(x[0])/5 - cos(xdes[0])/5))/5 + (pCost[27]*cos(x[0] + x[1])*(sin(x[0] + x[1])/5 - sin(xdes[0] + xdes[1])/5 + sin(x[0])/5 - sin(xdes[0])/5))/5;
    out[2] = pCost[2] * (x[2] - xdes[2]);
    out[3] = pCost[3] * (x[3] - xdes[3]);
    out[4] = pCost[4] * (x[4] - xdes[4]);
    out[5] = pCost[5] * (x[5] - xdes[5]);
}
/** Gradient dl/du **/
void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
	ctypeRNum* pCost = (ctypeRNum*)userparam;
    ctypeRNum* udes = param->udes;

	out[0] = pCost[6]*(u[0] - udes[0]);
    out[1] = pCost[7]*(u[1] - udes[1]);
}
/** Gradient dl/dp **/
void dldp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
}


/** Terminal cost V(T,x(T),p,param,userparam) 
    ---------------------------------------- **/
void Vfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
	ctypeRNum* pCost = (ctypeRNum*)userparam;
    ctypeRNum* xdes = param->xdes;

    const ctypeRNum px_d  = 0.2 * cos(xdes[0]) + 0.2*cos(xdes[0]+xdes[1]);
    const ctypeRNum py_d  = 0.2 * sin(xdes[0]) + 0.2*sin(xdes[0]+xdes[1]);

    const ctypeRNum px = 0.2*cos(x[0]) + 0.2*cos(x[0]+x[1]);
    const ctypeRNum py = 0.2*sin(x[0]) + 0.2*sin(x[0]+x[1]);

	out[0] = (pCost[8] * POW2(x[0] - xdes[0])
            + pCost[9] * POW2(x[1] - xdes[1])
            + pCost[10] * POW2(x[2] - xdes[2])
            + pCost[11] * POW2(x[3] - xdes[3])
            + pCost[12] * POW2(x[4] - xdes[4])
            + pCost[13] * POW2(x[5] - xdes[5])
            + pCost[28] * POW2(px - px_d)
            + pCost[29] * POW2(py - py_d))
            / 2.0;
}
/** Gradient dV/dx **/
void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
	ctypeRNum* pCost = (ctypeRNum*)userparam;
    ctypeRNum* xdes = param->xdes;

	out[0] = pCost[8]  * (x[0] - xdes[0]) - pCost[28]*(sin(x[0] + x[1])/5 + sin(x[0])/5)*(cos(x[0] + x[1])/5 - cos(xdes[0] + xdes[1])/5 + cos(x[0])/5 - cos(xdes[0])/5) + pCost[29]*(cos(x[0] + x[1])/5 + cos(x[0])/5)*(sin(x[0] + x[1])/5 - sin(xdes[0] + xdes[1])/5 + sin(x[0])/5 - sin(xdes[0])/5);
    out[1] = pCost[9]  * (x[1] - xdes[1]) - (pCost[28]*sin(x[0] + x[1])*(cos(x[0] + x[1])/5 - cos(xdes[0] + xdes[1])/5 + cos(x[0])/5 - cos(xdes[0])/5))/5 + (pCost[29]*cos(x[0] + x[1])*(sin(x[0] + x[1])/5 - sin(xdes[0] + xdes[1])/5 + sin(x[0])/5 - sin(xdes[0])/5))/5;
    out[2] = pCost[10] * (x[2] - xdes[2]);
    out[3] = pCost[11] * (x[3] - xdes[3]);
    out[4] = pCost[12] * (x[4] - xdes[4]);
    out[5] = pCost[13] * (x[5] - xdes[5]);
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
    //out[0] = x[4];
    //out[1] = x[5];
}
/** Jacobian dg/dx multiplied by vector vec, i.e. (dg/dx)^T*vec or vec^T*(dg/dx) **/
void dgdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    //out[0] = vec[0];
    //out[1] = vec[1];
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

    // const ctypeRNum objx = ((ctypeRNum*)userparam)[14];
    // const ctypeRNum objy = ((ctypeRNum*)userparam)[15];
    // const ctypeRNum objr = ((ctypeRNum*)userparam)[16];
    // const ctypeRNum objd = ((ctypeRNum*)userparam)[17];
    // 
    // const ctypeRNum l1 = 0.2;
    // const ctypeRNum l2 = 0.2;
    // 
    // const double s1  = sin(x[0]);
    // const double c1  = cos(x[0]);
    // const double s12 = sin(x[0] + x[1]);
    // const double c12 = cos(x[0] + x[1]);
    // 
    // const double px = l1 * c1 + l2 * c12;
    // const double py = l1 * s1 + l2 * s12;
    // 
    // out[0] = POW2(objr + objd) - (POW2(px-objx) + POW2(py-objy));


}
/** Jacobian dh/dx multiplied by vector vec, i.e. (dh/dx)^T*vec or vec^T*(dg/dx) **/
void dhdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, const typeGRAMPCparam *param, typeUSERPARAM *userparam)
{
    // const ctypeRNum objx = ((ctypeRNum*)userparam)[14];
    // const ctypeRNum objy = ((ctypeRNum*)userparam)[15];
    // const ctypeRNum objr = ((ctypeRNum*)userparam)[16];
    // const ctypeRNum objd = ((ctypeRNum*)userparam)[17];
    // 
	// out[0] = vec[0]*(2*(sin(x[0] + x[1])/5 + sin(x[0])/5)*(cos(x[0] + x[1])/5 - objx + cos(x[0])/5) - 2*(cos(x[0] + x[1])/5 + cos(x[0])/5)*(sin(x[0] + x[1])/5 - objy + sin(x[0])/5));
    // out[1] = vec[0]*((2*sin(x[0] + x[1])*(cos(x[0] + x[1])/5 - objx + cos(x[0])/5))/5 - (2*cos(x[0] + x[1])*(sin(x[0] + x[1])/5 - objy + sin(x[0])/5))/5);

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
