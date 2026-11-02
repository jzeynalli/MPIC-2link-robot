# Model and optimal control problem

This page describes the model that is implemented in
[`probfct_2LINKROBOT_cascaded.c`](../probfct_2LINKROBOT_cascaded.c) and solved by GRAMPC.

## Cascaded control structure

The MPC does not command joint torques directly. It sits on top of an inner
joint-space impedance (PD) controller with stiffness `Kp` and damping `Kd`.
The MPC plans the **rate of the joint reference** `q̇_d`, and the reference
`q_d` is integrated inside the model. The inner loop then pulls the joints
`q` towards `q_d`, while contact forces `F` push the end effector back.

```
          u = q̇_d          q_d                     q
 MPC ───────────────► ∫ ──────► impedance loop ──────► robot ──► end effector ──► environment
  ▲                              (Kp, Kd)                 ▲                          │
  │                                                       └──────── −Jᵀ(q) F ────────┤
  └────────────────────────── measured / estimated F, q ─────────────────────────────┘
```

## States, inputs, parameters

| Symbol | Index in C (`x[i]`, `u[i]`) | Meaning | Unit |
|---|---|---|---|
| `q1, q2` | `x[0], x[1]` | joint angles | rad |
| `qd1, qd2` | `x[2], x[3]` | joint reference of the inner impedance loop | rad |
| `Fx, Fy` | `x[4], x[5]` | contact force at the end effector (Cartesian) | N |
| `u1, u2` | `u[0], u[1]` | joint reference rate `q̇_d` (MPC input) | rad/s |

Dimensions (`ocp_dim`): `Nx = 6`, `Nu = 2`, `Np = 0`, no equality or inequality constraints
(`Ng = Nh = NgT = NhT = 0`). Inputs are limited by box constraints `umin ≤ u ≤ umax`.

Robot geometry: both links have length `l1 = l2 = 0.2 m`. **These values are hard-coded in
the C file**: as constants in `ffct` and as the factor `/5` (= `0.2`) in all derivative
expressions. See [usage.md](usage.md#changing-the-link-lengths) before changing them.

Forward kinematics and Jacobian:

$$
p(q) = \begin{bmatrix} l_1\cos q_1 + l_2\cos(q_1+q_2) \\ l_1\sin q_1 + l_2\sin(q_1+q_2) \end{bmatrix},
\qquad
J(q) = \frac{\partial p}{\partial q} =
\begin{bmatrix} -l_1 s_1 - l_2 s_{12} & -l_2 s_{12} \\ l_1 c_1 + l_2 c_{12} & l_2 c_{12} \end{bmatrix}
$$

## System dynamics `ffct`

The inner loop is modelled quasi-statically (link inertia is neglected), i.e. from

$$
K_d(\dot q_d - \dot q) + K_p(q_d - q) - J(q)^\top F = 0
$$

follows

$$
\begin{aligned}
\dot q   &= u + K_d^{-1}\big(K_p (q_d - q) - J(q)^\top F\big) \\
\dot q_d &= u \\
\dot F   &= \bar K \, J(q)\, u
\end{aligned}
$$

with `Kp = diag(kp1, kp2)`, `Kd = diag(kd1, kd2)` and the 2×2 contact stiffness

$$
\bar K = \begin{bmatrix} k_{ex} & k_{exy} \\ k_{eyx} & k_{ey} \end{bmatrix}.
$$

The force rate uses the commanded Cartesian velocity `J(q) u`. An earlier variant with
`J(q_d)` is kept as a comment in `run_2link_planar.m`.

`dfdx_vec` and `dfdu_vec` return `(∂f/∂x)ᵀ·vec` and `(∂f/∂u)ᵀ·vec`. They were generated
symbolically with the first section of [`run_2link_planar.m`](../run_2link_planar.m).

## Cost function

Integral cost `lfct`:

$$
l(x,u) = \tfrac12\Big(\sum_{i=1}^{6} Q_i (x_i - x_{des,i})^2
       + \sum_{j=1}^{2} R_j (u_j - u_{des,j})^2
       + Q_{p,x}\,(p_x - p_{x,d})^2 + Q_{p,y}\,(p_y - p_{y,d})^2\Big)
$$

Terminal cost `Vfct`:

$$
V(x(T)) = \tfrac12\Big(\sum_{i=1}^{6} S_i (x_i - x_{des,i})^2
       + S_{p,x}\,(p_x - p_{x,d})^2 + S_{p,y}\,(p_y - p_{y,d})^2\Big)
$$

`p = p(q)` is the current end-effector position and `p_d = p(xdes[0], xdes[1])` is the desired
position computed from the joint setpoint. The joint-space terms (`Q`, `S`) and the
Cartesian terms (`Qp`, `Sp`) can be weighted independently. Gradients `dldx`, `dldu`,
`dVdx` were generated with the second section of `run_2link_planar.m`.

## User parameter vector

All weights and gains are passed to the C code in one vector `userparam`, built in
[`initData.m`](../initData.m) as `[Q R S H Kp Kd Ke Qp Sp]` (30 entries):

| C index (0-based) | MATLAB index (1-based) | Name | Meaning |
|---|---|---|---|
| 0–5   | 1–6   | `Q`  | integral state weights |
| 6–7   | 7–8   | `R`  | integral input weights |
| 8–13  | 9–14  | `S`  | terminal state weights |
| 14–17 | 15–18 | `H`  | obstacle `[objx objy objr objd]` for the (disabled) circular inequality constraint in `hfct` |
| 18–19 | 19–20 | `Kp` | `kp1, kp2`, inner-loop joint stiffness |
| 20–21 | 21–22 | `Kd` | `kd1, kd2`, inner-loop joint damping |
| 22–25 | 23–26 | `Ke` | `kex, kexy, keyx, key`, contact stiffness `K̄` (overwritten online in mode 2) |
| 26–27 | 27–28 | `Qp` | integral Cartesian position weights |
| 28–29 | 29–30 | `Sp` | terminal Cartesian position weights |

## Environment / contact model (MATLAB side)

The environment is not part of the OCP. It is evaluated in `startMPC.m`
(`compute_force`) as a penetration-based spring model:

- **Wall** at `x = wall_x` (between `wall_ymin` and `wall_ymax`):
  `Fx = Ke_wall(1,1)·(p_x − wall_x)` if `p_x ≥ wall_x`, `Fy = 0`.
- **Circular object** with center `c` and radius `r`: if `‖p − c‖ < r`,
  `F = Ke_circle · (r − ‖p − c‖) · n` with the outward normal `n = (p − c)/‖p − c‖`.

In mode 2 this force is written into the force states `x(5:6)` before each MPC step, and the
contact stiffness `K̄` in `userparam(23:26)` is updated by `update_stiffness_bar`.
That function computes the series stiffness of environment and impedance loop,

$$
\bar K = \big(I + K_e\, J K_p^{-1} J^\top\big)^{-1} K_e ,
$$

but **currently overwrites it with `K̄ = K_e`** (line `K_bar = Ke;`), so the pure
environment stiffness is used. Remove that line to use the series stiffness instead.
