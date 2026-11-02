# Code reference

## Main files

### `startMPC.m`
```matlab
[vec, grampc, figNr] = startMPC(figNr, md, compile, varargin)
```
Entry point: builds, initializes, runs the closed loop, plots and animates. See
[usage.md](usage.md#2-running-the-simulation) for the arguments and the flow.

Local functions:

| Function | Description |
|---|---|
| `[F, l] = compute_force(x_ee, p)` | Penetration-based contact force at end-effector position `x_ee` (2×1) for the wall and the circular object. `l` is a copy of `p`; when the circle is in contact, `l.wall_Ke` is set to the circle stiffness so that `update_stiffness_bar` uses the right `Ke`. Contact with the circle is ignored for distances `≤ 0.01 m` to the center. |
| `grampc = update_stiffness_bar(grampc, q, p)` | Reads `Kp` from `userparam(19:20)`, computes `K̄ = (I + Ke·J·Kp⁻¹·Jᵀ)⁻¹·Ke` at configuration `q`, then (currently) overrides it with `K̄ = Ke` and writes it to `userparam(23:26)`. |

Physical/environment parameters in the struct `p`:

| Field | Default | Meaning |
|---|---|---|
| `length` | `[0.2; 0.2]` | link lengths [m] (must match the C file) |
| `m1, m2, r1, r2, I1, I2, g` | | rigid-body parameters, only used by the helper `twoLinkPlanarODE`/`printTwoLinkXdot` |
| `wall_x, wall_ymin, wall_ymax` | `0.3, -0.4, 0.4` | wall surface position and extent [m] |
| `wall_Ke` | `diag([800, 0])` | wall stiffness [N/m] |
| `circle_center, circle_radius` | `[0.2; -0.2], 0.05` | circular object [m] |
| `circle_Ke` | `diag([800, 800])` | circle stiffness [N/m] |

Plot switches at the top of the file: `PLOT_PRED`, `PLOT_TRAJ`, `PLOT_STAT`, `PLOT_STEPS`
(plot update interval in steps), `PLOT_PAUSE`.

### `initData.m`
```matlab
[grampc, Tsim, grampc_sdata] = initData(md)
```
Creates and configures the GRAMPC struct for mode `md`:
initial state `x0`, setpoints `xdes`/`udes`, input limits, horizon `Thor = 0.5 s`, sampling time
`dt = 0.01 s`, `Nhor = 21`, `MaxGradIter = 10`, and the user parameter vector
`[Q R S H Kp Kd Ke Qp Sp]` ([layout](model.md#user-parameter-vector)). Estimates `PenaltyMin`
and returns the simulation time `Tsim = 3 s`. With three outputs it also creates the
Simulink data struct `grampc_sdata`.

### `probfct_2LINKROBOT_cascaded.c`
GRAMPC problem description ([math](model.md)).

| Function | Content |
|---|---|
| `ocp_dim` | `Nx = 6, Nu = 2`, no parameters or constraints |
| `ffct` | cascaded impedance model with force dynamics |
| `dfdx_vec`, `dfdu_vec` | transposed Jacobians times vector |
| `lfct`, `dldx`, `dldu` | integral cost (joint + Cartesian tracking, input penalty) and gradients |
| `Vfct`, `dVdx` | terminal cost and gradient |
| `hfct`, `dhdx_vec` | circular obstacle constraint, **commented out** |
| others | empty: not used by this problem (`dfdp_vec`, `gfct`, `gTfct`, `hTfct`, RODAS functions, ...) |

### `animateTwoLinkPlanar.m`
```matlab
animateTwoLinkPlanar(t, X, p, anim)
```
Animates the arm from the time vector `t` (N×1) and state history `X` (N×≥2, columns 1–2 are
`q1, q2`). Options in `anim` (all optional):

| Field | Default | Meaning |
|---|---|---|
| `fps` | `60` | animation / video frame rate |
| `showTrace`, `traceMode`, `traceLength` | `true`, `'tail'`, `250` | end-effector trace (`'tail'` or `'persistent'`) |
| `axisPadding` | `0.1` | extra space around the workspace [m] |
| `title` | `'2-Link Planar Arm'` | figure title |
| `wall_x, wall_ymin, wall_ymax` | | draw the wall |
| `circle_center, circle_radius` | | draw the circular object |
| `F_mpc_x/y`, `F_calc_x/y` | | force arrows and text overlay (model force vs. calculated force) |
| `record`, `outputFile`, `videoFormat`, `videoQuality` | `false`, `'animation'`, `'MPEG-4'`, `85` | video export via `VideoWriter` |

### `run_2link_planar.m`
Symbolic Math Toolbox script, split into sections. Run each section on its own:
1. Dynamics `f(x,u)` and the products `(∂f/∂x)ᵀ·vec`, `(∂f/∂u)ᵀ·vec`, printed as C code
   (`x[i]`, `u[i]`, `vec[i]`) for `dfdx_vec` / `dfdu_vec`. Also derives the
   `K̄` formula symbolically.
2. Integral cost `l(x,u)` and its gradients, printed with GRAMPC names
   (`pCost[i]`, `xdes[i]`) for `dldx` / `dldu` (also used for `dVdx`).
3. Linearization `A = ∂f/∂x`, `B = ∂f/∂u` of the full rigid-body model (reference only).

## Kinematics helpers

| Function | Description |
|---|---|
| `x = forward_kinematics(q, p)` | end-effector position `[x; y]` for joint angles `q` |
| `q = inverse_kinematics(x, p)` | joint solutions for position `x`. Returns 2×2 (two elbow solutions as columns), 2×1 at the workspace boundary, or `[]` if unreachable |
| `J = jacobian(q, p)` | 2×2 position Jacobian. Rotated into a task frame if `p.use_task_space` and `p.task_space_rotation` are set |

## Auxiliary files (not used by `startMPC`)

These files are kept for reference and for a torque-level version of the robot:

| File | Description |
|---|---|
| `twoLinkPlanarODE.m` | full rigid-body dynamics `M(q)q̈ + C(q,q̇)q̇ + G(q) = τ`, for use with `ode45` and a torque law `u_fun(t,x)` |
| `cartesianJTController.m` | Cartesian PD controller mapped to joint torques with `τ = JᵀF − D_q q̇`, with optional saturation |
| `printTwoLinkXdot.m` | prints the rigid-body dynamics symbolically in C syntax (`x[i]`, `u[i]`) |
| `ffct.m` | leftover from the GRAMPC template (2-state system), not related to this robot |
| `fixedsize_settings.h` | GRAMPC fixed-size build settings from the template. Only used if GRAMPC is compiled with `FIXEDSIZE`, and does **not** match this problem (`NX 2`, `NU 1`) |
