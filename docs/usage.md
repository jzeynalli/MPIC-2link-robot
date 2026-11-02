# Usage guide

## 1. Setup

1. Install MATLAB and a C compiler that MEX supports (on Windows e.g. *MATLAB Support for
   MinGW-w64 C/C++ Compiler*). Check with `mex -setup C`.
2. Clone the repository **with** the GRAMPC submodule:

   ```bash
   git clone --recursive https://github.com/jzeynalli/MPIC-2link-robot.git
   ```

   If you cloned without `--recursive`, run `git submodule update --init`.
   On Windows, run `git config --global core.longpaths true` first if you see *Filename too long*.

3. Open MATLAB and set the **repository root** as the current folder. All paths
   (`external/grampc/`, `+CmexFiles/`) are relative to it.

## 2. Running the simulation

```matlab
[vec, grampc, figNr] = startMPC(figNr, md, compile, varargin)
```

| Argument | Values | Default |
|---|---|---|
| `figNr` | number of the first figure; `0` disables the GRAMPC plots | `1` |
| `md` | `1` = free motion, `2` = contact with force feedback | `1` |
| `compile` | `0` = nothing, `1` = problem function, `2` = GRAMPC toolbox + problem function | `0` |
| `varargin` | compiler flags passed to GRAMPC's make scripts, e.g. `'debug'`, `'verbose'` | none |

Typical calls:

```matlab
startMPC(1, 2, 2);   % first time: build GRAMPC and the problem, run contact scenario
startMPC(1, 2, 1);   % after editing probfct_2LINKROBOT_cascaded.c
startMPC(1, 1);      % free-motion scenario, no rebuild
```

The toolbox and problem are also compiled automatically when `external/grampc/matlab/bin`
or `+CmexFiles` does not exist yet.

### What happens in `startMPC`

1. **Build**: compiles GRAMPC (`grampc_make_toolbox`) and the problem function
   (`grampc_make_probfct`) into `+CmexFiles/`.
2. **Init**: `initData(md)` sets up GRAMPC. The options, parameters and the
   gradient check (`grampc_check_gradients_Cmex`) are printed to the console. Check the
   gradient check output after any change to the C file.
3. **MPC loop** (`Tsim = 3 s`, `dt = 10 ms`): at each sampling step
   - *(mode 2 only)* compute the contact force from the current end-effector position, write it
     into the force states `x(5:6)`, and update the contact stiffness `K̄` in `userparam(23:26)`
     (both set to zero when there is no contact),
   - run one GRAMPC iteration (`grampc_run_Cmex`),
   - simulate the plant for one step with `ode45` on the same model (`grampc_ffct_Cmex`).
4. **Plots**: GRAMPC's predicted trajectories, closed-loop trajectories and solver statistics.
5. **Animation**: `animateTwoLinkPlanar` draws the arm, the wall, the circular object, the end-effector
   trace, and the predicted (blue) and calculated (red dashed) contact forces. It is recorded to
   `robot_sim.mp4` in the current folder (set `anim.record = false` to only display).

### Output

`vec` is the GRAMPC solution struct (`grampc_init_struct_sol`) with the time `vec.t`, states
`vec.x` (6×N), inputs `vec.u` (2×N), cost, constraint values and `vec.CPUtime` per step.
`grampc` is the final GRAMPC struct.

## 3. Scenarios

| | Mode 1: free motion | Mode 2: contact |
|---|---|---|
| Start `x0` | `q = qd = [π/2, 0]`, `F = 0` | `q = qd = [-0.6841, 1.3682]` (end effector at `(0.31, 0)`) |
| Target `xdes` | `q = qd = IK(0.31, 0)`, `F = 0` | same joints, **`F_des = [10, -10] N`** |
| Weights | joint reference `Q(3:4) = 2`, Cartesian `Qp = 10` | forces `Q(5:6) = 1`, Cartesian `Qp = 10` |
| Contact stiffness `Ke` | `0` (no force model in the OCP) | `[800 0 0 0]` initially, then updated online |

The target `xdes` in `initData.m` is given in Cartesian coordinates (`xydes`) and converted to
joint angles with `inverse_kinematics` (elbow solution 1).

## 4. Common modifications

### Change the target point
Edit `xydes` in `initData.m`. For mode 2, also set `user.param.x0` to a matching joint
configuration (the values in the comments correspond to the wall and the circle targets).

### Change the desired contact force
Edit `user.param.xdes(5:6)` in the `md == 2` branch of `initData.m`.

### Move or resize the wall / circle
Edit the `p.wall_*` and `p.circle_*` fields at the top of the *MPC loop* section in
`startMPC.m`. The same values are passed to the animation.

### Tune the controller
- Inner loop: `Kp`, `Kd` in `initData.m`.
- MPC: `Thor` (horizon), `Nhor` (discretization points), `MaxGradIter`, the weights
  `Q, R, S, Qp, Sp`, and the input limits `umin/umax`.
- Any other GRAMPC option can be set via `user.opt.*` (see the
  [GRAMPC documentation](https://grampc.github.io/grampc)).

### Change the cost or dynamics
1. Edit the symbolic expressions in `run_2link_planar.m` and run the matching section.
2. Copy the printed C expressions into `probfct_2LINKROBOT_cascaded.c`.
3. Rebuild with `startMPC(1, md, 1)` and check the console output of the gradient check.

### Changing the link lengths
The link lengths `0.2 m` appear in several places that must be kept consistent:
- `probfct_2LINKROBOT_cascaded.c`: `l1, l2` in `ffct`, the literals `0.2` in `lfct`/`Vfct`, and
  the factor `/5` in all derivative functions (regenerate them with `run_2link_planar.m`),
- `run_2link_planar.m`: `l1`, `l2` and the `0.2` factors,
- `initData.m`: `p_robot.length`,
- `startMPC.m`: `p.length`.

### Enable the circular obstacle constraint
`hfct` / `dhdx_vec` contain a commented-out inequality constraint that keeps the end effector
outside a circle `[objx, objy, objr, objd] = H`. To use it, uncomment the code, set `*Nh = 1`
in `ocp_dim`, and fill `H` in `initData.m`.

## 5. Troubleshooting

| Problem | Fix |
|---|---|
| `Undefined function 'grampc_make_toolbox'` / `external/grampc` is empty | The submodule was not cloned: `git submodule update --init`. |
| `Unable to resolve the name 'CmexFiles.grampc_init_Cmex'` | Current folder is not the repository root, or not compiled yet: `startMPC(1, md, 2)`. |
| Wrong results after editing the C file | Rebuild with `compile = 1` and check the gradient check output. |
| `Filename too long` while cloning on Windows | `git config --global core.longpaths true`, then clone again. |
| No compiler found | `mex -setup C` |
