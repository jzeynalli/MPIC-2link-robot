# GRAMPC example: 2-link planar robot with cascaded MPC and surface contact

Nonlinear model predictive control (MPC) of a 2-link planar robot arm that makes contact with its environment (a wall and a circular object). It is built on [GRAMPC v2.3](https://github.com/grampc/grampc). The MPC runs in a cascade around a joint-space impedance/PD controller. Contact forces come from a stiffness model of the environment and are fed back into the MPC through the effective stiffness `K_bar = (I + Ke·J·Kp⁻¹·Jᵀ)⁻¹·Ke`.

## Repository layout

```
.
├── startMPC.m                      # main entry point: compile, simulate, plot, animate
├── initData.m                      # GRAMPC options, parameters and user parameters
├── probfct_2LINKROBOT_cascaded.c   # GRAMPC problem description (dynamics, cost, constraints)
├── run_2link_planar.m              # symbolic derivation (Symbolic Math Toolbox) of model terms
├── animateTwoLinkPlanar.m          # animation / video export
├── forward_kinematics.m, inverse_kinematics.m, jacobian.m, ...
├── fixedsize_settings.h            # GRAMPC fixed-size build settings
└── external/grampc/                # GRAMPC v2.3 (git submodule, unmodified)
```

## Requirements

- MATLAB (tested with R2025b) and a C compiler set up for MEX (`mex -setup C`)
- Symbolic Math Toolbox (only for `run_2link_planar.m`)
- Git

## Getting started

Clone the repository with its GRAMPC submodule:

```bash
git clone --recursive https://github.com/<your-user>/grampc-2link-cascaded-surface.git
```

If you already cloned it without `--recursive`:

```bash
git submodule update --init
```

> On Windows, run `git config --global core.longpaths true` first if the clone fails with *Filename too long*.

Then, in MATLAB, with the **repository root** as the current folder:

```matlab
% first run: compile the GRAMPC toolbox and the problem function
startMPC(1, 2, 2);

% later runs (nothing is recompiled)
startMPC(1, 2);
```

Arguments of `startMPC(figNr, md, compile)`:

| Argument  | Meaning |
|-----------|---------|
| `figNr`   | number of the first figure (`0` turns plotting off) |
| `md`      | `1`: free motion, no contact force feedback; `2`: wall/circle contact with force feedback |
| `compile` | `0`: no compilation, `1`: problem function only, `2`: GRAMPC toolbox and problem function |

Compiled files (`+CmexFiles/`, `external/grampc/matlab/bin/`) and the recorded animation (`robot_sim.mp4`) are generated locally and are not tracked by git.

## Citation

If you use this example, please also cite GRAMPC:

> T. Englert, A. Völz, F. Mesmer, S. Rhein, K. Graichen: *A software framework for embedded nonlinear model predictive control using a gradient-based augmented Lagrangian approach (GRAMPC)*. Optimization and Engineering 20, 769–809 (2019). https://doi.org/10.1007/s11081-018-9417-2

## License

The example code in this repository is released under the BSD-3-Clause license, see [LICENSE](LICENSE). Parts of `startMPC.m` and `initData.m` are derived from the GRAMPC templates and keep the GRAMPC copyright notice. GRAMPC itself is included as an unmodified submodule under its own [BSD-3-Clause license](https://github.com/grampc/grampc/blob/v2.3/LICENSE.txt).
