# MPIC-2link-robot

Nonlinear model predictive control (MPC) of a 2-link planar robot in contact with its
environment (a wall and a circular object), built on [GRAMPC v2.3](https://github.com/grampc/grampc).

The MPC runs in a **cascade** around a joint-space impedance controller. It plans the joint
reference rate `q̇_d`, and the contact force `F` is part of the prediction model. This lets the
controller move to a target position in free space or apply a desired force on a surface.

## Repository layout

```
.
├── startMPC.m                      # entry point: build, simulate, plot, animate
├── initData.m                      # GRAMPC options, setpoints, weights, gains
├── probfct_2LINKROBOT_cascaded.c   # GRAMPC problem: dynamics, cost, gradients
├── run_2link_planar.m              # symbolic derivation of the C expressions
├── animateTwoLinkPlanar.m          # animation and video export
├── forward_kinematics.m, inverse_kinematics.m, jacobian.m
├── twoLinkPlanarODE.m, cartesianJTController.m, printTwoLinkXdot.m   # auxiliary
├── docs/                           # documentation
├── vi_mpic/                        # VI-MPIC draft and benchmark (branch vi-mpic)
└── external/grampc/                # GRAMPC v2.3 (git submodule, unmodified)
```

## Quick start

Requirements: MATLAB with a MEX C compiler (`mex -setup C`), Git, and the Symbolic Math
Toolbox (only to regenerate derivatives).

```bash
git clone --recursive https://github.com/jzeynalli/MPIC-2link-robot.git
```

In MATLAB, with the repository root as the current folder:

```matlab
startMPC(1, 2, 2);   % first run: compile GRAMPC + problem, contact scenario
startMPC(1, 1);      % free-motion scenario, no recompilation
```

`startMPC(figNr, md, compile)`: `md = 1` free motion, `md = 2` contact with force feedback;
`compile = 0/1/2` builds nothing / the problem function / GRAMPC and the problem function.

## Documentation

- [Usage guide](docs/usage.md): setup, running, scenarios, common changes, troubleshooting
- [Model and OCP](docs/model.md): states, dynamics, cost function, parameter vector, contact model
- [Code reference](docs/code-reference.md): every file and function
- [VI-MPIC draft and evaluation](docs/vi-mpic.md): variable-impedance MPIC for this robot and a
  benchmark against classical MPIC (code in [vi_mpic/](vi_mpic))
- [VI-MPIC v2](docs/vi-mpic-v2.md): what makes VI-MPIC better, a 1 kHz contact reflex, and a
  sliding task over an uncertain surface

## Author

Jahandar Zeynalli

## Acknowledgements and license

This project is released under the BSD-3-Clause license, see [LICENSE](LICENSE).

It uses [GRAMPC](https://github.com/grampc/grampc) (BSD-3-Clause) as an unmodified git
submodule. `startMPC.m`, `initData.m` and `probfct_2LINKROBOT_cascaded.c` are based on the
GRAMPC example templates and keep the GRAMPC copyright notice. `forward_kinematics.m`,
`inverse_kinematics.m` and `jacobian.m` come from the Robotics 1 lecture
(© 2023 Andreas Völz). If you use GRAMPC results, please cite:

> T. Englert, A. Völz, F. Mesmer, S. Rhein, K. Graichen: *A software framework for embedded
> nonlinear model predictive control using a gradient-based augmented Lagrangian approach
> (GRAMPC)*. Optimization and Engineering 20, 769–809 (2019).
> https://doi.org/10.1007/s11081-018-9417-2
