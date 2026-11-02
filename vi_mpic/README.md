# vi_mpic: VI-MPIC draft for the 2-link robot

Variable-impedance MPIC (VI-MPIC) drafted from the formulation notes and the IROS
2027 analysis, together with a benchmark against classical MPIC on a rigid-body
plant. The formulation, the setup and the results are in
[docs/vi-mpic.md](../docs/vi-mpic.md).

Run everything from this folder. It has its own `+CmexFiles` package, so keep the
repository root off the MATLAB path.

| file | content |
|---|---|
| `probfct_VI_MPIC.c` | GRAMPC problem: VI-MPIC model (8 states, 4 inputs), gates, stiffness cost, analytic adjoints. With `UP_VARK = 0` and the measured gate it is classical MPIC |
| `vi_params.m` | all parameters: plant, wall, belief, task, MPC, cost |
| `vi_variants.m` | controller variants: fixed-gain sweep, VI-MPIC and ablations, schedules |
| `vi_index.m` | layout of the GRAMPC `userparam` vector (mirror of the `UP_*` defines) |
| `vi_init_grampc.m` | GRAMPC struct of one variant |
| `vi_simulate.m` | closed loop: belief update, prestep, GRAMPC, rigid-body plant with the torque law of the notes (38) |
| `vi_metrics.m` | tracking error, force peak, approach and settling time, solve time |
| `vi_gate.m` | MATLAB mirror of the contact gate and penetration moments |
| `vi_kin.m`, `vi_ik.m` | kinematics and mass matrix of the arm |
| `vi_plot_runs.m` | time histories of runs |
| `vi_build.m` | compiles the problem into `+CmexFiles` |
| `check_vi_gradients.m` | adjoints against finite differences (GRAMPC gradient check) |
| `check_vi_mpic.m` | gate moments (Monte Carlo), overdamping floors, E1 force-model check, E5 convergence |
| `run_vi_benchmark.m` | full sweep, writes `results/*.csv` and `docs/figures/vi_*.png` |

```matlab
vi_build
check_vi_gradients
check_vi_mpic
run_vi_benchmark          % 864 closed-loop runs, about 30-40 minutes
```
