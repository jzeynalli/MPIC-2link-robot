# vi_mpic: VI-MPIC draft for the 2-link robot

Variable-impedance MPIC (VI-MPIC) drafted from the formulation notes and the IROS
2027 analysis, together with a benchmark against classical MPIC on a rigid-body
plant. The first draft, its setup and results are in
[docs/vi-mpic.md](../docs/vi-mpic.md); the improved version (v2), the 1 kHz
contact reflex and the sliding task are in [docs/vi-mpic-v2.md](../docs/vi-mpic-v2.md).

Run everything from this folder. It has its own `+CmexFiles` package, so keep the
repository root off the MATLAB path.

| file | content |
|---|---|
| `probfct_VI_MPIC.c` | GRAMPC problem: VI-MPIC model (8 states, 4 inputs), gates, stiffness cost, analytic adjoints. With `UP_VARK = 0` and the measured gate it is classical MPIC |
| `vi_params.m` | all parameters: plant, wall, belief, task, MPC, cost |
| `vi_variants.m` | controller variants: fixed-gain sweep, VI-MPIC, its ablations and the v2 improvement ladder, schedules; every variant also with the 1 kHz reflex (`*_rfx`) |
| `vi_index.m` | layout of the GRAMPC `userparam` vector (mirror of the `UP_*` defines) |
| `vi_init_grampc.m` | GRAMPC struct of one variant |
| `vi_simulate.m` | closed loop: belief update, prestep, GRAMPC, rigid-body plant with the torque law of the notes (38); v2 options and the reflex; sliding task over a wavy wall |
| `vi_metrics.m` | tracking error, force peak, approach and settling time, solve time |
| `vi_gate.m` | MATLAB mirror of the contact gate and penetration moments |
| `vi_kin.m`, `vi_ik.m` | kinematics and mass matrix of the arm |
| `vi_plot_runs.m` | time histories of runs |
| `vi_build.m` | compiles the problem into `+CmexFiles` |
| `check_vi_gradients.m` | adjoints against finite differences (GRAMPC gradient check) |
| `check_vi_mpic.m` | gate moments (Monte Carlo), overdamping floors, E1 force-model check, E5 convergence |
| `run_vi_benchmark.m` | first-draft sweep, writes `results/vi_benchmark_*.csv` and `docs/figures/vi_*.png` |
| `run_vi_sweep.m` | v2 sweeps: impact or sliding task, any list of variants, one CSV per call (parallel MATLAB processes) |
| `vi_report_v2.m` | merges the sweeps, prints the v2 tables, writes `docs/figures/vi2_*.png` |

```matlab
vi_build
check_vi_gradients
check_vi_mpic
run_vi_benchmark          % first draft: 864 closed-loop runs, about 30-40 minutes
```

The v2 sweeps, as run for docs/vi-mpic-v2.md (four processes in parallel,
about 30 minutes):

```matlab
off9 = 0.01 * (-2:0.5:2);  off5 = 0.01 * (-2:1:2);
run_vi_sweep('impact', {'mpic_k300_rfx', 'mpic_k500_rfx', 'mpic_k800_rfx', 'mpic_k1200_rfx', ...
    'mpic_k2000_rfx', 'mpic_k3000_rfx', 'mpic_k5000_rfx', 'mpic_k10000_rfx'}, [0.1 0.2 0.3], [10 50], off9, 'results/v2_impact_fixed_rfx.csv');
run_vi_sweep('impact', {'vimpic_cg', 'vimpic_cg_hold', 'vimpic_cg_hold_fl', 'vimpic_v2', 'sched_contact'}, ...
    [0.1 0.2 0.3], [10 50], off9, 'results/v2_impact_ladder.csv');
run_vi_sweep('impact', {'vimpic_rfx', 'vimpic_cg_rfx', 'vimpic_cg_hold_rfx', 'vimpic_cg_hold_fl_rfx', ...
    'vimpic_v2_rfx', 'sched_dist_rfx', 'sched_contact_rfx'}, [0.1 0.2 0.3], [10 50], off9, 'results/v2_impact_ladder_rfx.csv');
run_vi_sweep('slide', {'mpic_k300_rfx', 'mpic_k500_rfx', 'mpic_k800_rfx', 'mpic_k1200_rfx', 'mpic_k2000_rfx', ...
    'mpic_k3000_rfx', 'mpic_k5000_rfx', 'mpic_k10000_rfx', 'sched_contact_rfx', 'vimpic_rfx', 'vimpic_v2_rfx', ...
    'mpic_k800', 'mpic_k10000', 'vimpic', 'vimpic_v2', 'sched_contact'}, 0.2, [10 50], off5, 'results/v2_slide.csv');
vi_report_v2
```
