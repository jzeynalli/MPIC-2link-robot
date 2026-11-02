# fc_mpic: FC-MPIC for the 2-link robot

Friction-consistent MPIC (FC-MPIC) from the proposal of 2 Nov 2026, implemented in
GRAMPC for the 2-link robot. It is compared with classical MPIC (Gold et al.) and with
VI-MPIC as first drafted (branch `vi-mpic`, without the v2 changes), in a sanding task
on a wall with LuGre friction. The formulation, the setup and the results are in
[docs/fc-mpic.md](../docs/fc-mpic.md).

Run everything from this folder. The problems are compiled into the packages `+CmexFC`
and `+CmexVI` (not `+CmexFiles`), so one closed loop can call either.

| file | content |
|---|---|
| `probfct_FC_MPIC.c` | GRAMPC problem, x = [q, qd, ft, fn]: classical MPIC (continuous or discrete time), FC-MPIC (return map, eq. 2.4-2.8) and pure sliding, with the discrete adjoint of Appendix A |
| `fc_params.m` | plant, LuGre friction, low-level controller, task, MPC and cost; friction cases `'nominal'`, `'abrasive'` |
| `fc_variants.m` | controllers: MPIC, MPIC in discrete time, FC-MPIC and its ablations, wrong friction coefficient, VI-MPIC |
| `fc_index.m` | layout of the `userparam` vector of `probfct_FC_MPIC.c` |
| `fc_init_grampc.m` | GRAMPC struct of a variant (either problem) |
| `fc_prestep.m` | frozen model data of an MPC step: stick stiffness, kbar_n, measurement-consistent mu_k |
| `fc_model_step.m` | MATLAB mirror of the discrete-time model (open-loop checks) |
| `fc_plant.m` | rigid-body arm, joint PD, spring-damper wall with LuGre friction |
| `fc_simulate.m` | closed loop of one variant in the sanding task |
| `fc_vi_params.m` | settings of VI-MPIC (from `vi_params`) with the shared plant, task, cost and MPC values |
| `fc_metrics.m` | tracking and normal-force errors, overall and at stroke reversals |
| `run_fc_benchmark.m` | one benchmark set, writes `results/fc_<set>_it<iters>.csv` |
| `fc_report.m` | trade-off tables and figures from all result files |
| `fc_prediction.m` | open-loop prediction accuracy of the force models over the horizon |
| `fc_plot_runs.m` | time histories over one stroke period |
| `fc_timing.m` | GRAMPC time per step, measured one controller at a time |
| `check_fc_model.m` | friction loop against the plant (Prop. 2.4), Props. 2.1 and 2.3, mirror and equivalence checks |
| `check_fc_gradients.m` | adjoints against finite differences (GRAMPC gradient check) |
| `fc_build.m` | compiles both problems |

```matlab
fc_build
check_fc_gradients
check_fc_model(true)
fc_prediction(true)
for it = [10 50]
    for s = {'joint', 'joint_hi', 'kq', 'cart', 'cart_d', 'mu', 'decoupled', 'stroke'}
        run_fc_benchmark(s{1}, it);           % 1752 runs in total
    end
end
fc_report(true)
fc_plot_runs('nominal', 1e5, 10, '../docs/figures/fc_runs_nominal.png')
fc_timing                                     % with no other MATLAB job running
```
