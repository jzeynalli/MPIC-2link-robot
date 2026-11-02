function Pv = fc_vi_params(P)
%FC_VI_PARAMS Parameters of VI-MPIC (first draft) in the sanding task.
%   VI-MPIC keeps its own settings from vi_params (stiffness bounds and
%   rates, risk cost, belief update). The values that every controller of
%   the FC-MPIC study shares (PD bandwidth, model wall stiffness, desired
%   force, cost weights, MPC settings) are taken from P. As in vi_params,
%   the risk weights equal the force and position weights (q_F = QF,
%   q_p = Qp).

Pv = P.vi;
Pv.omega           = P.omega;
Pv.ke_model        = P.ke_model;
Pv.belief.p_hat    = P.wall.x;
Pv.task.F_des      = P.task.F_des;
Pv.task.v_app      = P.cost.v_app;
Pv.cost.Qp         = P.cost.Qp;
Pv.cost.QF         = [P.cost.QF 0];
Pv.cost.RQ         = P.cost.RQ;
Pv.cost.Qv         = P.cost.Qv;
Pv.cost.Vscale     = P.cost.Vscale;
Pv.cost.qF         = P.cost.QF;
Pv.cost.qp         = P.cost.Qp(2);
Pv.mpc.dt          = P.mpc.dt;
Pv.mpc.Thor        = P.mpc.Thor;
Pv.mpc.Nhor        = P.mpc.Nhor_cont;
Pv.mpc.MaxGradIter = P.mpc.MaxGradIter;
Pv.mpc.uq_max      = P.mpc.uq_max;
end
