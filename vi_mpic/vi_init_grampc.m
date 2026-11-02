function grampc = vi_init_grampc(P, V, x0)
%VI_INIT_GRAMPC GRAMPC struct for controller variant V at the initial state x0.
%   The per-step entries of userparam (J0, belief, target, ...) are filled
%   by vi_simulate before every MPC step.

I = vi_index();
up = zeros(1, I.len);
up(I.omega)  = P.omega;
up(I.ke)     = [P.ke_model 0];
up(I.J0)     = [1 0 0 1];
up(I.J0inv)  = [1 0 0 1];
up(I.phat)   = P.belief.p_hat;
up(I.sigma)  = P.belief.sigma0;
up(I.ell)    = -1e3;
up(I.gmode)  = V.gate;
up(I.psi)    = V.psi;
up(I.Qp)     = P.cost.Qp;
up(I.pdes)   = P.task.pA';
up(I.pxmax)  = P.task.p_cap;
up(I.QF)     = P.cost.QF;
up(I.Fdes)   = [P.task.F_des 0];
up(I.RQ)     = P.cost.RQ;
up(I.RS)     = P.cost.RS;
if V.risk
    up(I.qFr) = P.cost.qF;
    up(I.qpr) = P.cost.qp;
end
up(I.sigw)   = P.cost.sigma_w;
up(I.QS)     = P.cost.QS;
up(I.sref)   = log(P.k_max) * [1 1];
up(I.smin)   = log(P.k_min) * [1 1];
up(I.smax)   = log(P.k_max) * [1 1];
up(I.Vscale) = P.cost.Vscale;

% only VI-MPIC moves the stiffness; the other variants get ds/dt = 0 and the
% stiffness rate drops out of the model and its gradient
vark = strcmp(V.stiff, 'vi');
up(I.vark) = double(vark);

grampc = CmexFiles.grampc_init_Cmex(up);

if vark
    sdot = [P.sdot_min P.sdot_max];
    us = P.mpc.uScale_s;
else
    sdot = [0 0];
    us = 1;
end
user.param.x0   = x0(:)';
user.param.xdes = zeros(1, 8);
user.param.u0   = zeros(1, 4);
user.param.udes = zeros(1, 4);
user.param.umax = [ P.mpc.uq_max  P.mpc.uq_max sdot(2) sdot(2)];
user.param.umin = [-P.mpc.uq_max -P.mpc.uq_max sdot(1) sdot(1)];
user.param.Thor = P.mpc.Thor;
user.param.dt   = P.mpc.dt;
user.param.t0   = 0;

user.opt.Nhor              = P.mpc.Nhor;
user.opt.MaxGradIter       = P.mpc.MaxGradIter;
user.opt.MaxMultIter       = 1;
user.opt.ConstraintsAbsTol = 1e-3 * ones(1, 4);
% the gradient step scales with uScale^2: without it ds/dt [1/s] barely
% moves next to dqd/dt [rad/s] within MaxGradIter iterations
user.opt.ScaleProblem = 'on';
user.opt.xScale  = ones(1, 8);
user.opt.xOffset = zeros(1, 8);
user.opt.uScale  = [1 1 us us];
user.opt.uOffset = zeros(1, 4);

grampc = grampc_update_struct_grampc(grampc, user);
end
