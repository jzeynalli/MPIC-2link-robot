function [grampc, C] = fc_init_grampc(P, V, x0, Pv)
%FC_INIT_GRAMPC GRAMPC struct of controller variant V at the initial state x0.
%   [grampc, C] = fc_init_grampc(P, V, x0, Pv) initialises the problem of
%   the variant: probfct_FC_MPIC.c (package +CmexFC) or, for VI-MPIC,
%   probfct_VI_MPIC.c (package +CmexVI) with the VI parameters Pv of
%   fc_vi_params. C holds the MEX functions of that package (init,
%   setparam, setopt, run, ffct, check). The per-step entries of userparam
%   are filled by fc_simulate before every MPC step.

if strcmp(V.problem, 'vi')
    [grampc, C] = initVI(P, V, x0, Pv);
else
    [grampc, C] = initFC(P, V, x0);
end
end

function [grampc, C] = initFC(P, V, x0)
C = mexOf('CmexFC');
I = fc_index();
up = zeros(1, I.len);
up(I.model)  = V.model;
up(I.disc)   = V.disc;
up(I.gate)   = 1;
up(I.W)      = [1 0 0 1];
up(I.BJ)     = [0 1 1 0];
up(I.Kh)     = [1 0 0 1];
up(I.kbn)    = P.ke_model;
up(I.mu)     = P.fc.mu;
up(I.epsf)   = P.fc.eps_f;
up(I.beta)   = P.fc.beta;
up(I.epsv)   = P.fc.eps_v;
up(I.J0)     = [1 0 0 1];
up(I.J0inv)  = [1 0 0 1];
up(I.Qp)     = P.cost.Qp;
up(I.pdes)   = [P.wall.x P.task.y0];
up(I.ya)     = P.task.A;
up(I.yw)     = 2 * pi / P.task.T;
up(I.yt0)    = P.task.t0;
up(I.ff)     = P.cost.ff;
up(I.QF)     = P.cost.QF;
up(I.Fdes)   = P.task.F_des;
up(I.RQ)     = P.cost.RQ;
up(I.vapp)   = P.cost.v_app;
up(I.Vscale) = P.cost.Vscale;
grampc = C.init(up);

user.param.x0   = x0(:)';
user.param.xdes = zeros(1, 6);
user.param.u0   = zeros(1, 2);
user.param.udes = zeros(1, 2);
user.param.umax =  P.mpc.uq_max * [1 1];
user.param.umin = -P.mpc.uq_max * [1 1];
user.param.Thor = P.mpc.Thor;
user.param.dt   = P.mpc.dt;
user.param.t0   = 0;
if V.disc
    % GRAMPC 2.3 discrete-time systems need Thor = (Nhor - 1) dt
    user.opt.Nhor           = round(P.mpc.Thor / P.mpc.dt) + 1;
    user.opt.Integrator     = 'discrete';
    user.opt.IntegratorCost = 'discrete';
else
    user.opt.Nhor = P.mpc.Nhor_cont;
end
user.opt.MaxGradIter = P.mpc.MaxGradIter;
user.opt.MaxMultIter = 1;
grampc = update(grampc, user, C);
end

function [grampc, C] = initVI(P, V, x0, Pv)
% vi_init_grampc of the first draft, for the package +CmexVI and with the
% stroke of the sanding task
C = mexOf('CmexVI');
I = vi_index();
up = zeros(1, I.len);
up(I.omega)  = Pv.omega;
up(I.ke)     = [Pv.ke_model 0];
up(I.J0)     = [1 0 0 1];
up(I.J0inv)  = [1 0 0 1];
up(I.phat)   = Pv.belief.p_hat;
up(I.sigma)  = Pv.belief.sigma0;
up(I.ell)    = -1e3;
up(I.gmode)  = V.gate;
up(I.psi)    = V.psi;
up(I.Qp)     = Pv.cost.Qp;
up(I.pdes)   = Pv.task.pA';
up(I.pxmax)  = Pv.task.p_cap;
up(I.QF)     = Pv.cost.QF;
up(I.Fdes)   = [Pv.task.F_des 0];
up(I.RQ)     = Pv.cost.RQ;
up(I.RS)     = Pv.cost.RS;
if V.risk
    up(I.qFr) = Pv.cost.qF;
    up(I.qpr) = Pv.cost.qp;
end
up(I.sigw)   = Pv.cost.sigma_w;
up(I.QS)     = Pv.cost.QS;
up(I.sref)   = log(Pv.k_max) * [1 1];
up(I.smin)   = log(Pv.k_min) * [1 1];
up(I.smax)   = log(Pv.k_max) * [1 1];
up(I.Vscale) = Pv.cost.Vscale;
vark = strcmp(V.stiff, 'vi');
up(I.vark)   = double(vark);
up(I.ya)     = P.task.A;
up(I.yw)     = 2 * pi / P.task.T;
up(I.yt0)    = P.task.t0;
up(I.ff)     = P.cost.ff;
grampc = C.init(up);

if vark
    sdot = [Pv.sdot_min Pv.sdot_max];
    us = Pv.mpc.uScale_s;
else
    sdot = [0 0];
    us = 1;
end
user.param.x0   = x0(:)';
user.param.xdes = zeros(1, 8);
user.param.u0   = zeros(1, 4);
user.param.udes = zeros(1, 4);
user.param.umax = [ Pv.mpc.uq_max  Pv.mpc.uq_max sdot(2) sdot(2)];
user.param.umin = [-Pv.mpc.uq_max -Pv.mpc.uq_max sdot(1) sdot(1)];
user.param.Thor = Pv.mpc.Thor;
user.param.dt   = Pv.mpc.dt;
user.param.t0   = 0;

user.opt.Nhor              = Pv.mpc.Nhor;
user.opt.MaxGradIter       = Pv.mpc.MaxGradIter;
user.opt.MaxMultIter       = 1;
user.opt.ConstraintsAbsTol = 1e-3 * ones(1, 4);
user.opt.ScaleProblem = 'on';
user.opt.xScale  = ones(1, 8);
user.opt.xOffset = zeros(1, 8);
user.opt.uScale  = [1 1 us us];
user.opt.uOffset = zeros(1, 4);
grampc = update(grampc, user, C);
end

function C = mexOf(pkg)
C.init     = str2func([pkg '.grampc_init_Cmex']);
C.setparam = str2func([pkg '.grampc_setparam_Cmex']);
C.setopt   = str2func([pkg '.grampc_setopt_Cmex']);
C.run      = str2func([pkg '.grampc_run_Cmex']);
C.ffct     = str2func([pkg '.grampc_ffct_Cmex']);
C.check    = str2func([pkg '.grampc_check_gradients_Cmex']);
end

function grampc = update(grampc, user, C)
% grampc_update_struct_grampc for a package other than +CmexFiles
grampc.userparam = grampc_set_datatype(class(grampc.opt.LineSearchMax), grampc.userparam);
for f = fieldnames(user.param)'
    grampc = C.setparam(grampc, f{1}, double(user.param.(f{1})));
end
for f = fieldnames(user.opt)'
    val = user.opt.(f{1});
    if isnumeric(val)
        val = double(val);
    end
    grampc = C.setopt(grampc, f{1}, val);
end
end
