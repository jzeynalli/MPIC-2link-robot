function check_vi_gradients(tol, step)
%CHECK_VI_GRADIENTS Finite-difference check of the analytic adjoints of probfct_VI_MPIC.c.
%   Runs GRAMPC's gradient check (dfdx_vec, dfdu_vec, dldx, dldu, dVdx,
%   dhdx_vec, dhdu_vec) at random states near the wall for every gate mode,
%   with and without the stiffness-rate term. Mismatches are printed by
%   GRAMPC as "... exceeds supplied tolerance ...".

if nargin < 2
    step = 1e-8;
end
if nargin < 1
    tol = 1e-5;
end
here = fileparts(mfilename('fullpath'));
old = cd(here);
cleanup = onCleanup(@() cd(old));
addpath(fullfile(here, '..', 'external', 'grampc', 'matlab', 'mfiles'));

P = vi_params();
I = vi_index();
rng(1);
for gate = 0:2
    for psi = 0:1
        V = vi_variants(P, {'vimpic'});
        V.gate = gate;
        V.psi = psi;
        for trial = 1:3
            % a configuration near the wall and a reference just beyond it
            q  = vi_ik([0.29 + 0.01 * rand; 0.02 * randn], P.robot);
            qd = q + 0.02 * randn(2, 1);
            [~, J0] = vi_kin(q, P.robot);
            qdk = qd + 0.01 * randn(2, 1);
            pk = vi_kin(qdk, P.robot);
            x0 = [q; qd; 3 + 5 * rand; 0; log(500 + 5000 * rand(2, 1))];
            u0 = [0.3 * randn(2, 1); 5 * randn(2, 1)];

            grampc = vi_init_grampc(P, V, x0);
            up = grampc.userparam;
            Ji = inv(J0);
            up(I.J0)    = [J0(1, 1) J0(1, 2) J0(2, 1) J0(2, 2)];
            up(I.J0inv) = [Ji(1, 1) Ji(1, 2) Ji(2, 1) Ji(2, 2)];
            up(I.qdk)   = qdk';
            up(I.mk)    = pk(1);
            up(I.ell)   = pk(1) - 0.004;          % reference planned beyond ell
            up(I.gmeas) = 1;
            up(I.pdes)  = [0.30 0.01];
            up(I.qFr)   = P.cost.qF;
            up(I.qpr)   = P.cost.qp;
            up(I.QS)    = 0.1;
            up(I.Qr)    = [1e4 5e3];
            up(I.Qv)    = 100;
            up(I.vapp)  = 0.1;
            grampc.userparam = up;
            grampc = CmexFiles.grampc_setparam_Cmex(grampc, 'x0', x0);
            grampc = CmexFiles.grampc_setparam_Cmex(grampc, 'u0', u0);
            fprintf('--- gate %d, psi %d, trial %d\n', gate, psi, trial);
            CmexFiles.grampc_check_gradients_Cmex(grampc, tol, step);
        end
    end
end
fprintf('--- gradient check done (tolerance %g)\n', tol);
end
