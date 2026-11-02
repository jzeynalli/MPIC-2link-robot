function check_fc_gradients(tol, step)
%CHECK_FC_GRADIENTS Finite-difference check of the analytic adjoints of probfct_FC_MPIC.c.
%   Runs GRAMPC's gradient check (dfdx_vec, dfdu_vec, dldx, dldu, dVdx) for
%   every force model, in discrete and (model 0) continuous time, with and
%   without contact, at states near the friction cone, inside it, on its
%   other side, and with almost no contact force (where the smooth sign of
%   the corrector is not saturated and the last term of eq. (A.5) matters).
%   Mismatches are printed by GRAMPC as "... exceeds supplied tolerance".
%   dVdT is not implemented (the horizon length is not optimised) and is
%   reported by the checker for every case.

if nargin < 2
    step = 1e-7;
end
if nargin < 1
    tol = 1e-5;
end
here = fileparts(mfilename('fullpath'));
old = cd(here);
cleanup = onCleanup(@() cd(old));

P = fc_params('nominal');
I = fc_index();
rng(1);
names = {'mpic', 'mpic_d', 'fc', 'slide'};
for n = 1:numel(names)
    V = fc_variants(P, names(n));
    for gate = 0:1
        for trial = 1:6
            q = vi_ik([0.302 + 0.002 * randn; -0.04 + 0.04 * rand], P.robot);
            [~, J0] = vi_kin(q, P.robot);
            KP = diag(P.ctrl.kq .* (0.8 + 0.4 * rand(2, 1)));
            fn = 8 + 4 * rand;
            switch trial
                case {1, 2}
                    ft = 0.5 * fn * (0.9 + 0.2 * rand);     % at the cone
                case 3
                    ft = 1;                                % inside
                case 4
                    ft = -0.48 * fn;                       % other side
                case 5
                    fn = 0.2 + 0.2 * rand;                 % almost no contact force
                    ft = 0.15;
                otherwise
                    fn = 0.2 + 0.2 * rand;
                    ft = -0.1;
            end
            Md = fc_prestep(P, V, J0, KP, KP / P.omega, [fn; ft], P.fc.mu);
            x0 = [q; q + 0.02 * randn(2, 1); ft; fn];
            u0 = 0.25 * randn(2, 1);
            [grampc, C] = fc_init_grampc(P, V, x0, fc_vi_params(P));
            up = grampc.userparam;
            up(I.gate)  = gate;
            up(I.W)     = rowMajor(Md.W);
            up(I.G)     = rowMajor(Md.G);
            up(I.BJ)    = rowMajor(Md.BJ);
            up(I.Kh)    = rowMajor(Md.Kh);
            up(I.kbn)   = Md.kbn;
            up(I.mu)    = Md.mu;
            up(I.J0)    = rowMajor(J0);
            up(I.J0inv) = rowMajor(inv(J0));
            up(I.yt0)   = -0.3 * rand;
            up(I.Qv)    = 100;
            grampc.userparam = up;
            grampc = C.setparam(grampc, 'x0', x0);
            grampc = C.setparam(grampc, 'u0', u0);
            fprintf('--- %s, gate %d, trial %d (ft %.2f N, fn %.2f N)\n', names{n}, gate, trial, ft, fn);
            C.check(grampc, tol, step);
        end
    end
end
fprintf('--- gradient check done (tolerance %g)\n', tol);
end

function r = rowMajor(A)
r = reshape(A.', 1, []);
end
