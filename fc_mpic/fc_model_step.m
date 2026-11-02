function x = fc_model_step(x, u, M)
%FC_MODEL_STEP One step of the discrete-time MPC model of probfct_FC_MPIC.c in contact.
%   x = fc_model_step(x, u, M) maps x = [q; qd; ft; fn] over one sampling
%   interval with the input u = dqd/dt and the model data M of fc_prestep
%   (M.model: 0 Gold et al., 1 FC-MPIC, 2 pure sliding). Mirror of ffct
%   with UP_DISC = 1 and UP_GATE = 1, for the open-loop checks.

dt = M.dt;
r = M.BJ * u;                                   % reference rate (r_t; r_n)
x(1:2) = x(1:2) + dt * (u + M.W * (x(3:4) - x(1:2)) - M.G * x(5:6));
x(3:4) = x(3:4) + dt * u;
switch M.model
    case 1
        % return map: elastic predictor and smooth plastic corrector, eq. (2.4)-(2.8)
        tr = x(5:6) + dt * M.Kh * r;
        x(5:6) = tr - M.Kh(:, 1) * fc_corrector(tr, M);
    otherwise
        fn = x(6) + dt * M.kbn * r(2);
        x(6) = fn;
        if M.model == 2
            x(5) = M.mu * max(fn, 0) * tanh(r(1) / M.epsv);
        end
end
end

function s = fc_corrector(tr, M)
% s = e sp(e ftr - mu fnr+) / (ktt - mu e knt), e = tanh(ftr / eps_f), eq. (2.8)
e = tanh(tr(1) / M.epsf);
bx = M.beta * (e * tr(1) - M.mu * max(tr(2), 0));
S = (max(bx, 0) + log1p(exp(-abs(bx)))) / M.beta;
s = e * S / (M.Kh(1, 1) - M.mu * e * M.Kh(2, 1));
end
