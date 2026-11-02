function I = fc_index()
%FC_INDEX 1-based positions in the GRAMPC userparam vector of probfct_FC_MPIC.c.
%   Mirror of the UP_* defines in probfct_FC_MPIC.c (C index + 1).

I.model  = 1;       % force model: 0 Gold et al., 1 FC-MPIC, 2 pure sliding
I.disc   = 2;       % 1: discrete-time map, 0: continuous-time ODE (model 0)
I.gate   = 3;       % measured contact gate (0 or 1)
I.W      = 4:7;     % W = Kd^-1 Kp, row major
I.G      = 8:11;    % G = Kd^-1 J0' B, row major
I.BJ     = 12:15;   % B' J0, row major (row 1 tangential, row 2 normal)
I.Kh     = 16:19;   % stick stiffness (tt, tn, nt, nn) [N/m]
I.kbn    = 20;      % normal series stiffness of Gold et al. [N/m]
I.mu     = 21;      % friction coefficient mu_k of the MPC step
I.epsf   = 22;      % eps_f of the smooth sign [N]
I.beta   = 23;      % softplus sharpness [1/N]
I.epsv   = 24;      % eps_v of the pure-sliding model [m/s]
I.J0     = 25:28;   % frozen Jacobian J(q_k), row major
I.J0inv  = 29:32;   % its inverse, row major
I.Qp     = 33:34;   % EE position weights (x, y)
I.pdes   = 35:36;   % EE target at the start of the stroke (x, y) [m]
I.ya     = 37;      % stroke amplitude [m]
I.yw     = 38;      % stroke angular frequency [rad/s]
I.yt0    = 39;      % horizon time at which the stroke starts [s]
I.ff     = 40;      % 1: input cost on u - u* (stroke feed-forward)
I.QF     = 41;      % normal force weight
I.Fdes   = 42;      % desired normal force [N]
I.RQ     = 43;      % input weight
I.Qv     = 44;      % weight on the normal reference velocity
I.vapp   = 45;      % desired normal reference velocity [m/s]
I.Vscale = 46;      % terminal cost factor
I.len    = 46;
end
