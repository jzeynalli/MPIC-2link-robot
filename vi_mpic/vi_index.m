function I = vi_index()
%VI_INDEX 1-based positions in the GRAMPC userparam vector.
%   Mirror of the UP_* defines in probfct_VI_MPIC.c (C index + 1).

I.omega  = 1;       % PD bandwidth omega = Kp/Kd [1/s]
I.ke     = 2:3;     % environment stiffness used by the model (x, y) [N/m]
I.J0     = 4:7;     % frozen Jacobian J(q_k), row major
I.J0inv  = 8:11;    % its inverse, row major
I.phat   = 12;      % surface belief: mean position along +x [m]
I.sigma  = 13;      % surface belief: standard deviation [m]
I.ell    = 14;      % deepest contact-free depth [m], -1e3 if none
I.mk     = 15;      % depth of the desired frame at t_k [m]
I.qdk    = 16:17;   % qd_k, linearisation point of the depth
I.gmode  = 18;      % gate: 0 measured, 1 Phi(z), 2 conditioned
I.gmeas  = 19;      % measured gate for mode 0
I.psi    = 20;      % 1: keep the w.*sdot term of the force row
I.Qp     = 21:22;   % EE position weights (x, y)
I.pdes   = 23:24;   % EE target at the start of the horizon (x, y) [m]
I.vdes   = 25;      % target velocity along x [m/s]
I.tdelay = 26;      % horizon time at which the target starts to move [s]
I.pxmax  = 27;      % end of the approach ramp along x [m]
I.QF     = 28:29;   % force weights (x, y)
I.Fdes   = 30:31;   % desired force (x, y) [N]
I.RQ     = 32;      % weight on dqd/dt
I.RS     = 33;      % weight on ds/dt
I.qFr    = 34;      % q_F of the force-risk term
I.qpr    = 35;      % q_p of the disturbance-risk term
I.sigw   = 36;      % disturbance std sigma_w [N]
I.QS     = 37;      % weight on s - s_ref
I.sref   = 38:39;   % s_ref (x, y)
I.smin   = 40:41;   % lower log-stiffness bound (x, y)
I.smax   = 42:43;   % upper log-stiffness bound (x, y)
I.Vscale = 44;      % terminal cost factor
I.Qr     = 45:46;   % weights on the desired-frame position FK(qd) (x, y)
I.vark   = 47;      % 1: ds/dt is a decision variable (VI-MPIC)
I.Qv     = 48;      % weight on the normal reference velocity
I.vapp   = 49;      % desired normal reference velocity [m/s]
% sanding stroke along the wall (FC-MPIC comparison, fc_mpic/); zeros keep the first draft
I.ya     = 50;      % stroke amplitude [m]
I.yw     = 51;      % stroke angular frequency [rad/s]
I.yt0    = 52;      % horizon time at which the stroke starts [s]
I.ff     = 53;      % 1: input cost on dqd/dt - u*, u* the stroke feed-forward
I.len    = 53;
end
