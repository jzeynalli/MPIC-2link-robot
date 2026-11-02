function P = vi_params()
%VI_PARAMS Default parameters of the VI-MPIC study: plant, belief, task, MPC and cost.
%   All controller variants and the benchmark start from these values.

% robot (truth plant): horizontal plane, Coriolis terms compensated by the
% low-level torque law, notes eq. (38)
P.robot.l = [0.2; 0.2];                     % link lengths [m], LINK1/LINK2 in the C file
P.robot.m = [0.5; 0.5];                     % link masses [kg]
P.robot.r = 0.5 * P.robot.l;                % centre of mass along each link [m]
P.robot.I = P.robot.m .* P.robot.l.^2 / 12; % link inertia about the centre of mass [kg m^2]
P.robot.h = 2e-4;                           % plant integration step [s]

% low-level joint PD: Kp = J0' diag(exp(s)) J0, Kd = Kp / omega
P.omega    = 10;                            % PD bandwidth [1/s]
P.k_min    = 300;                           % stiffness floor [N/m], >= 4 omega^2 m_eff (notes eq. 15)
P.k_max    = 1e4;                           % stiffness ceiling [N/m]
P.sdot_max = 0.95 * 1.6 * P.omega;          % stiffening rate bound, analysis Prop. 3.4 [1/s]
P.sdot_min = -40;                           % softening is not restricted (numerical bound) [1/s]

% environment: wall with normal +x at an uncertain position
P.wall.ke  = 5000;                          % true stiffness [N/m]
P.wall.ce  = 10;                            % true damping [Ns/m]
P.ke_model = 5000;                          % stiffness used by the MPC [N/m]

% surface belief and its update, analysis Table 5
P.belief.p_hat      = 0.30;                 % prior mean of the wall position [m]
P.belief.sigma0     = 0.01;                 % prior standard deviation [m]
P.belief.sigma_meas = 1e-3;                 % standard deviation once the wall is located [m]
P.belief.sigma_lost = 5e-3;                 % inflated standard deviation after contact loss [m]
P.F_contact         = 0.5;                  % contact detection threshold [N]

% task: hold pA under a disturbance, then guarded approach and press with F_des
P.task.pA           = [0.22; 0.0];          % free-motion target [m]
P.task.dist_F       = [5; 0];               % disturbance force at the EE [N]
P.task.dist_window  = [0.2 0.7];            % disturbance active [s]
P.task.track_window = [0.2 1.0];            % window of the tracking metric [s]
P.task.t_app        = 1.0;                  % start of the approach [s]
P.task.v_app        = 0.1;                  % speed of the approach target [m/s]
P.task.p_cap        = P.belief.p_hat + 3 * P.belief.sigma0;  % end of the approach ramp [m]
P.task.F_des        = 5;                    % desired normal force [N]
P.task.T_sim        = 3.5;                  % simulated time [s]
P.task.offsets      = P.belief.sigma0 * (-2:0.5:2);          % true minus believed wall position [m]

% MPC (GRAMPC)
P.mpc.dt          = 0.01;                   % sampling time [s]
P.mpc.Thor        = 0.5;                    % prediction horizon [s]
P.mpc.Nhor        = 26;                     % discretisation points
P.mpc.MaxGradIter = 10;                     % gradient iterations per step
P.mpc.uq_max      = 3;                      % bound on dqd/dt [rad/s]
P.mpc.uScale_s    = 20;                     % input scaling of ds/dt for the gradient method

% cost: notes eq. (9) and the stiffness cost eq. (30)
P.cost.Qp      = [2e4 2e4];                 % EE position weights [1/m^2]
P.cost.QF      = [0.5 0];                   % force weights [1/N^2]
P.cost.RQ      = 0.01;                      % weight on dqd/dt
P.cost.Qv      = 1e4;                       % weight on the approach velocity [s^2/m^2]
P.cost.RS      = 1e-3;                      % weight on ds/dt
P.cost.Vscale  = 1;                         % terminal cost factor
P.cost.qF      = 0.5;                       % force-risk weight (= QF)
P.cost.qp      = 2e4;                       % disturbance-risk weight (= Qp)
P.cost.sigma_w = 5;                         % disturbance level the stiffness should reject [N]
P.cost.QS      = 0;                         % pull towards s_ref (off)
end
