function P = fc_params(fcase)
%FC_PARAMS Default parameters of the FC-MPIC study: plant, friction, low-level controller, task, MPC and cost.
%   P = fc_params() is the nominal friction case, P = fc_params('abrasive')
%   the abrasive one (the two cases of the proposal, Sec. 3). All
%   controller variants and the benchmark start from these values.

if nargin < 1
    fcase = 'nominal';
end

% robot: the arm of the VI-MPIC study (vi_params), horizontal plane,
% Coriolis terms compensated by the low-level torque law
P.robot.l = [0.2; 0.2];                     % link lengths [m], LINK1/LINK2 in the C files
P.robot.m = [0.5; 0.5];                     % link masses [kg]
P.robot.r = 0.5 * P.robot.l;                % centre of mass along each link [m]
P.robot.I = P.robot.m .* P.robot.l.^2 / 12; % link inertia about the centre of mass [kg m^2]
P.robot.h = 2e-4;                           % plant integration step [s]

% low-level joint PD, Kd = Kp / omega
%   'joint': Kp = diag(kq), the joint impedance of Gold et al. eq. (4) and of
%            the original MPIC of this repository. Its Cartesian compliance
%            couples the normal and tangential directions.
%   'cart' : Kp = J0' diag(kc) J0 with J0 frozen per MPC step, the
%            controller of VI-MPIC. Its compliance is diagonal along the wall.
P.ctrl.type = 'joint';
P.ctrl.kq   = [120; 120];                   % joint stiffness [Nm/rad]
P.ctrl.kc   = [1000; 1000];                 % Cartesian stiffness (x, y) [N/m]
P.omega     = 10;                           % PD bandwidth Kp/Kd [1/s], as in VI-MPIC

% wall with normal +x (as in VI-MPIC), now with friction
P.wall.x   = 0.30;                          % position [m]
P.wall.ke  = 5000;                          % normal stiffness [N/m]
P.wall.ce  = 10;                            % normal damping [Ns/m]
P.ke_model = 5000;                          % normal stiffness used by the MPC [N/m]
P.F_contact = 0.5;                          % contact detection threshold [N]

% LuGre friction with Stribeck effect (Canudas de Wit et al. 1995); bristle
% parameters of the prototype of the proposal (App. B)
P.fric.sigma0 = 3e4;                        % bristle stiffness [N/m]
P.fric.sigma1 = 30;                         % bristle damping [Ns/m]
P.fric.sigma2 = 2;                          % viscous friction [Ns/m]
P.fric.v_s    = 0.01;                       % Stribeck velocity [m/s]

% task: sanding strokes along the wall, py* = y0 + A (1 - cos(2 pi (t - t0) / T)),
% while the normal force is kept at F_des; the EE starts in static contact
P.task.F_des    = 10;                       % desired normal force [N]
P.task.y0       = -0.04;                    % start of the stroke [m]
P.task.A        = 0.04;                     % amplitude [m], stroke length 2 A
P.task.t0       = 0.5;                      % start of the strokes [s]
P.task.T_sim    = 6.5;                      % simulated time [s]
P.task.t_metric = 0.5;                      % metrics from here on [s]
P.task.rev_frac = 0.4;                      % reversal windows: |dpy*/dt| < rev_frac * peak

% friction cases of the proposal (Sec. 3): Coulomb / static level, stroke
% period, force-measurement noise
switch fcase
    case 'nominal'
        P.fric.mu_c = 0.5;  P.fric.mu_s = 0.6;  P.task.T = 2.0;  P.noise_F = 0.1;
    case 'abrasive'
        P.fric.mu_c = 0.8;  P.fric.mu_s = 0.9;  P.task.T = 1.2;  P.noise_F = 0.5;
    otherwise
        error('fc_params: unknown case %s', fcase);
end
P.case = fcase;

% FC-MPIC model, proposal Sec. 2.3-2.4
P.fc.mu       = P.fric.mu_c;                % model friction coefficient (the true Coulomb level)
P.fc.eps_f    = 0.3;                        % smooth sign [N], proposal value
P.fc.beta     = 4;                          % softplus sharpness [1/N], proposal value
P.fc.eps_v    = 0.005;                      % pure-sliding model: tanh(v / eps_v) [m/s]
P.fc.k_t      = inf;                        % tangential contact stiffness of the model (rigid)
P.fc.f_min    = 2;                          % measurement-consistent cone, eq. (2.9), for fn > f_min [N]
P.fc.v_min    = 0.02;                       % friction estimate, eq. (2.10): sliding speed [m/s]
P.fc.gain     = 0.05;                       % friction estimate: gain per MPC step
P.fc.painleve = 0.9;                        % mu_k <= painleve * ktt / |knt| (safeguard of Prop. 2.3)

% MPC (GRAMPC), the same for every controller
P.mpc.dt          = 0.01;                   % sampling time [s]
P.mpc.Thor        = 0.5;                    % prediction horizon [s]
P.mpc.Nhor_cont   = 26;                     % grid points, continuous-time models (as VI-MPIC)
P.mpc.MaxGradIter = 10;                     % gradient iterations per step
P.mpc.uq_max      = 3;                      % bound on dqd/dt [rad/s]

% cost: the terms of the VI-MPIC study (notes eq. 9) with the feed-forward of
% the stroke; in contact the normal position weight is replaced by the force
P.cost.Qp     = [2e4 2e4];                  % EE position weights [1/m^2]
P.cost.QF     = 0.5;                        % normal force weight [1/N^2]
P.cost.RQ     = 0.01;                       % weight on dqd/dt - u*
P.cost.Qv     = 1e4;                        % guarded approach after a loss of contact [s^2/m^2]
P.cost.v_app  = 0.1;                        % approach velocity of the reference [m/s]
P.cost.Vscale = 1;                          % terminal cost factor
P.cost.ff     = 1;                          % velocity feed-forward u* = J0^-1 (0, dpy*/dt)

% VI-MPIC (first draft) keeps its own settings from vi_params; only the
% shared task, plant and MPC values are overwritten (see fc_simulate)
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'vi_mpic'));
addpath(fullfile(here, '..', 'external', 'grampc', 'matlab', 'mfiles'));
P.vi = vi_params();
end
