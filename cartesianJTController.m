function tau = cartesianJTController(t, x, xd, xdot_d, Kp, Kd, Dq, tau_max, p)
% cartesian PD in task space + Jacobian transpose mapping to torques.

q  = x(1:2);
dq = x(3:4);

% current end-effector position and velocity
xe   = forward_kinematics(q, p);     % 2x1
J    = jacobian(q, p);               % 2x2
xdot = J * dq;                       % 2x1

% Task-space PD
e    = xd - xe;
edot = xdot_d - xdot;
F    = Kp*e + Kd*edot;              % interpreted as a virtual Cartesian force [N]

% map to joint torques (virtual work): tau = J^T F
tau = J.' * F - Dq*dq;

% saturate
if nargin >= 9 && ~isempty(tau_max)
    tau = min(max(tau, -tau_max), tau_max);
end
end
