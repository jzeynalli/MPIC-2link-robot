function xdot = twoLinkPlanarODE(t, x, u_fun, p)
% full 2R planar dynamics with COM offsets and link inertias.
% x = [q1; q2; dq1; dq2], tau = [tau1; tau2]

q1  = x(1);
q2  = x(2);
dq1 = x(3);
dq2 = x(4);

tau = u_fun(t, x);           % must return 2x1

m1 = p.m1;
m2 = p.m2;
L1 = p.length(1);
r1 = p.r1;
r2 = p.r2;
I1 = p.I1;
I2 = p.I2;
g  = p.g;

c2 = cos(q2);
s2 = sin(q2);

% inertia matrix M(q)
M11 = I1 + I2 + m1*r1^2 + m2*(L1^2 + r2^2 + 2*L1*r2*c2);
M12 = I2 + m2*(r2^2 + L1*r2*c2);
M22 = I2 + m2*r2^2;
M = [M11, M12;
     M12, M22];

% coriolis/centrifugal matrix C(q,dq) such that C(q,dq)*dq is the vector term
h = m2*L1*r2*s2;
C = [ -h*dq2,      -h*(dq1 + dq2);
       h*dq1,       0            ];

% gravity vector G(q)
G1 = (m1*r1 + m2*L1)*g*cos(q1) + m2*r2*g*cos(q1 + q2);
G2 = m2*r2*g*cos(q1 + q2);
G  = [G1; G2];

ddq = M \ (tau - C*[dq1; dq2] - G); % LU/Cholesky

xdot = [dq1; dq2; ddq(1); ddq(2)];
end
