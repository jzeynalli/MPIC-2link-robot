function [xdot, vars] = printTwoLinkXdot(p)
% symbolic variables (states , inputs)
syms q1 q2 dq1 dq2 tau1 tau2 real
vars = [q1; q2; dq1; dq2; tau1; tau2];
x   = [q1; q2; dq1; dq2];
tau = [tau1; tau2];

% parameters as plain doubles taken from p
m1 = p.m1;  m2 = p.m2;
L1 = p.length(1);
r1 = p.r1;  r2 = p.r2;
I1 = p.I1;  I2 = p.I2;
g  = p.g;

c2 = cos(q2); s2 = sin(q2);

% build dynamics symbolically (same as your numeric code)
M11 = I1 + I2 + m1*r1^2 + m2*(L1^2 + r2^2 + 2*L1*r2*c2);
M12 = I2 + m2*(r2^2 + L1*r2*c2);
M22 = I2 + m2*r2^2;
M = [M11, M12;
     M12, M22];

h = m2*L1*r2*s2;
C = [ -h*dq2,      -h*(dq1 + dq2);
       h*dq1,       0            ];

G1 = (m1*r1 + m2*L1)*g*cos(q1) + m2*r2*g*cos(q1 + q2);
G2 = m2*r2*g*cos(q1 + q2);
G  = [G1; G2]

ddq  = M \ (tau - C*[dq1; dq2] - G);
xdot = [dq1; dq2; ddq(1); ddq(2)];

% simplify & print 
xdot = simplify(xdot); 

% PRINTING (string replacement), not the symbolic math.
old = [q1  q2  dq1  dq2  tau1  tau2];
new = ["x[0]","x[1]","x[2]","x[3]","u[0]","u[1]"];

for i = 1:numel(xdot)
    s = string(xdot(i));% convert symbolic to string for printing
    s = replace(s, string(old), new);  % replace variable tokens (print-only)
    fprintf('xdot(%d) = %s;\n\n', i, s);
    if i < 3
        g = string(G(i));
        g = replace(g,string(old), new);
        fprintf('G(%d) = %s; \n\n', i, g);
    end
end
end
