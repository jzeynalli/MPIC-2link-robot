clear; clc;

%% Symbols
syms x1 x2 x3 x4 x5 x6 real
syms u1 u2 real

% gains / parameters
syms kp1 kp2 kd1 kd2 kex kexy keyx key objx objy objr real

% weights 
syms vec1 vec2 vec3 vec4 vec5 vec6 real

%% Constants (fixed link lengths)
l1 = sym(0.2);
l2 = sym(0.2);

%% Trig terms
s1  = sin(x1);
c1  = cos(x1);
s12 = sin(x1 + x2);
c12 = cos(x1 + x2);

px = l1 * c1 + l2 * c12;
py = l1 * s1 + l2 * s12;

Fh = (objr)^2 - ((px-objx)^2 + (py-objy)^2);

%% J(q)
J11 = -l1*s1 - l2*s12;
J12 = -l2*s12;
J21 =  l1*c1 + l2*c12;
J22 =  l2*c12;

 % J(qd)
% Jd11 = -l1*sin(u1) - l2*sin(u1 + u2);
% Jd12 = -l2*sin(u1 + u2);
% Jd21 =  l1*cos(u1) + l2*cos(u1 + u2);
% Jd22 =  l2*cos(u1 + u2);

%% Kc = J * Kp_cart_inv * J'  
Kc11 = (J11^2)/kp1 + (J12^2)/kp2;
Kc12 = (J11*J21)/kp1 + (J12*J22)/kp2;
Kc21 = Kc12;
Kc22 = (J21^2)/kp1 + (J22^2)/kp2;

%% B = I + Ke*Kc, inv(B), Kbar = inv(B)*Ke
B11 = 1 + kex*Kc11;
B12 =     kex*Kc12;
B21 =     key*Kc21;
B22 = 1 + key*Kc22;

detB   = B11*B22 - B12*B21;
invB11 =  B22/detB;
invB12 = -B12/detB;
invB21 = -B21/detB;
invB22 =  B11/detB;

Kbar11 = invB11*kex;
Kbar12 = invB12*key;
Kbar21 = invB21*kex;
Kbar22 = invB22*key;

%% Define F = [f1..f6]'

f1 = u1 + (kp1/kd1)*(x3 - x1) - (J11*x5 + J21*x6)/kd1;
f2 = u2 + (kp2/kd2)*(x4 - x2) - (J12*x5 + J22*x6)/kd2;
f3 = u1;
f4 = u2;

% f5 = kex*(Jd11*u1 + Jd12*u2) + kexy*(Jd21*u1 + Jd22*u2);
% f6 = keyx*(Jd11*u1 + Jd12*u2) + key*(Jd21*u1 + Jd22*u2);

f5 = kex*(J11*u1 + J12*u2) + kexy*(J21*u1 + J22*u2);
f6 = keyx*(J11*u1 + J12*u2) + key*(J21*u1 + J22*u2);


F = [f1; f2; f3; f4; f5; f6];

%% Jacobians
x = [x1; x2; x3; x4; x5; x6];
u = [u1; u2];

Jx = jacobian(F, x);   % dF/dx  (6x6) 
Ju = jacobian(F, u);   % dF/du  (6x2) 

%% Multiply transposed Jacobians by vec
vec = [vec1; vec2; vec3; vec4; vec5; vec6];

gx = Jx.' * vec;   % (6x1) = (dF/dx)' * vec
gu = Ju.' * vec;   % (2x1) = (dF/du)' * vec

%  display 
disp('F');  disp(F);
%disp('Jx = dF/dx:');  disp(Jx);
%disp('Ju = dF/du:');  disp(Ju);
disp('gx = Jx''*vec:'); disp(gx);
disp('gu = Ju''*vec:'); disp(gu);

%% === Convert gx, gu to C-style strings

% 1) Convert each entry to string
gx_str = strings(6,1);
gu_str = strings(2,1);
for i = 1:6
    gx_str(i) = string(char(gx(i)));
end
for i = 1:2
    gu_str(i) = string(char(gu(i)));
end

% 2) Replace variable names -> x[i], u[i], vec[i] 
pat = ["(?<![A-Za-z0-9_])x1(?![A-Za-z0-9_])"
       "(?<![A-Za-z0-9_])x2(?![A-Za-z0-9_])"
       "(?<![A-Za-z0-9_])x3(?![A-Za-z0-9_])"
       "(?<![A-Za-z0-9_])x4(?![A-Za-z0-9_])"
       "(?<![A-Za-z0-9_])x5(?![A-Za-z0-9_])"
       "(?<![A-Za-z0-9_])x6(?![A-Za-z0-9_])"
       "(?<![A-Za-z0-9_])u1(?![A-Za-z0-9_])"
       "(?<![A-Za-z0-9_])u2(?![A-Za-z0-9_])"
       "(?<![A-Za-z0-9_])vec1(?![A-Za-z0-9_])"
       "(?<![A-Za-z0-9_])vec2(?![A-Za-z0-9_])"
       "(?<![A-Za-z0-9_])vec3(?![A-Za-z0-9_])"
       "(?<![A-Za-z0-9_])vec4(?![A-Za-z0-9_])"
       "(?<![A-Za-z0-9_])vec5(?![A-Za-z0-9_])"
       "(?<![A-Za-z0-9_])vec6(?![A-Za-z0-9_])"];

rep = ["x[0]","x[1]","x[2]","x[3]","x[4]","x[5]", ...
       "u[0]","u[1]", ...
       "vec[0]","vec[1]","vec[2]","vec[3]","vec[4]","vec[5]"];

gx_str = regexprep(gx_str, pat, rep); 
gu_str = regexprep(gu_str, pat, rep); 


fprintf('--- gx (6x1) ---\n');
for i = 1:6
    fprintf('gx[%d] = %s;\n', i-1, gx_str(i));
end

fprintf('--- gu (2x1) ---\n');
for i = 1:2
    fprintf('gu[%d] = %s;\n', i-1, gu_str(i));
end

%% integral cost integation
clear; clc;
syms x0 x1 x2 x3 x4 x5 real
syms u0 u1 real
syms xdes0 xdes1 xdes2 xdes3 xdes4 xdes5 real
syms udes0 udes1 real
syms p0 p1 p2 p3 p4 p5 p6 p7 p26 p27 real

% Forward kinematics (l1 = l2 = 0.2)
px_d = 0.2*cos(xdes0) + 0.2*cos(xdes0 + xdes1);
py_d = 0.2*sin(xdes0) + 0.2*sin(xdes0 + xdes1);
px   = 0.2*cos(x0)    + 0.2*cos(x0 + x1);
py   = 0.2*sin(x0)    + 0.2*sin(x0 + x1);

% Cost function (out[0])
L = (  p0  * (x0 - xdes0)^2 ...
     + p1  * (x1 - xdes1)^2 ...
     + p2  * (x2 - xdes2)^2 ...
     + p3  * (x3 - xdes3)^2 ...
     + p4  * (x4 - xdes4)^2 ...
     + p5  * (x5 - xdes5)^2 ...
     + p6  * (u0 - udes0)^2 ...
     + p7  * (u1 - udes1)^2 ...
     + p26 * (px - px_d)^2  ...
     + p27 * (py - py_d)^2  ) / 2.0;

% Symbolic gradients
x_vec = [x0; x1; x2; x3; x4; x5];
u_vec = [u0; u1];

gx = jacobian(L, x_vec).';   % 6x1 column vector
gu = jacobian(L, u_vec).';   % 2x1 column vector

gx_simplified = simplify(gx);
gu_simplified = simplify(gu);

fprintf('=== Gradient w.r.t. x (gx) ===\n');
for i = 1:6
    fprintf('gx[%d] = %s;\n', i-1, char(gx_simplified(i)));
end

fprintf('\n=== Gradient w.r.t. u (gu) ===\n');
for i = 1:2
    fprintf('gu[%d] = %s;\n', i-1, char(gu_simplified(i)));
end

%% --- Optional: C-style name substitution for copy-paste into GRAMPC ---
xC    = sym({'x[0]','x[1]','x[2]','x[3]','x[4]','x[5]'});
uC    = sym({'u[0]','u[1]'});
xdesC = sym({'xdes[0]','xdes[1]','xdes[2]','xdes[3]','xdes[4]','xdes[5]'});
udesC = sym({'udes[0]','udes[1]'});
pC    = sym({'pCost[0]','pCost[1]','pCost[2]','pCost[3]', ...
             'pCost[4]','pCost[5]','pCost[6]','pCost[7]', ...
             'pCost[26]','pCost[27]'});

old_syms = [x0 x1 x2 x3 x4 x5 u0 u1 ...
            xdes0 xdes1 xdes2 xdes3 xdes4 xdes5 udes0 udes1 ...
            p0 p1 p2 p3 p4 p5 p6 p7 p26 p27];
new_syms = [xC uC xdesC udesC pC];

gx_C = subs(gx_simplified, old_syms, new_syms);
gu_C = subs(gu_simplified, old_syms, new_syms);

fprintf('\n=== C-style output for GRAMPC ===\n');
fprintf('// dL/dx\n');
for i = 1:6
    fprintf('dLdx[%d] = %s;\n', i-1, char(gx_C(i)));
end
fprintf('// dL/du\n');
for i = 1:2
    fprintf('dLdu[%d] = %s;\n', i-1, char(gu_C(i)));
end




%% Symbolic Jacobian for xdot = f(x,u)
clear; clc;

syms q1 q2 dq1 dq2 tau1 tau2 real

% State and input vectors
x = [q1; q2; dq1; dq2];
u = [tau1; tau2];

% Dynamics
f1 = dq1;
f2 = dq2;

f3 = -(3*(2000*tau1 - 2000*tau2 + (1377*cos(q1 + 2*q2))/2 - (4131*cos(q1))/2 + 20*dq1^2*sin(q2) + 20*dq2^2*sin(q2) + 15*dq1^2*sin(2*q2) - 3000*tau2*cos(q2) + 40*dq1*dq2*sin(q2)))/(10*(9*cos(q2)^2 - 16));

f4 = (3*(2000*tau1 - 10000*tau2 - (4131*cos(q1 - q2))/2 + (1377*cos(q1 + 2*q2))/2 + (3213*cos(q1 + q2))/2 - (4131*cos(q1))/2 + 100*dq1^2*sin(q2) + 20*dq2^2*sin(q2) + 30*dq1^2*sin(2*q2) + 15*dq2^2*sin(2*q2) + 3000*tau1*cos(q2) - 6000*tau2*cos(q2) + 40*dq1*dq2*sin(q2) + 30*dq1*dq2*sin(2*q2)))/(10*((9*cos(2*q2))/2 - 23/2));

f = [f1; f2; f3; f4];

% -------- Linearization Jacobians --------
% A(x,u) = df/dx
A = simplify(jacobian(f, x));          % jacobian(f,v) gives partials of f w.r.t. 

% B(x,u) = df/du   (input Jacobian)
B = simplify(jacobian(f, u));          % same call but with u=[tau1;tau2] 

% Display
disp('A = df/dx = '); disp(A);
disp('B = df/du = '); disp(B);

% -------- Optional: generate fast numeric function handles --------
% Control variable order with 'Vars' when generating numeric evaluators 
A_fun = matlabFunction(A, 'Vars', {q1,q2,dq1,dq2,tau1,tau2});  % A_fun(q1,q2,dq1,dq2,tau1,tau2) 
B_fun = matlabFunction(B, 'Vars', {q1,q2,dq1,dq2,tau1,tau2});  % B_fun(q1,q2,dq1,dq2,tau1,tau2) 

% Example numeric evaluation:
% A0 = A_fun(0, 0, 0, 0, 0, 0);
% B0 = B_fun(0, 0, 0, 0, 0, 0);

%% --- C-side naming convention (symbols used only for printing) ---
syms x0 x1 x2 x3 u0 u1 v0 v1 v2 v3 real
xSym   = [x0; x1; x2; x3];   
uSym   = [u0; u1];           
vecSym = [v0; v1; v2; v3];

% (df/dx)^T * vec  and  (df/du)^T * vec
ATv = simplify(A.' * vecSym);
BTv = simplify(B.' * vecSym);

% Replacement rules for printing 
old = [q1 q2 dq1 dq2 tau1 tau2 v0 v1 v2 v3];
new = ["x[0]" "x[1]" "x[2]" "x[3]" "u[0]" "u[1]" "vec[0]" "vec[1]" "vec[2]" "vec[3]"];

% Print as C assignment lines
disp('---- ATv = (df/dx)^T * vec ----');
for i = 1:numel(ATv)
    s = string(ATv(i));
    s = replace(s, string(old), new);
    fprintf('out[%d] = %s;\n', i-1, s);
end

disp('---- BTv = (df/du)^T * vec ----');
for i = 1:numel(BTv)
    s = string(BTv(i));
    s = replace(s, string(old), new);
    fprintf('out[%d] = %s;\n', i-1, s);
end

%% 2-Link planar prototype (main script)
clear; clc; close all;

% Parameters p
p = struct();
p.length = [0.20; 0.20];   % [L1; L2] link lengths [m]
p.m1 = 0.5;                % [kg]
p.m2 = 0.5;                % [kg]
p.r1 = 0.5*p.length(1);    % COM offset along link1 [m]
p.r2 = 0.5*p.length(2);    % COM offset along link2 [m]
p.I1 = (1/12)*p.m1*p.length(1)^2;  % slender rod inertia about COM [kg*m^2]
p.I2 = (1/12)*p.m2*p.length(2)^2;  % [kg*m^2]
p.g  = 9.18;               % [m/s^2]

[xdot_sym, vars] = printTwoLinkXdot(p);

%% numeric reference ODE 
u0 = [0.3; -0.2];
u_fun = @(t,x) u0;

t0 = 0; 
x0 = [0.2; -0.4; 0.7; -0.1];
xdot_num = twoLinkPlanarODE(t0, x0, u_fun, p)
xdot_sym_eval = double(subs(xdot_sym, vars, [x0; u0]))


%% Initial state x0 = [q1; q2; dq1; dq2]
q1_0  = deg2rad(30);
q2_0  = deg2rad(-60);
dq1_0 = 0.0;
dq2_0 = 0.0;
x0 = [q1_0; q2_0; dq1_0; dq2_0];

%% Task-space target (end-effector)
xd = [-0.2; -0.2];          % desired [x;y] in meters (must be within reach)
xdot_d = [0; 0];            % desired EE velocity
x0=[xd,xdot_d];
% cartesian PD gains
Kp = diag([80, 80]);        % [N/m]
Kd = diag([18, 18]);        % [N/(m/s)]

% joint damping (optional-helps)
Dq = diag([0.3, 0.3]);      % [N*m/(rad/s)]

% torque saturation (optional)
tau_max = [3; 3];           % [N*m]

% controller: tau = J(q)' * (Kp*e + Kd*edot) - Dq*dq 
u_fun = @(t, x) cartesianJTController(t, x, xd, xdot_d, Kp, Kd, Dq, tau_max, p);

%% Simulation settings
t0 = 0.0;
tf = 100.0;

odeopts = odeset( ...
    'RelTol', 1e-7, ...
    'AbsTol', 1e-9);

%% Simulate
odefun = @(t, x) twoLinkPlanarODE(t, x, u_fun, p);
[t, X] = ode45(odefun, [t0 tf], x0, odeopts);

%% Animate
anim = struct();
anim.fps = 120;               % target playback FPS
anim.showTrace = true;       % show end-effector trace
anim.traceMode = 'persistent';  % keeps full trace
anim.traceLength = 300;      % number of points to keep in trace
anim.axisPadding = 0.10;     % [m] extra padding
anim.title = '2-Link Planar Arm';

animateTwoLinkPlanar(t, X, p, anim);
