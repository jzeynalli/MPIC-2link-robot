function [vec,grampc,figNr] = startMPC(figNr,md,compile, varargin)
% This function runs the BallOnPlate example. It compiles the c-Code,
% initializes the grampc struct, runs the simulation and plots the results.
%
% Input arguments are:
% 1) figNr - number of the first plot
% 2) compile - flag whether to compile the whole toolbox or/and the problem function: 
%              1: only the problem function is compiled
%              2: the whole toolbox and the problem function is compiled
%              else or empty input: nothing is compiled
% 3 - end) - flags for the compilation (e.g. 'debug' or 'verbose') see 
%            make.m in the matlab folder for more details
%
%
% This file is part of GRAMPC - (https://github.com/grampc/grampc)
%
% GRAMPC -- A software framework for embedded nonlinear model predictive
% control using a gradient-based augmented Lagrangian approach
%
% Copyright 2014-2025 by Knut Graichen, Andreas Voelz, Thore Wietzke,
% Tobias Englert (<v2.3), Felix Mesmer (<v2.3), Soenke Rhein (<v2.3),
% Bartosz Kaepernick (<v2.0), Tilman Utz (<v2.0).
% All rights reserved.
%
% GRAMPC is distributed under the BSD-3-Clause license, see LICENSE.txt
%

%% Check input arguments
if nargin < 1 || isempty(figNr)
    figNr = 1;
end

if nargin < 2 || isempty(md)
    md = 1;
end

if nargin < 3 || isempty(compile)
    compile = 0;
end



%% Parameters
% path to grampc root
grampc_root_path = 'external/grampc/';
addpath([grampc_root_path 'matlab/mfiles']);
% name of problem function
probfct = 'probfct_2LINKROBOT_cascaded.c';

% plot predicted trajectories
PLOT_PRED = 1;
% plot solution trajectories
PLOT_TRAJ = 1;
% plot optimization statistics
PLOT_STAT = 1;
% update plots after N steps
PLOT_STEPS = 100;
% pause after each plot
PLOT_PAUSE = 0;

% Options for the reference simulation
odeopt =  [];%odeset('RelTol',1e-6,'AbsTol',1e-8);


%% Compilation
% compile toolbox
if compile > 1 || ~exist([grampc_root_path 'matlab/bin'], 'dir')
    grampc_make_toolbox(grampc_root_path, varargin{:});
end
% compile problem
if compile > 0 || ~exist('+CmexFiles', 'dir')
    grampc_make_probfct(grampc_root_path, probfct, varargin{:});
end


%% Initialization
% init GRAMPC and print options and parameters
[grampc,Tsim] = initData(md);
CmexFiles.grampc_printopt_Cmex(grampc);
CmexFiles.grampc_printparam_Cmex(grampc);

CmexFiles.grampc_check_gradients_Cmex(grampc, 1e-5, 1e-8);

% init solution structure
vec = grampc_init_struct_sol(grampc, Tsim);

% init plots and store figure handles
if figNr > 0 && PLOT_PRED
    phpP = grampc_init_plot_pred(grampc,figNr);
    figNr = figNr+1;
end
if figNr > 0 && PLOT_TRAJ
    phpT = grampc_init_plot_sim(vec,figNr);
    figNr = figNr+1;
end
if figNr > 0 && PLOT_STAT
    phpS = grampc_init_plot_stat(vec,grampc,figNr);
    figNr = figNr+1;
end
%% MPC loop
p = struct();
p.length = [0.20; 0.20];   % [L1; L2] link lengths [m]
p.m1 = 0.5;                % [kg]
p.m2 = 0.5;                % [kg]
p.r1 = 0.5*p.length(1);    % COM offset along link1 [m]
p.r2 = 0.5*p.length(2);    % COM offset along link2 [m]
p.I1 = (1/12)*p.m1*p.length(1)^2;  % slender rod inertia about COM [kg*m^2]
p.I2 = (1/12)*p.m2*p.length(2)^2;  % [kg*m^2]
p.g  = 9.18;               % [m/s^2]

% Object parameters
p.wall_x    = 0.3;     % wall x-position [m]
p.wall_ymin = -0.40;    % wall lower y limit [m]
p.wall_ymax =  0.40;    % wall upper y limit [m]
p.wall_Ke   = diag([800, 0]);   % environment stiffness [N/m], same as K_bar computation

p.circle_center = [0.2; -0.2];
p.circle_radius = 0.05;
p.circle_Ke = diag([800,800]);


flag_once = 0;

if md ==1

    i = 1;
while 1

    %--- Set current time and state (including updated forces) ---%
    grampc = CmexFiles.grampc_setparam_Cmex(grampc, 't0', vec.t(i));
    grampc = CmexFiles.grampc_setparam_Cmex(grampc, 'x0', vec.x(:,i));

    %--- Run MPC ---%
    [grampc, vec.CPUtime(i)] = CmexFiles.grampc_run_Cmex(grampc);
    vec = grampc_update_struct_sol(grampc, vec, i);

    printed = CmexFiles.grampc_printstatus_Cmex(grampc.sol.status, 'Error');
    if printed
        fprintf('at simulation time %f.\n --------\n', vec.t(i));
    end

    if i+1 > length(vec.t), break; end

    %--- 5. Simulate one step with ODE45 ---%
    [~, xtemp] = ode45(@CmexFiles.grampc_ffct_Cmex, ...
                        vec.t(i) + [0, double(grampc.param.dt)], ...
                        vec.x(:,i), odeopt, ...
                        vec.u(:,i), vec.p(:,i), grampc.param, grampc.userparam);
    vec.x(:, i+1) = xtemp(end, :)';
        
    % evaluate time-dependent constraints 
    % to obtain h(x,u,p) instead of max(0,h(x,u,p))
    %vec.constr(:,i) = CmexFiles.grampc_ghfct_Cmex(vec.t(i), vec.x(:,i), vec.u(:,i), vec.p(:,i), grampc.param, grampc.userparam);
    
    % update iteration counter
    i = i + 1;
    
    % plot data
    if figNr > 0 && (mod(i,PLOT_STEPS) == 0 || i == length(vec.t))
        if PLOT_PRED
            grampc_update_plot_pred(grampc,phpP);
        end
        if PLOT_TRAJ
            grampc_update_plot_sim(vec,phpT);
        end
        if PLOT_STAT
            grampc_update_plot_stat(vec,grampc,phpS);
        end
        drawnow
        if PLOT_PAUSE
            pause;
        end
    end
end


elseif md == 2

    i = 1;
while 1

    %Estimate contact force from current state 
    q_cur  = vec.x(1:2, i);
    qd_cur = vec.x(3:4, i);
    x_ee   = forward_kinematics(q_cur, p);


    [F_contact, l] = compute_force(x_ee, p);

    if all(F_contact==0)
        grampc.userparam(23) = 0;
        grampc.userparam(24) = 0;
        grampc.userparam(25) = 0;
        grampc.userparam(26) = 0;
        vec.x(5, i) = 0;
        vec.x(6, i) = 0;

    else
        
        vec.x(5, i) =  F_contact(1);
        vec.x(6, i) =  F_contact(2);
        
        grampc = update_stiffness_bar(grampc, q_cur,l);

    end


    %--- Set current time and state 
    grampc = CmexFiles.grampc_setparam_Cmex(grampc, 't0', vec.t(i));
    grampc = CmexFiles.grampc_setparam_Cmex(grampc, 'x0', vec.x(:,i));


    %---  Run MPC ---%
    [grampc, vec.CPUtime(i)] = CmexFiles.grampc_run_Cmex(grampc);
    vec = grampc_update_struct_sol(grampc, vec, i);

    printed = CmexFiles.grampc_printstatus_Cmex(grampc.sol.status, 'Error');
    if printed
        fprintf('at simulation time %f.\n --------\n', vec.t(i));
    end

    if i+1 > length(vec.t), break; end

    %--- Simulate one step with ODE45 
    [~, xtemp] = ode45(@CmexFiles.grampc_ffct_Cmex, ...
                        vec.t(i) + [0, double(grampc.param.dt)], ...
                        vec.x(:,i), odeopt, ...
                        vec.u(:,i), vec.p(:,i), grampc.param, grampc.userparam);
    vec.x(:, i+1) = xtemp(end, :)';
        
    
    % update iteration counter
    i = i + 1;
    
    % plot data
    if figNr > 0 && (mod(i, PLOT_STEPS) == 0 || i == length(vec.t))
        if PLOT_PRED, grampc_update_plot_pred(grampc, phpP); end
        if PLOT_TRAJ, grampc_update_plot_sim(vec, phpT);     end
        if PLOT_STAT, grampc_update_plot_stat(vec, grampc, phpS); end
        drawnow;
        if PLOT_PAUSE, pause; end
    end
end


end



%% Animate closed-loop trajectory (after simulation)
t = vec.t;
X = vec.x.';
F_mpc_x  = X(:,5);
F_mpc_y  = X(:,6);
F_calc_x = zeros(size(t));
F_calc_y = zeros(size(t));

for k = 1:length(t)
    x_ee_k        = forward_kinematics(X(k,1:2)', p);
    [Fc,~]             = compute_force(x_ee_k, p);
    F_calc_x(k)   = Fc(1);
    F_calc_y(k)   = Fc(2);
end

% Animation
anim             = struct();
anim.fps         = 120;
anim.showTrace   = true;
anim.traceMode   = 'persistent';
anim.traceLength = 300;
anim.axisPadding = 0.10;
anim.title       = '2-Link Planar Arm (GRAMPC + Wall Contact)';
anim.F_mpc_x     = F_mpc_x;
anim.F_mpc_y     = F_mpc_y;
anim.F_calc_x    = F_calc_x;
anim.F_calc_y    = F_calc_y;

% Pass wall geometry for animation
anim.wall_x      = p.wall_x;
anim.wall_ymin   = p.wall_ymin;
anim.wall_ymax   = p.wall_ymax;
%pass circle geometry
 anim.circle_center = p.circle_center;
 anim.circle_radius = p.circle_radius;

 % Record to robot_sim.mp4
anim.record      = true;
anim.outputFile  = 'robot_sim';   % extension added automatically
anim.videoFormat = 'MPEG-4';      % → .mp4  |  'Motion JPEG AVI' → .avi
anim.videoQuality = 95;           % 0–100

animateTwoLinkPlanar(t, X, p, anim);

end

function [F, l] = compute_force(x_ee, p)
    F = [0; 0];
    l=p;

    %for wall
    depth = x_ee(1) - p.wall_x;
    if depth >= 0

         % Reaction force acts in -x direction (wall pushes robot back)
         Fx = p.wall_Ke(1,1) * depth;
         Fy = 0;   % no y-force component
         F_wall  = [Fx; Fy];
         F = F + F_wall;
    end
    %for circular object
    v = x_ee - p.circle_center;
    dist = norm(v);

    if dist < p.circle_radius && dist > 0.01
        
        depth_c = p.circle_radius - dist;
        n=v/dist; %normal vector pointing outward from circle
        depth_v = depth_c * n; %displacement as vector
        F_circle = p.circle_Ke * depth_v;
        F = F + F_circle;
        l.wall_Ke = p.circle_Ke;
    end


    
end

function grampc = update_stiffness_bar(grampc, q,p)
    % 1. Read Kp from userparam 
    Kp1 = grampc.userparam(19);
    Kp2 = grampc.userparam(20);
    Kp = diag([Kp1, Kp2]);

    % 2. Define Environment Stiffness Ke 
    Ke = p.wall_Ke;

    % 3. Calculate Jacobian (J)
    l1 = p.length(1);
    l2 = p.length(2); 
    
    s1  = sin(q(1)); c1  = cos(q(1));
    s12 = sin(q(1)+q(2)); c12 = cos(q(1)+q(2));
    
    J11 = -l1*s1 - l2*s12;
    J12 = -l2*s12;
    J21 =  l1*c1 + l2*c12;
    J22 =  l2*c12;
    
    J = [J11, J12; J21, J22];

    % 4. Calculate K_bar
    % Formula: K_bar = (I + Ke * (J * Kp^-1 * J'))^-1 * Ke
    
    % Cartesian Compliance: K_inv_cart = J * Kp^-1 * J'
    K_inv_cart = J * (eye(2)/Kp) * J';
    
    % Stiffness Bar
    I = eye(2);
    K_bar = (I + Ke * K_inv_cart) \ Ke;
    
    K_bar = Ke;
    % Mapping: [K11, K12; K21, K22] -> [23, 24; 25, 26]
    grampc.userparam(23) = K_bar(1,1);
    grampc.userparam(24) = K_bar(1,2);
    grampc.userparam(25) = K_bar(2,1);
    grampc.userparam(26) = K_bar(2,2);
end