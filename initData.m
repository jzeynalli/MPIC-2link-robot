function [grampc,Tsim,grampc_sdata] = initData(md)
% This function initializes a grampc struct in MATLAB and sets parameters 
% and options. In case of three output arguments the struct grampc_sdata for
% the use in Simulink is created as well. Define all options and parameters
% for the use of GRAMPC in MATLAB here.
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

%% Parameter definition
% Initial values and setpoints of the states

if nargin < 1 || isempty(md)
    md = 1;
end


%temp
p_robot = struct();
p_robot.length = [0.2; 0.2];   % set your link lengths here

xydes = [0.31; 0.0]; % rectangle

%xydes = [0.19; -0.16]; %circle
q_all = inverse_kinematics(xydes, p_robot);
qdes = q_all(:,1)

if md ==1
user.param.x0   = [pi/2, 0, pi/2, 0, 0.0, 0.0];
user.param.xdes = [qdes(1), qdes(2), qdes(1), qdes(2), 0.0, 0.0];
elseif md==2
user.param.x0   = [-0.6841,  1.3682, -0.6841,  1.3682, 0.0, 0.0]; %for rectangle
 %user.param.x0   = [-1.6007,  1.8016, -1.6007,  1.8016, 0.0, 0.0]; %for circle 
user.param.xdes = [qdes(1), qdes(2), qdes(1), qdes(2), 10.0, -10.0]; 

end
% Initial values, setpoints and limits of the inputs
user.param.u0   = [0.0, 0.0];
user.param.udes = [0.0, 0.0];
user.param.umax = [ 3.0,  3.0];
user.param.umin = [-3.0, -3.0];

% Time variables
user.param.Thor  = 0.5;         % Prediction horizon

user.param.dt    = 0.01;        % Sampling time
user.param.t0    = 0.0;         % time at the current sampling step

%% Option definition
% Basic algorithmic options
user.opt.Nhor        = 21;          % Number of steps for the system integration
user.opt.MaxGradIter = 10;           % Maximum number of 
% user.opt.ConvergenceCheck = 'on';

% Do augmented Lagrangian update every MPC step
% user.opt.AugLagUpdateGradientRelTol = 1e0;

% Constraints thresholds
%user.opt.ConstraintsAbsTol = 1e-3*[1 1 1 1];

% optional settings for a better performance
 % user.opt.LineSearchType = 'adaptive';
 % user.opt.LineSearchInit = 1e-3;
 % user.opt.LineSearchExpAutoFallback = 'off';
 %user.opt.PenaltyMin = 1000; % Comment line 75 (grampc = CmexFiles.grampc_estim_penmin_Cmex(grampc,1);) to use this option value

%% User parameter definition 
% weighting matrices
if md ==1

Q = [0.0, 0.0, 2.0, 2.0, 0.00, 0.00];
R = [0.01 0.01];
S = Q;
H = [0.0 0.0 0.0 0.0];
Kp = [1000 1000];
Kd = [600 600];
Ke = [0 0 0 0];
Qp = [10 10];
Sp = Qp;

elseif md ==2        

Q = [0.0, 0.0, 0.0, 0.0, 1, 1];
R = [0.01 0.01];
S = Q;
H = [0.0 0.0 0.0 0.0];
Kp = [1000 1000];
Kd = [600 600];
Ke = [800 0 0 0];
Qp = [10 10];
Sp = Qp;

end



% e.g. system parameters or weights for the cost function
userparam = [Q R S H Kp Kd Ke Qp Sp];

%% Grampc initialization
grampc = CmexFiles.grampc_init_Cmex(userparam);

%% Update grampc struct while ensuring correct data types
grampc = grampc_update_struct_grampc(grampc,user);

%% Estimate and set PenaltyMin (optional)
[grampc, ~] = CmexFiles.grampc_estim_penmin_Cmex(grampc,1);

%% Simulation time
if nargout>1
    Tsim = 3;
end

%% Simulink data
% Converting the grampc struct to a code generation compatible format for
% the use in simulink
if nargout>2
    grampc_sdata = grampc_generate_sdata(grampc);
end
