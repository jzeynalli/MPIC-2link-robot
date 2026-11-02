% This file is part of the Robotics 1 lecture
% Copyright 2023 by Andreas Voelz
function J = jacobian(q, p)
%JACOBIAN Compute derivative of endeffector pose x for configuration q
    c1 = cos(q(1)); c12 = cos(q(1) + q(2));
    s1 = sin(q(1)); s12 = sin(q(1) + q(2));
    J = [-p.length(1) * s1 - p.length(2) * s12, ...
         -p.length(2) * s12;
          p.length(1) * c1 + p.length(2) * c12, ...
          p.length(2) * c12];
      
    % transform to task space
    if isfield(p, 'use_task_space') && p.use_task_space
        J = p.task_space_rotation' * J;
    end
end