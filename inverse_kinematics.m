% This file is part of the Robotics 1 lecture
% Copyright 2023 by Andreas Voelz
function q = inverse_kinematics(x, p)
%INVERSE_KINEMATICS Compute inverse kinematics solutions
% x:        vector of size p.nx with endeffector pose
% p:        parameter structure

    % link lengths
    a1 = p.length(1);
    a2 = p.length(2);
    
    % determine cos(q(2))
    c2 = (x(1)^2 + x(2)^2 - a1^2 - a2^2) / (2 * a1 * a2);
    
    if -1 < c2 && c2 < 1
        % two solutions
        q = zeros(2, 2);
        s2 = sqrt(1 - c2^2);
        % first solution
        q(1, 1) = atan2(x(2), x(1)) - atan2(a2 * s2, a1 + a2 * c2);
        q(2, 1) = atan2(s2, c2);
        % second solution
        q(1, 2) = atan2(x(2), x(1)) - atan2(-a2 * s2, a1 + a2 * c2);
        q(2, 2) = atan2(-s2, c2); 
    elseif c2 == 1
        % single solution
        q = zeros(2, 1);
        q(1) = atan2(x(2), x(1));
        q(2) = 0;
    elseif c2 == -1
        % infinitely many solutions (pick one)
        q = zeros(2, 1);
        q(1) = 0;
        q(2) = pi;
    else
        % no solution
        q = [];
    end
end
