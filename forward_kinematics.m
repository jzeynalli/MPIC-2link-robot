% This file is part of the Robotics 1 lecture
% Copyright 2023 by Andreas Voelz
function x = forward_kinematics(q, p)
    c1 = cos(q(1)); c12 = cos(q(1) + q(2));
    s1 = sin(q(1)); s12 = sin(q(1) + q(2));
    x = [p.length(1) * c1 + p.length(2) * c12;
         p.length(1) * s1 + p.length(2) * s12];
end

