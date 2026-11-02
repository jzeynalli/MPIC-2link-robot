function [p, J, M] = vi_kin(q, R)
%VI_KIN End-effector position, Jacobian and joint mass matrix of the 2-link arm.
%   R is the robot struct of vi_params (fields l, m, r, I). M is returned
%   only when requested.

l1 = R.l(1); l2 = R.l(2);
s1 = sin(q(1));  c1 = cos(q(1));
s12 = sin(q(1) + q(2)); c12 = cos(q(1) + q(2));

p = [l1 * c1 + l2 * c12;
     l1 * s1 + l2 * s12];
J = [-l1 * s1 - l2 * s12, -l2 * s12;
      l1 * c1 + l2 * c12,  l2 * c12];

if nargout > 2
    [a1, a2, a3] = vi_mass_consts(R);
    c2 = cos(q(2));
    M = [a1 + 2 * a2 * c2, a3 + a2 * c2;
         a3 + a2 * c2,     a3];
end
end

function [a1, a2, a3] = vi_mass_consts(R)
% M11 = a1 + 2 a2 cos(q2), M12 = a3 + a2 cos(q2), M22 = a3
a1 = R.I(1) + R.I(2) + R.m(1) * R.r(1)^2 + R.m(2) * (R.l(1)^2 + R.r(2)^2);
a2 = R.m(2) * R.l(1) * R.r(2);
a3 = R.I(2) + R.m(2) * R.r(2)^2;
end
