function q = vi_ik(p, R)
%VI_IK Inverse kinematics of the 2-link arm, elbow solution with q2 > 0.

l1 = R.l(1); l2 = R.l(2);
c2 = (p(1)^2 + p(2)^2 - l1^2 - l2^2) / (2 * l1 * l2);
if abs(c2) > 1
    error('vi_ik: point (%g, %g) is out of reach', p(1), p(2));
end
s2 = sqrt(1 - c2^2);
q = [atan2(p(2), p(1)) - atan2(l2 * s2, l1 + l2 * c2);
     atan2(s2, c2)];
end
