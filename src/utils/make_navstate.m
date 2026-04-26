function X = make_navstate(theta, p, v)
%MAKE_NAVSTATE Build a 4x4 2D NavState group matrix.

X = eye(4);
X(1:2, 1:2) = rotation2d(theta);
X(1:2, 3) = p(:);
X(1:2, 4) = v(:);
end
