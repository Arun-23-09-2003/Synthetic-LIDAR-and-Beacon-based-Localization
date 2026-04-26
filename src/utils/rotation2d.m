function R = rotation2d(theta)
%ROTATION2D 2D rotation matrix.

c = cos(theta);
s = sin(theta);
R = [c, -s; s, c];
end
