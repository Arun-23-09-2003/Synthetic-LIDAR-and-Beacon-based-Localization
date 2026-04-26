function dX = navstate_exp(delta)
%NAVSTATE_EXP Exponential map for the 2D NavState tangent coordinates.
%
% delta = [dtheta; dpx; dpy; dvx; dvy].

phi = delta(1);
rho = delta(2:3);
nu = delta(4:5);

A = 1 - (phi^2) / 6;
B = phi / 2 - (phi^3) / 24;
J = [A, -B; B, A];

R_delta = rotation2d(phi);
p_delta = J * rho(:);
v_delta = J * nu(:);

dX = eye(4);
dX(1:2, 1:2) = R_delta;
dX(1:2, 3) = p_delta;
dX(1:2, 4) = v_delta;
end
