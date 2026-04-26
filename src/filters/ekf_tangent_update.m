function [X_upd, P_upd, dx, nis] = ekf_tangent_update(X, P, residual, H, R_meas, cfg)
%EKF_TANGENT_UPDATE Apply a tangent-space EKF update to the NavState.

S = H * P * H' + R_meas;
K = (P * H') / S;
dx = K * residual;

dx_norm = norm(dx);
if dx_norm > cfg.filters.max_correction_norm
    dx = dx * (cfg.filters.max_correction_norm / dx_norm);
end

[theta, ~, ~] = navstate_components(X);
R_nominal = rotation2d(theta);
delta_group = [dx(1); R_nominal' * dx(2:3); R_nominal' * dx(4:5)];
X_upd = X * navstate_exp(delta_group);
I = eye(size(P));
P_upd = (I - K * H) * P * (I - K * H)' + K * R_meas * K';
P_upd = 0.5 * (P_upd + P_upd');
nis = residual' * (S \ residual);
end
