function [X_pred, P_pred] = propagate_navstate(X, P, input_k, dt, cfg)
%PROPAGATE_NAVSTATE Error-state prediction using body acceleration and yaw rate.

[theta, ~, v] = navstate_components(X);
R = rotation2d(theta);
a_body = input_k.accel_body(:);
omega = input_k.omega_meas;

rho = R' * (v * dt) + 0.5 * a_body * dt^2;
nu = a_body * dt;
delta = [omega * dt; rho; nu];
X_pred = X * navstate_exp(delta);

S2 = [0 -1; 1 0];
dacc_dtheta = R * S2 * a_body;

F = eye(5);
F(2:3, 1) = 0.5 * dacc_dtheta * dt^2;
F(2:3, 4:5) = eye(2) * dt;
F(4:5, 1) = dacc_dtheta * dt;

G = zeros(5, 3);
G(1, 3) = dt;
G(2:3, 1:2) = 0.5 * R * dt^2;
G(4:5, 1:2) = R * dt;

Q_input = diag([cfg.filters.accel_noise_sigma^2, ...
    cfg.filters.accel_noise_sigma^2, ...
    cfg.filters.gyro_noise_sigma^2]);

P_pred = F * P * F' + G * Q_input * G' + cfg.filters.process_noise_floor;
P_pred = 0.5 * (P_pred + P_pred');
end
