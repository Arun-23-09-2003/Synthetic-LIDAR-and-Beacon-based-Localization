function odom = simulate_prediction_inputs(cfg, truth)
%SIMULATE_PREDICTION_INPUTS Create noisy acceleration/yaw-rate inputs.

t = truth.t;
dt = cfg.motion.dt;
num_steps = numel(t);

a_world = zeros(2, num_steps);
a_world(1, :) = gradient(truth.v(1, :), dt);
a_world(2, :) = gradient(truth.v(2, :), dt);

accel_body = zeros(2, num_steps);
for k = 1:num_steps
    R = rotation2d(truth.theta(k));
    accel_body(:, k) = R' * a_world(:, k);
end

omega = gradient(unwrap(truth.theta), dt);

accel_bias = cfg.motion.accel_bias_sigma * randn(2, 1);
gyro_bias = cfg.motion.gyro_bias_sigma * randn(1, 1);

accel_noisy = accel_body + accel_bias + cfg.motion.accel_noise_sigma * randn(2, num_steps);
omega_noisy = omega + gyro_bias + cfg.motion.gyro_noise_sigma * randn(1, num_steps);

u_true.t = t;
u_true.accel_body = accel_body;
u_true.omega_meas = omega;

u_noisy.t = t;
u_noisy.accel_body = accel_noisy;
u_noisy.omega_meas = omega_noisy;

odom.t = t;
odom.accel_body = accel_noisy;
odom.omega_meas = omega_noisy;
odom.u_true = u_true;
odom.u_noisy = u_noisy;
odom.model = 'body_acceleration_and_yaw_rate';
odom.noise.accel_sigma = cfg.motion.accel_noise_sigma;
odom.noise.gyro_sigma = cfg.motion.gyro_noise_sigma;
odom.noise.accel_bias = accel_bias;
odom.noise.gyro_bias = gyro_bias;
end
