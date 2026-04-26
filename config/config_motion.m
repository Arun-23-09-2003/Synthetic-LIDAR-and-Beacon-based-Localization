function motion_cfg = config_motion()
%CONFIG_MOTION Motion and prediction input parameters.

motion_cfg.prediction_rate_hz = 50;
motion_cfg.dt = 1 / motion_cfg.prediction_rate_hz;
motion_cfg.default_duration_s = 60;
motion_cfg.accel_noise_sigma = 0.08;       % m/s^2
motion_cfg.gyro_noise_sigma = 0.012;       % rad/s
motion_cfg.accel_bias_sigma = 0.018;       % m/s^2, per scenario constant
motion_cfg.gyro_bias_sigma = 0.004;        % rad/s, per scenario constant
motion_cfg.initial_heading_noise_sigma = 0.0;
motion_cfg.initial_position_noise_sigma = 0.0;
motion_cfg.initial_velocity_noise_sigma = 0.0;
end
