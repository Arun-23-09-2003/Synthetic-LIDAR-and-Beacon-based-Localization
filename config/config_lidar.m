function lidar_cfg = config_lidar()
%CONFIG_LIDAR LiDAR scan and pose-measurement parameters.

lidar_cfg.fov_deg = 270;
lidar_cfg.num_beams = 181;
lidar_cfg.max_range = 12.0;
lidar_cfg.range_sigma = 0.035;
lidar_cfg.scan_rate_hz = 5;
lidar_cfg.pose_update_rate_hz = 2.5;
lidar_cfg.min_valid_returns = 25;
lidar_cfg.quality_model_enabled = true;
lidar_cfg.pose_sigma_xy_base = 0.08;
lidar_cfg.pose_sigma_xy_gain = 0.55;
lidar_cfg.pose_sigma_theta_base = deg2rad(1.2);
lidar_cfg.pose_sigma_theta_gain = deg2rad(7.0);
lidar_cfg.min_quality_for_pose = 0.18;
lidar_cfg.random_dropout_probability = 0.03;
end
