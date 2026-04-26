function filter_cfg = config_filters()
%CONFIG_FILTERS EKF process noise and filter cases.

filter_cfg.initial_covariance = diag([deg2rad(4.0)^2, 0.25^2, 0.25^2, 0.18^2, 0.18^2]);
filter_cfg.process_noise_floor = diag([1e-7, 1e-6, 1e-6, 1e-5, 1e-5]);
filter_cfg.accel_noise_sigma = 0.11;
filter_cfg.gyro_noise_sigma = 0.018;
filter_cfg.max_correction_norm = 4.0;
filter_cfg.min_range_for_jacobian = 0.20;

cases = struct('id', {}, 'name', {}, 'use_beacon', {}, 'beacon_adaptive', {}, 'use_lidar', {});
cases(end+1) = make_case('dead_reckoning', 'Dead reckoning', false, false, false);
cases(end+1) = make_case('beacon_fixed', 'Beacon fixed covariance', true, false, false);
cases(end+1) = make_case('beacon_adaptive', 'Beacon adaptive covariance', true, true, false);
cases(end+1) = make_case('lidar_only', 'LiDAR only', false, false, true);
cases(end+1) = make_case('lidar_beacon_adaptive', 'LiDAR + adaptive beacon fusion', true, true, true);
filter_cfg.cases = cases;
end

function case_cfg = make_case(id, name, use_beacon, beacon_adaptive, use_lidar)
case_cfg.id = id;
case_cfg.name = name;
case_cfg.use_beacon = use_beacon;
case_cfg.beacon_adaptive = beacon_adaptive;
case_cfg.use_lidar = use_lidar;
end
