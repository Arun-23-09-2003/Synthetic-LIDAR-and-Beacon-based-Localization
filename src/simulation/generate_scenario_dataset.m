function generate_scenario_dataset(cfg, scenario_cfg, scenario_dir)
%GENERATE_SCENARIO_DATASET Generate and save one complete scenario.

map = create_indoor_map(scenario_cfg);
beacons = create_beacons(cfg, scenario_cfg);
truth = generate_truth_trajectory(cfg, scenario_cfg, map);
odom = simulate_prediction_inputs(cfg, truth);
truth.u_true = odom.u_true;
truth.u_noisy = odom.u_noisy;

beacon_measurements = simulate_beacon_measurements(cfg, scenario_cfg, map, beacons, truth);
[lidar_scan, lidar_pose] = simulate_lidar_measurements(cfg, scenario_cfg, map, truth);

scenario_config = scenario_cfg;
scenario_config.generated_with_global_seed = cfg.random.global_seed;
scenario_config.motion = cfg.motion;
scenario_config.beacon_defaults = cfg.beacon_defaults;
scenario_config.lidar = cfg.lidar;

save(fullfile(scenario_dir, 'map.mat'), 'map');
save(fullfile(scenario_dir, 'beacons.mat'), 'beacons');
save(fullfile(scenario_dir, 'truth.mat'), 'truth');
save(fullfile(scenario_dir, 'prediction_inputs.mat'), 'odom');
save(fullfile(scenario_dir, 'beacon_measurements.mat'), 'beacon_measurements');
save(fullfile(scenario_dir, 'lidar_scans.mat'), 'lidar_scan');
save(fullfile(scenario_dir, 'lidar_pose_measurements.mat'), 'lidar_pose');
save(fullfile(scenario_dir, 'scenario_config.mat'), 'scenario_config');
end
