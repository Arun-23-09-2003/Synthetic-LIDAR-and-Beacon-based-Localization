function [rmse_table, availability_table, config_table, aggregate_table] = collect_results_tables(cfg)
%COLLECT_RESULTS_TABLES Build all required result tables.

scenario_col = {};
case_id_col = {};
case_name_col = {};
position_rmse_col = [];
heading_rmse_col = [];
velocity_rmse_col = [];
max_position_col = [];
median_position_col = [];

availability_scenario_col = {};
beacon_avail_pct_col = [];
avg_beacon_count_col = [];
no_beacon_pct_col = [];
one_beacon_pct_col = [];
two_beacon_pct_col = [];
three_beacon_pct_col = [];
lidar_avail_pct_col = [];
avg_lidar_quality_col = [];

for scenario_idx = 1:cfg.num_scenarios
    scenario_name = scenario_folder_name(scenario_idx);
    scenario_data_dir = fullfile(cfg.paths.generated_data, scenario_name);
    scenario_result_dir = fullfile(cfg.paths.filter_outputs, scenario_name);

    beacon_data = load(fullfile(scenario_data_dir, 'beacon_measurements.mat'));
    lidar_data = load(fullfile(scenario_data_dir, 'lidar_pose_measurements.mat'));
    availability = compute_sensor_availability(beacon_data.beacon_measurements, lidar_data.lidar_pose);

    availability_scenario_col{end+1, 1} = scenario_name; %#ok<AGROW>
    beacon_avail_pct_col(end+1, 1) = availability.beacon_measurement_availability_pct; %#ok<AGROW>
    avg_beacon_count_col(end+1, 1) = availability.avg_available_beacons_per_update; %#ok<AGROW>
    no_beacon_pct_col(end+1, 1) = availability.no_beacon_update_pct; %#ok<AGROW>
    one_beacon_pct_col(end+1, 1) = availability.one_beacon_update_pct; %#ok<AGROW>
    two_beacon_pct_col(end+1, 1) = availability.two_beacon_update_pct; %#ok<AGROW>
    three_beacon_pct_col(end+1, 1) = availability.three_or_more_beacon_update_pct; %#ok<AGROW>
    lidar_avail_pct_col(end+1, 1) = availability.lidar_pose_availability_pct; %#ok<AGROW>
    avg_lidar_quality_col(end+1, 1) = availability.avg_lidar_quality; %#ok<AGROW>

    for case_idx = 1:numel(cfg.filters.cases)
        case_cfg = cfg.filters.cases(case_idx);
        result_data = load(fullfile(scenario_result_dir, [case_cfg.id '.mat']));
        metrics = result_data.metrics;

        scenario_col{end+1, 1} = scenario_name; %#ok<AGROW>
        case_id_col{end+1, 1} = case_cfg.id; %#ok<AGROW>
        case_name_col{end+1, 1} = case_cfg.name; %#ok<AGROW>
        position_rmse_col(end+1, 1) = metrics.rmse_position; %#ok<AGROW>
        heading_rmse_col(end+1, 1) = metrics.rmse_heading; %#ok<AGROW>
        velocity_rmse_col(end+1, 1) = metrics.rmse_velocity; %#ok<AGROW>
        max_position_col(end+1, 1) = metrics.max_position_error; %#ok<AGROW>
        median_position_col(end+1, 1) = metrics.median_position_error; %#ok<AGROW>
    end
end

rmse_table = table(scenario_col, case_id_col, case_name_col, ...
    position_rmse_col, heading_rmse_col, velocity_rmse_col, ...
    max_position_col, median_position_col, ...
    'VariableNames', {'scenario', 'case_id', 'case_name', ...
    'position_rmse_m', 'heading_rmse_rad', 'velocity_rmse_mps', ...
    'max_position_error_m', 'median_position_error_m'});

availability_table = table(availability_scenario_col, beacon_avail_pct_col, ...
    avg_beacon_count_col, no_beacon_pct_col, one_beacon_pct_col, ...
    two_beacon_pct_col, three_beacon_pct_col, lidar_avail_pct_col, ...
    avg_lidar_quality_col, ...
    'VariableNames', {'scenario', 'beacon_measurement_availability_pct', ...
    'avg_available_beacons_per_update', 'no_beacon_update_pct', ...
    'one_beacon_update_pct', 'two_beacon_update_pct', ...
    'three_or_more_beacon_update_pct', 'lidar_pose_availability_pct', ...
    'avg_lidar_quality'});

config_table = build_config_table(cfg);
aggregate_table = build_aggregate_table(cfg, rmse_table);
end

function config_table = build_config_table(cfg)
case_id = {};
case_name = {};
use_beacon = [];
beacon_adaptive = [];
use_lidar = [];
fixed_beacon_sigma_m = [];
for i = 1:numel(cfg.filters.cases)
    case_cfg = cfg.filters.cases(i);
    case_id{end+1, 1} = case_cfg.id; %#ok<AGROW>
    case_name{end+1, 1} = case_cfg.name; %#ok<AGROW>
    use_beacon(end+1, 1) = case_cfg.use_beacon; %#ok<AGROW>
    beacon_adaptive(end+1, 1) = case_cfg.beacon_adaptive; %#ok<AGROW>
    use_lidar(end+1, 1) = case_cfg.use_lidar; %#ok<AGROW>
    fixed_beacon_sigma_m(end+1, 1) = cfg.beacon_defaults.fixed_filter_sigma; %#ok<AGROW>
end

config_table = table(case_id, case_name, use_beacon, beacon_adaptive, ...
    use_lidar, fixed_beacon_sigma_m, ...
    'VariableNames', {'case_id', 'case_name', 'use_beacon', ...
    'beacon_adaptive', 'use_lidar', 'fixed_beacon_sigma_m'});
end

function aggregate_table = build_aggregate_table(cfg, rmse_table)
case_id = {};
case_name = {};
mean_position_rmse_m = [];
median_position_rmse_m = [];
mean_heading_rmse_rad = [];
mean_velocity_rmse_mps = [];
mean_max_position_error_m = [];

for i = 1:numel(cfg.filters.cases)
    case_cfg = cfg.filters.cases(i);
    rows = strcmp(rmse_table.case_id, case_cfg.id);
    case_id{end+1, 1} = case_cfg.id; %#ok<AGROW>
    case_name{end+1, 1} = case_cfg.name; %#ok<AGROW>
    mean_position_rmse_m(end+1, 1) = mean(rmse_table.position_rmse_m(rows)); %#ok<AGROW>
    median_position_rmse_m(end+1, 1) = median(rmse_table.position_rmse_m(rows)); %#ok<AGROW>
    mean_heading_rmse_rad(end+1, 1) = mean(rmse_table.heading_rmse_rad(rows)); %#ok<AGROW>
    mean_velocity_rmse_mps(end+1, 1) = mean(rmse_table.velocity_rmse_mps(rows)); %#ok<AGROW>
    mean_max_position_error_m(end+1, 1) = mean(rmse_table.max_position_error_m(rows)); %#ok<AGROW>
end

aggregate_table = table(case_id, case_name, mean_position_rmse_m, ...
    median_position_rmse_m, mean_heading_rmse_rad, mean_velocity_rmse_mps, ...
    mean_max_position_error_m, ...
    'VariableNames', {'case_id', 'case_name', 'mean_position_rmse_m', ...
    'median_position_rmse_m', 'mean_heading_rmse_rad', ...
    'mean_velocity_rmse_mps', 'mean_max_position_error_m'});
end
