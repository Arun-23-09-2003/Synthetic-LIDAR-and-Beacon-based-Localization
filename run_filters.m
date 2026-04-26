function run_filters(cfg)
%RUN_FILTERS Load saved datasets and run all configured filters.

if nargin < 1 || isempty(cfg)
    project_root = setup_project_paths();
    cfg = load_config(project_root);
end

ensure_dir(cfg.paths.filter_outputs);

for scenario_idx = 1:cfg.num_scenarios
    scenario_name = scenario_folder_name(scenario_idx);
    scenario_data_dir = fullfile(cfg.paths.generated_data, scenario_name);
    scenario_result_dir = fullfile(cfg.paths.filter_outputs, scenario_name);
    ensure_dir(scenario_result_dir);

    fprintf('Running filters for %s\n', scenario_name);

    data = load_scenario_data(scenario_data_dir);

    for case_idx = 1:numel(cfg.filters.cases)
        case_cfg = cfg.filters.cases(case_idx);
        fprintf('  %s\n', case_cfg.id);
        result = run_filter_case(data, case_cfg, cfg);

        estimated_states = result.estimated_states;
        state_covariances = result.state_covariances;
        innovation_history = result.innovation_history;
        measurement_usage_history = result.measurement_usage_history;
        runtime_info = result.runtime_info;
        metrics = result.metrics;

        save(fullfile(scenario_result_dir, [case_cfg.id '.mat']), ...
            'estimated_states', 'state_covariances', 'innovation_history', ...
            'measurement_usage_history', 'runtime_info', 'metrics');
    end
end
end

function data = load_scenario_data(scenario_data_dir)
map_data = load(fullfile(scenario_data_dir, 'map.mat'));
beacon_data = load(fullfile(scenario_data_dir, 'beacons.mat'));
truth_data = load(fullfile(scenario_data_dir, 'truth.mat'));
prediction_data = load(fullfile(scenario_data_dir, 'prediction_inputs.mat'));
beacon_meas_data = load(fullfile(scenario_data_dir, 'beacon_measurements.mat'));
lidar_scan_data = load(fullfile(scenario_data_dir, 'lidar_scans.mat'));
lidar_pose_data = load(fullfile(scenario_data_dir, 'lidar_pose_measurements.mat'));
scenario_config_data = load(fullfile(scenario_data_dir, 'scenario_config.mat'));

data.map = map_data.map;
data.beacons = beacon_data.beacons;
data.truth = truth_data.truth;
data.odom = prediction_data.odom;
data.beacon_measurements = beacon_meas_data.beacon_measurements;
data.lidar_scan = lidar_scan_data.lidar_scan;
data.lidar_pose = lidar_pose_data.lidar_pose;
data.scenario_config = scenario_config_data.scenario_config;
end
