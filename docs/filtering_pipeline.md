# Filtering Pipeline

Filtering is handled by `run_filters.m` and functions in `src/filters/`.

## Main Functions

- `run_filters.m`: loads saved scenario data and writes one result file per case.
- `run_filter_case.m`: executes one EKF configuration over one scenario.
- `propagate_navstate.m`: performs prediction using noisy body acceleration and yaw rate.
- `ekf_tangent_update.m`: applies tangent-space EKF corrections through NavState composition.
- `compute_filter_metrics.m`: computes errors and RMSE values after each case.

## Inputs

For each scenario, `run_filters.m` loads:

- `map.mat`
- `beacons.mat`
- `truth.mat`
- `prediction_inputs.mat`
- `beacon_measurements.mat`
- `lidar_scans.mat`
- `lidar_pose_measurements.mat`
- `scenario_config.mat`

## Filter Cases

The configured cases are:

- `dead_reckoning`
- `beacon_fixed`
- `beacon_adaptive`
- `lidar_only`
- `lidar_beacon_adaptive`

All cases use the same saved truth, map, beacons, odometry, beacon measurements, and LiDAR measurements.

## Outputs

Each `results/filter_outputs/scenario_XXX/<case>.mat` file includes:

- `estimated_states`
- `state_covariances`
- `innovation_history`
- `measurement_usage_history`
- `runtime_info`
- `metrics`

## Important Assumptions

`run_filters.m` never regenerates maps, truth trajectories, or measurements. It is safe to rerun after `generate_dataset.m`.
