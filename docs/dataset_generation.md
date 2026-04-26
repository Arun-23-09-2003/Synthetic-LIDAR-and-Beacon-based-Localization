# Dataset Generation

Dataset generation is handled by `generate_dataset.m` and the functions in `src/simulation/`.

## Main Functions

- `generate_dataset.m`: loops over configured scenarios and saves each scenario folder.
- `src/simulation/generate_scenario_dataset.m`: coordinates map, beacon, trajectory, odometry, beacon measurement, and LiDAR generation for one scenario.
- `src/simulation/generate_truth_trajectory.m`: creates a smooth 2D trajectory from scenario waypoints and exports NavState matrices.
- `src/simulation/plan_wall_avoiding_path.m`: routes requested waypoint regions through valid wall openings before trajectory sampling.
- `src/simulation/simulate_prediction_inputs.m`: computes body-frame acceleration and yaw-rate from truth, then adds noise and per-scenario bias.
- `src/simulation/simulate_beacon_measurements.m`: generates range, RSSI, and availability matrices.
- `src/simulation/simulate_lidar_measurements.m`: ray-casts LiDAR scans and creates synthetic LiDAR pose measurements.

## Inputs

- `cfg`: full project configuration.
- `scenario_cfg`: one scenario definition from `cfg.scenarios`.
- Fixed random seed from `scenario_cfg.seed`.

## Outputs

Each `data/generated/scenario_XXX/` folder contains:

- `map.mat`
- `beacons.mat`
- `truth.mat`
- `prediction_inputs.mat`
- `beacon_measurements.mat`
- `lidar_scans.mat`
- `lidar_pose_measurements.mat`
- `scenario_config.mat`

## Saved Data Shape

- `truth.p` and `truth.v` are `2 x N`.
- `truth.X` is `4 x 4 x N`.
- `truth.requested_waypoints` stores the high-level requested route.
- `truth.waypoints` stores the wall-avoiding planned route actually sampled.
- Beacon measurement matrices are `num_beacons x num_beacon_times`.
- LiDAR ranges and valid masks are `num_beams x num_scans`.

## Important Assumptions

Dataset generation may use privileged simulator knowledge such as true pose, wall intersections, and attenuation. The filter pipeline only uses robot-side measurement fields.

Generated trajectories are validated with `trajectory_wall_crossings.m` before they are saved. If any sampled trajectory segment intersects a wall, generation raises an error instead of silently saving an invalid dataset.
