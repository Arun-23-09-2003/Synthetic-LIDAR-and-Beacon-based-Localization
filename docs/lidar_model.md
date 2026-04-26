# LiDAR Model

LiDAR simulation is implemented in `src/simulation/simulate_lidar_measurements.m`.

## Main Functions

- `raycast_map.m`: returns nearest wall intersections for each beam.
- `simulate_lidar_measurements.m`: generates noisy scan ranges, valid masks, quality scores, and pose measurements.
- `generate_project_figures.m`: plots example scans and quality time histories.

## Inputs

- `map`
- `truth`
- LiDAR parameters from `config_lidar.m`

## Outputs

The saved `lidar_scan` structure includes:

- `t`
- `step_index`
- `ranges`
- `angles`
- `valid_mask`
- `quality`
- `hit_points`

The saved `lidar_pose` structure includes:

- `t`
- `step_index`
- `pose`
- `R`
- `quality`
- `available`

## Quality Model

Scan quality is based on valid return fraction, range spread, and near-wall return fraction. The resulting quality controls LiDAR pose measurement covariance.

## Important Assumptions

The EKF does not update directly on individual beams. It uses the synthetic LiDAR-derived pose and covariance, matching the project scope of localization filtering rather than scan matching or SLAM.
