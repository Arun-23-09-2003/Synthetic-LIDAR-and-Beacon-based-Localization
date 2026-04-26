# Configuration

Configuration is centralized in `config/load_config.m`, which calls the other config files and returns one `cfg` structure.

## Main Files

- `config/load_config.m`: builds paths, random seed settings, motion parameters, beacon defaults, LiDAR settings, filter cases, output settings, and scenario definitions.
- `config/config_random.m`: defines `cfg.random.global_seed = 6505` and scenario-specific seeds.
- `config/config_maps.m`: defines the three scenario layouts, bounds, beacon positions, ranges, penetration budgets, trajectory waypoints, and durations.
- `config/config_motion.m`: defines the 50 Hz prediction rate and acceleration/yaw-rate noise.
- `config/config_beacons.m`: defines beacon rate, RSSI model constants, range noise model constants, dropout probability, and fixed filter covariance.
- `config/config_lidar.m`: defines field of view, beam count, max range, scan rate, pose update rate, and quality-dependent pose noise.
- `config/config_filters.m`: defines EKF initial covariance, process noise, and the five filter cases.
- `config/config_outputs.m`: defines figure resolution and plotting defaults.

## Inputs

Configuration files do not load generated data. They only provide parameters and paths.

## Outputs

The main output is the `cfg` structure passed into generation, filtering, metrics, and plotting functions.

## Important Assumptions

Scenario-specific geometry and sensor placement live in `config_maps.m`. Source modules should use values from `cfg` or `scenario_cfg` rather than hard-coding scenario parameters.
