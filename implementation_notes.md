# Implementation Notes for the AAS Manuscript Scaffold

These notes summarize the MATLAB implementation used to create `report/main.tex`.

## Folder Structure

- `config/`: configuration functions for seeds, maps, beacons, LiDAR, motion, filters, and output settings.
- `src/geometry/`: segment intersection, ray-casting, wall attenuation, path-clearance, and trajectory-wall validation helpers.
- `src/simulation/`: map generation, beacon creation, wall-aware trajectory generation, prediction input simulation, beacon simulation, and LiDAR simulation.
- `src/filters/`: NavState propagation and error-state EKF update functions.
- `src/metrics/`: RMSE and sensor availability metrics plus table assembly.
- `src/plotting/`: figure generation for root and per-scenario output folders.
- `data/generated/scenario_XXX/`: saved synthetic datasets.
- `results/filter_outputs/scenario_XXX/`: saved filter output `.mat` files.
- `results/figures/`: root figure set copied from the default scenario.
- `results/figures/scenario_XXX/`: full figure set for each scenario.
- `results/tables/`: exported CSV tables.

## Main Scripts

- `main.m`: clears the workspace, loads configuration, resets generated outputs, generates datasets, runs filters, and generates tables/figures.
- `generate_dataset.m`: generates saved synthetic scenarios only.
- `run_filters.m`: loads saved scenario data and runs filter cases only.
- `generate_outputs.m`: loads saved data and filter outputs, then writes metrics tables and figures.

## Configuration Files

- `config_random.m`: global seed `6505` and scenario seeds.
- `config_maps.m`: three scenario definitions, bounds, durations, beacon positions, beacon ranges, penetration budgets, requested waypoints, and dropout scale.
- `config_motion.m`: 50 Hz prediction rate, acceleration/yaw-rate noise, and per-scenario bias scales.
- `config_beacons.m`: 10 Hz beacon update rate, RSSI model, range-noise model, dropout probability, and fixed beacon filter sigma.
- `config_lidar.m`: 270 degree FOV, 181 beams, 12 m max range, 5 Hz scans, 2.5 Hz pose updates, and quality-dependent pose covariance parameters.
- `config_filters.m`: initial covariance, process noise floor, filter input noise, correction cap, minimum beacon Jacobian range, and five filter cases.
- `config_outputs.m`: figure resolution and size settings.

## Generated Data

Each scenario folder contains:

- `map.mat`
- `beacons.mat`
- `truth.mat`
- `prediction_inputs.mat`
- `beacon_measurements.mat`
- `lidar_scans.mat`
- `lidar_pose_measurements.mat`
- `scenario_config.mat`

The generated truth trajectory stores `truth.p` and `truth.v` as `2 x N`, and `truth.X` as `4 x 4 x N`.

## Filter Cases

The implemented filter cases are:

- `dead_reckoning`
- `beacon_fixed`
- `beacon_adaptive`
- `lidar_only`
- `lidar_beacon_adaptive`

All cases use the same saved measurements for a scenario. The filter initializes the nominal state from the first truth state and uses the configured initial covariance.

## Metrics

The exported metrics include:

- position RMSE
- heading RMSE
- velocity RMSE
- maximum position error
- median position error
- beacon measurement availability percentage
- average available beacons per beacon update
- zero/one/two/three-plus beacon update percentages
- LiDAR pose availability percentage
- average LiDAR quality

Tables are saved under `results/tables/`.

## Figures

The plotting pipeline exports 17 PNG figures per scenario plus a root default-scenario figure set. The report scaffold references the root figure files. Per-scenario figure folders are available for replacing or expanding figure panels.

## Key Code Assumptions

- The environment is a known two-dimensional wall-segment map.
- Wall openings are represented by gaps between wall segments.
- Truth trajectories are planned with a grid-based wall-avoiding planner and validated against wall intersections before saving.
- The simulator may use true pose, wall intersections, true range, and true attenuation.
- The filter does not use true attenuation, wall crossings, true range before noise, or true noise samples.
- Beacon availability is determined by range limit, wall attenuation budget, RSSI threshold, and random dropout.
- LiDAR scan matching is not implemented. The simulator converts scans into synthetic pose measurements with quality-dependent covariance.
- The EKF state is a 2D NavState matrix with five tangent coordinates.
- Measurement updates use tangent-space EKF corrections and compose corrections back onto the NavState group with `navstate_exp`.
- The initial nominal state is the saved truth state at the first timestep.

## Gaps Between README and Implementation

- The README asks for use of an AAS LaTeX template, but no repository-local AAS template file was found. The report scaffold uses the standard `aastex631` class name.
- The README says five scenarios are preferred if runtime remains reasonable; the implementation generates the minimum required three scenarios.
- The project uses synthetic LiDAR-derived pose measurements rather than implemented scan matching or raw beam updates, which is allowed by the README scope.
- The beacon definition includes `path_loss_exponent` and `tx_power`, but the implemented RSSI model uses linear distance loss and wall attenuation constants rather than a full path-loss equation.
- The output structure stores filter results in separate `.mat` files per case rather than one combined `results(case_id)` array; each file contains the required estimated states, covariance history, innovation history, usage history, runtime info, and metrics.
- Tables in `report/main.tex` are static transcriptions of current CSV outputs. They should be refreshed manually if the MATLAB pipeline is rerun with changed parameters or random seeds.
