# AE6505 Final Project Implementation Specification

## Project Title

**Adaptive Error-State Kalman Filtering for Indoor Robot Localization Using LiDAR and Beacon Measurements**

## Purpose

This project studies indoor robot localization in a synthetic but physically grounded 2D environment. A mobile robot moves through a known indoor map with walls of different thicknesses and estimates its navigation state using a Kalman-filter-based sensor fusion pipeline.

The system includes:

- A 2D NavState process model
- Beacon range measurements with distance-dependent and signal-strength-dependent uncertainty
- LiDAR ray-cast simulation and LiDAR-derived pose measurements
- Intermittent and multi-rate sensor availability
- Multiple synthetic scenarios for evaluation

The goal is **not** to build a full SLAM system. The goal is to evaluate how a Kalman filter behaves when measurements are nonlinear, intermittent, range-dependent, and degraded by indoor structure.

All source code must be written in **MATLAB**.

---

## Core Research Question

Can an adaptive error-state Kalman filter improve indoor robot localization when beacon measurements are intermittent and have distance-dependent and signal-strength-dependent uncertainty?

---

## Hypothesis

A Kalman filter that adapts beacon measurement covariance using measured range and received signal strength will outperform a naive filter that treats all beacon measurements as equally reliable. Fusion with LiDAR pose measurements should further improve robustness in regions where beacon measurements are weak, blocked, or geometrically poor.

---

## High-Level Scope

### Included

- Synthetic indoor map generation
- Synthetic robot trajectory generation
- Synthetic prediction input generation
- Synthetic beacon range and RSSI measurement generation
- Synthetic LiDAR scan and LiDAR pose-measurement generation
- Error-state EKF using a 2D NavState representation
- Comparison of multiple filtering configurations
- Quantitative evaluation using known ground truth
- Automatic generation of all figures and tables
- MATLAB scripts to reproduce all outputs

### Not Included

- Full SLAM
- Real sensor data collection
- Full wireless propagation modeling
- Raw point-cloud registration as the main estimator
- ROS, Python, C++, Gazebo, or external robotics simulators
- Manuscript or report writing

---

## Project Flow

The implementation should separate **dataset generation** from **filtering** and **output generation**. This allows the map, beacon layout, ground-truth trajectory, and sensor measurements to be generated once and reused while iterating on the filter.

Top-level MATLAB scripts:

```text
main.m
generate_dataset.m
run_filters.m
generate_outputs.m
```

The full pipeline should be:

```text
Configuration
  -> Dataset generation
  -> Saved synthetic scenarios
  -> Filter execution
  -> Saved filter outputs
  -> Metrics, figures, and tables
```

The command below must regenerate the complete project from scratch:

```matlab
main
```

The modular workflow should also support:

```matlab
generate_dataset
run_filters
generate_outputs
```

---

## State Representation

Use a 2D NavState matrix Lie group representation for the robot state:

\[
X =
\begin{bmatrix}
R & p & v \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
\]

where:

\[
R \in SO(2)
\]

\[
p =
\begin{bmatrix}
x \\
y
\end{bmatrix}
\]

\[
v =
\begin{bmatrix}
\dot{x} \\
\dot{y}
\end{bmatrix}
\]

Here, \(R\) represents the robot heading, \(p\) represents the 2D position, and \(v\) represents the 2D velocity. This gives the navigation state five degrees of freedom:

\[
\theta,\ x,\ y,\ \dot{x},\ \dot{y}
\]

The estimated state should remain on the NavState group instead of being updated as a purely Euclidean vector. The EKF should maintain a nominal state on the manifold and represent uncertainty using a local tangent-space error state.

The local error state is:

\[
\delta x =
\begin{bmatrix}
\delta \theta &
\delta p_x &
\delta p_y &
\delta v_x &
\delta v_y
\end{bmatrix}^T
\]

The covariance matrix is therefore:

\[
P \in \mathbb{R}^{5 \times 5}
\]

Prediction and correction steps should be applied as small tangent-space perturbations. These perturbations are mapped back onto the NavState manifold using the exponential map. After an EKF correction \(\delta x\), the nominal state should be updated using group composition rather than direct vector addition:

\[
X^+ = X\operatorname{Exp}(\delta x)
\]

where \(\operatorname{Exp}(\delta x)\) maps the local error coordinates from the Lie algebra to the NavState group.

---

## NavState Exponential Map Contract

The implementation must include a dedicated helper function for the NavState exponential map:

```matlab
function dX = navstate_exp(delta)
```

with:

```matlab
delta = [dtheta; dpx; dpy; dvx; dvy];
```

Define:

\[
\delta x =
\begin{bmatrix}
\phi & \rho_x & \rho_y & \nu_x & \nu_y
\end{bmatrix}^T
\]

where:

\[
\phi = \delta \theta
\]

\[
\rho =
\begin{bmatrix}
\rho_x \\
\rho_y
\end{bmatrix}
\]

\[
\nu =
\begin{bmatrix}
\nu_x \\
\nu_y
\end{bmatrix}
\]

Use a third-order Taylor approximation for the left-Jacobian terms used to project local translational perturbations onto the group:

\[
A(\phi) \approx 1 - \frac{\phi^2}{6}
\]

\[
B(\phi) \approx \frac{\phi}{2} - \frac{\phi^3}{24}
\]

\[
J(\phi) \approx
\begin{bmatrix}
A(\phi) & -B(\phi) \\
B(\phi) & A(\phi)
\end{bmatrix}
\]

The NavState exponential map should then use:

\[
R_\delta = R(\phi)
\]

\[
p_\delta = J(\phi)\rho
\]

\[
v_\delta = J(\phi)\nu
\]

The output should be a valid NavState group element:

```text
dX =
[R_delta, p_delta, v_delta;
 0,       1,       0;
 0,       0,       1]
```

The returned matrix should then be composed with the current nominal state during EKF prediction or correction. This keeps the estimate on the manifold while allowing the EKF covariance and Kalman updates to operate in local tangent-space coordinates.

---

## Environment Model

The indoor environment is a known 2D floor plan made of wall segments. The map should be large enough that many beacon measurements become naturally unavailable due to range limits and wall attenuation.

Recommended map size:

```text
30 m x 20 m or larger
```

Each wall is represented as:

```matlab
wall.x1
wall.y1
wall.x2
wall.y2
wall.thickness
wall.material_factor
wall.label
```

Recommended wall classes:

| Wall Type | Suggested Thickness | Notes |
|---|---:|---|
| Outer boundary | 0.30 m to 0.40 m | Strongest attenuation |
| Main interior wall | 0.12 m to 0.20 m | Moderate attenuation |
| Thin partition | 0.05 m to 0.10 m | Weak attenuation |
| Door/glass/opening region | 0.00 m to 0.03 m | Low attenuation or pass-through region |

The environment module must provide functions to:

- Plot the map
- Check line-segment intersections between sensor rays and walls
- Compute hidden wall attenuation between a robot position and a beacon
- Ray-cast LiDAR beams against walls

Required environment outputs:

```matlab
sim.map.walls
sim.map.bounds
sim.map.wall_thickness_values
sim.map.room_labels
```

Important distinction:

- The **simulator** may use wall geometry and wall thickness to decide if a beacon signal exists.
- The **filter** must not directly know how many walls the signal passed through or the wall thickness crossed.
- The filter should only use received beacon range, RSSI/signal strength, beacon ID, timestamp, and availability.

---

## Multiple Scenario Requirement

The project should not rely on a single map or single trajectory. The dataset should include multiple synthetic scenarios to make the analysis more comprehensive.

A minimum of **three scenarios** is required. Five scenarios is preferred if runtime remains reasonable.

Recommended scenario types:

| Scenario Type | Purpose |
|---|---|
| Open hallway map | Tests long-range motion and LiDAR ambiguity |
| Multi-room map | Tests wall attenuation and beacon dropout |
| Dense indoor map | Tests frequent line-of-sight changes |
| Sparse beacon map | Tests weak beacon coverage |
| Large indoor map | Tests out-of-range beacon behavior |

Each scenario may vary:

- Map size
- Wall layout
- Wall thicknesses
- Beacon count
- Beacon placement
- Robot trajectory
- Beacon penetration budgets
- LiDAR feature quality
- Sensor dropout behavior

The simulation should include regions where:

- No beacon is available
- Only one beacon is available
- Two beacons are available
- Three or more beacons are available
- LiDAR is the dominant correction source
- Beacons are the dominant correction source

This prevents the beacon system from being unrealistically overdetermined at all times.

---

## Robot Ground-Truth Trajectory

The synthetic trajectory should be generated from smooth commanded motion through multiple indoor regions.

Minimum trajectory requirements:

- Starts in one room
- Passes through at least one hallway
- Turns around corners
- Enters at least one beacon-shadow or weak-signal region
- Returns to or approaches a region with better sensor visibility
- Includes both straight and turning segments

The ground-truth trajectory must include:

```matlab
truth.t
truth.theta
truth.p          % 2 x N or N x 2, but use one convention consistently
truth.v          % 2 x N or N x 2, matching truth.p
truth.X          % 4 x 4 x N NavState matrices
truth.u_true
truth.u_noisy
```

---

## Prediction Input Simulation

The filter needs a prediction source. Use a synthetic odometry-like or IMU-like input generated from the known truth trajectory and corrupted with noise.

Recommended input option A:

```matlab
u.v_forward      % noisy forward speed
u.omega_meas     % noisy yaw-rate measurement
```

Recommended input option B:

```matlab
u.accel_body     % noisy body-frame planar acceleration
u.omega_meas     % noisy yaw-rate measurement
```

The selected input model must be used consistently in:

- Truth simulation
- Prediction input generation
- EKF propagation
- Motion profile plots

The prediction model should propagate:

- Heading using measured yaw rate
- Velocity using the selected motion input
- Position using velocity and time step

Required prediction output:

```matlab
measurements.odom
```

---

## Beacon Measurement Simulation

### Beacon Definition

Each beacon is fixed in the known map. More than three beacons should be placed in the environment so the robot experiences changing beacon availability over time.

Each beacon should contain:

```matlab
beacon.id
beacon.position              % [x; y]
beacon.max_range
beacon.penetration_budget
beacon.tx_power
beacon.rssi_ref
beacon.rssi_threshold
beacon.path_loss_exponent
beacon.range_sigma_base
beacon.range_sigma_gain
beacon.rssi_sigma
```

Recommended beacon maximum range:

```text
6 m to 12 m
```

The map should be significantly larger than the effective range of any single beacon so that the robot cannot observe all beacons at once.

### Simulator-Side Beacon Logic

At each beacon measurement time, for every beacon:

1. Compute true range from robot to beacon.
2. Reject the measurement if true range exceeds `beacon.max_range`.
3. Compute hidden wall attenuation using wall intersections between the robot and beacon.
4. Reject the measurement if attenuation exceeds `beacon.penetration_budget`.
5. Generate RSSI/signal strength.
6. Reject the measurement if RSSI is below `beacon.rssi_threshold`.
7. Generate measured range with noise.
8. Mark the measurement as available or dropped out.

A simple simulator-side penetration model is acceptable:

\[
A_i =
\sum_j t_j m_j
\]

where \(t_j\) is crossed wall thickness and \(m_j\) is material factor.

A beacon is available only if:

\[
A_i \leq A_{max,i}
\]

and:

\[
d_i \leq d_{max,i}
\]

A simple signal-strength model is:

\[
RSSI_i = RSSI_{0,i} - \alpha_d d_i - \alpha_w A_i + n_s
\]

where \(d_i\) is true range and \(A_i\) is hidden wall attenuation.

The true range-noise standard deviation may be:

\[
\sigma_{r,i} =
\sigma_{0,i} + k_d d_i + k_s\max(0, RSSI_{ref} - RSSI_i)
\]

Then generate:

\[
z_i = d_i + n_r
\]

### Filter-Side Beacon Contract

The filter must not use hidden wall count, hidden wall thickness, true attenuation, true robot pose, or true range before noise.

The filter receives only:

```matlab
z_beacon.timestamp
z_beacon.beacon_id
z_beacon.range_meas
z_beacon.rssi
z_beacon.available
```

The robot-side measurement model is:

\[
h_i(X) = \|p - b_i\|
\]

where \(b_i\) is the known beacon position.

The residual is:

\[
r_i = z_i - h_i(\hat{X})
\]

The robot-side adaptive covariance should use measured range and/or RSSI, not direct wall knowledge:

\[
\hat{\sigma}_{r,i} =
\sigma_{0,i} + k_d z_i + k_s\max(0, RSSI_{ref} - RSSI_i)
\]

\[
R_i = \hat{\sigma}_{r,i}^2
\]

For the local error ordering:

```matlab
delta_x = [delta_theta; delta_px; delta_py; delta_vx; delta_vy];
```

use the beacon Jacobian:

\[
H_i =
\begin{bmatrix}
0 &
\frac{\hat{p}_x - b_{x,i}}{\hat{d}_i} &
\frac{\hat{p}_y - b_{y,i}}{\hat{d}_i} &
0 &
0
\end{bmatrix}
\]

where:

\[
\hat{d}_i = \|\hat{p} - b_i\|
\]

This keeps the estimator realistic because it adapts measurement confidence using only quantities available onboard the robot.

---

## LiDAR Simulation

The project should include LiDAR simulation but should avoid becoming a scan-matching or SLAM project. The map is assumed to be known, and LiDAR is used as a localization correction source.

### LiDAR Ray-Cast Model

Ray-cast a 2D LiDAR scan against the wall map.

Recommended parameters:

```matlab
lidar.fov_deg
lidar.num_beams
lidar.max_range
lidar.range_sigma
lidar.scan_rate_hz
lidar.pose_update_rate_hz
lidar.min_valid_returns
lidar.quality_model_enabled
```

At selected LiDAR update times, the simulator should:

1. Place the LiDAR at the true robot pose.
2. Cast rays across the LiDAR field of view.
3. Find the nearest wall intersection for each ray.
4. Add range noise.
5. Mark rays as max-range if no wall intersection is found.
6. Compute a LiDAR quality score from scan geometry.

Required LiDAR scan outputs:

```matlab
measurements.lidar_scan.t
measurements.lidar_scan.ranges
measurements.lidar_scan.angles
measurements.lidar_scan.valid_mask
measurements.lidar_scan.quality
```

### LiDAR Pose Measurement Used by the Filter

The EKF does not need to directly update on every LiDAR beam. To keep the project focused on Kalman filtering, the implementation may convert each scan into a synthetic LiDAR pose measurement:

\[
z_{lidar} =
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
+ n_{lidar}
\]

The LiDAR pose covariance should depend on scan quality:

\[
R_{lidar,k} =
\operatorname{diag}
\left(
\sigma_x^2(q_k),
\sigma_y^2(q_k),
\sigma_\theta^2(q_k)
\right)
\]

where \(q_k\) is the simulated LiDAR quality score.

Required LiDAR pose outputs:

```matlab
measurements.lidar_pose.t
measurements.lidar_pose.pose       % [x; y; theta]
measurements.lidar_pose.R          % 3 x 3 covariance
measurements.lidar_pose.quality
measurements.lidar_pose.available
```

The filter may use the LiDAR-provided pose estimate and covariance. It must not use the true robot pose.

### LiDAR Measurement Model

The LiDAR pose update measures:

\[
h_{lidar}(X) =
\begin{bmatrix}
p_x \\
p_y \\
\theta
\end{bmatrix}
\]

Residual:

\[
r =
\begin{bmatrix}
z_x - \hat{p}_x \\
z_y - \hat{p}_y \\
\operatorname{wrapToPi}(z_\theta - \hat{\theta})
\end{bmatrix}
\]

For the local error order:

```matlab
delta_x = [delta_theta; delta_px; delta_py; delta_vx; delta_vy];
```

use:

\[
H_{lidar} =
\begin{bmatrix}
0 & 1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 \\
1 & 0 & 0 & 0 & 0
\end{bmatrix}
\]

---

## Sensor Rates

Use different sensor rates to make the simulation realistic.

Recommended default values:

| Source | Rate |
|---|---:|
| Prediction / odometry input | 50 Hz |
| Beacon measurements | 10 Hz |
| LiDAR scan generation | 5 Hz |
| LiDAR pose correction | 2 Hz to 5 Hz |

The filter loop must support missing measurements at any timestep.

---

## Measurement-Side Assumptions

The code should clearly separate simulator-side knowledge from robot/filter-side knowledge.

Simulator-side privileged knowledge:

```text
true robot state
true wall intersections
true wall thicknesses
true beacon penetration usage
true range before noise
true LiDAR ray intersections
```

Robot/filter-side available knowledge:

```text
known beacon positions
known map for LiDAR localization model
noisy odometry or IMU-like input
LiDAR-derived pose measurement and covariance
beacon measured range
beacon RSSI/signal strength
beacon ID
sensor timestamp
sensor availability/dropout
```

The filter must not directly use true wall count, true wall thickness crossed, true robot pose, or true noise values.

---

## Filter Configurations

Each filter case must use the same saved ground truth, map, trajectory, and sensor measurements.

Required filter cases:

| Case | Description | Purpose |
|---|---|---|
| Dead reckoning | Prediction only, no measurement updates | Establishes the weakest baseline |
| Beacon fixed covariance | Uses available beacon ranges with constant covariance | Baseline for naive beacon fusion |
| Beacon adaptive covariance | Uses beacon range/RSSI-dependent covariance | Tests the main beacon-noise hypothesis |
| LiDAR only | Uses prediction and LiDAR pose updates | Isolates LiDAR correction performance |
| LiDAR + adaptive beacon fusion | Uses prediction, adaptive beacon updates, and LiDAR pose updates | Final proposed method |

Filter outputs should be stored in a consistent structure:

```matlab
results(case_id).name
results(case_id).t
results(case_id).X_hat
results(case_id).p_hat
results(case_id).v_hat
results(case_id).theta_hat
results(case_id).P
results(case_id).innovation_history
results(case_id).measurement_usage_history
results(case_id).position_error
results(case_id).heading_error
results(case_id).velocity_error
results(case_id).rmse_position
results(case_id).rmse_heading
results(case_id).rmse_velocity
```

---

## Required Metrics

Compute metrics against ground truth for every scenario and for the aggregate set of scenarios.

### Position RMSE

\[
RMSE_p =
\sqrt{
\frac{1}{N}
\sum_{k=1}^{N}
\|p_k - \hat{p}_k\|^2
}
\]

### Heading RMSE

\[
RMSE_\theta =
\sqrt{
\frac{1}{N}
\sum_{k=1}^{N}
\operatorname{wrapToPi}(\theta_k - \hat{\theta}_k)^2
}
\]

### Velocity RMSE

\[
RMSE_v =
\sqrt{
\frac{1}{N}
\sum_{k=1}^{N}
\|v_k - \hat{v}_k\|^2
}
\]

Additional useful metrics:

- Maximum position error
- Median position error
- Beacon availability percentage
- Average number of available beacons per timestep
- LiDAR measurement availability percentage
- Optional normalized innovation squared statistics

---

## Required Figures

All figures must be exported automatically to:

```text
results/figures/
```

The plotting pipeline should not require manual editing, screenshots, or manual figure formatting. Every figure should be generated from the same configuration and simulation outputs used by the filters.

| Figure | Filename | Required Content | Purpose |
|---|---|---|---|
| Indoor environment map | `fig_map_wall_thickness.png` | 2D floor plan, outer walls, inner walls, wall thickness labels or color-coded thickness | Shows how the synthetic indoor environment is constructed |
| Beacon placement and nominal coverage | `fig_beacon_layout.png` | Beacon locations, beacon IDs, nominal range circles, optional penetration-budget indicator | Shows sensor geometry and expected coverage regions |
| Robot dynamics and true trajectory | `fig_true_trajectory.png` | True trajectory overlaid on the map, start/end markers, heading arrows | Shows the motion scenario used for filtering |
| Control input / motion profile | `fig_motion_profile.png` | True and noisy input signals, such as speed, yaw rate, acceleration, or velocity components | Shows how the robot dynamics model is driven |
| Example LiDAR ray-cast scan | `fig_lidar_scan_example.png` | Robot pose, LiDAR rays, wall intersections, max-range rays, returned scan points | Explains how synthetic LiDAR measurements are generated |
| LiDAR measurement quality over time | `fig_lidar_quality.png` | LiDAR update availability, simulated pose-measurement standard deviation, or scan quality score over time | Shows when LiDAR is reliable or degraded |
| Beacon availability over time | `fig_beacon_availability.png` | Time on x-axis, beacon ID on y-axis, binary availability or heatmap of usable measurements | Shows the reactive beacon measurement model |
| Beacon RSSI and range example | `fig_beacon_signal_profile.png` | For selected beacons, true range, measured range, RSSI/signal strength, and dropout regions | Shows distance-dependent and signal-dependent beacon behavior |
| Beacon measurement noise model | `fig_beacon_noise_model.png` | Measurement standard deviation versus measured range and/or RSSI | Explains robot-side beacon covariance generation |
| Measurement timeline | `fig_measurement_timeline.png` | Odometry prediction rate, LiDAR update times, beacon update times, and dropout intervals | Shows the multi-rate sensor fusion setup |
| Trajectory comparison | `fig_trajectory_comparison.png` | True trajectory, dead reckoning, beacon-only estimates, LiDAR-only estimate, fused estimate | Main qualitative localization result |
| Position error over time | `fig_position_error.png` | Position error norm for all compared methods | Main quantitative localization time history |
| Heading error over time | `fig_heading_error.png` | Heading error for all compared methods | Orientation performance comparison |
| Velocity error over time | `fig_velocity_error.png` | Velocity error norm for relevant methods | Shows performance of the NavState velocity estimate |
| Covariance consistency plot | `fig_covariance_consistency.png` | Position and/or heading error with approximate covariance bounds | Shows whether filter uncertainty is reasonable |
| RMSE comparison | `fig_rmse_comparison.png` | Bar chart or table-style plot comparing RMSE across methods | Final quantitative comparison |
| Final result summary | `fig_result_summary.png` | Compact multi-panel summary with map, trajectory comparison, position error, and RMSE | Useful compact summary of final results |

Minimum required figures:

```text
fig_map_wall_thickness.png
fig_beacon_layout.png
fig_true_trajectory.png
fig_lidar_scan_example.png
fig_beacon_availability.png
fig_beacon_signal_profile.png
fig_beacon_noise_model.png
fig_trajectory_comparison.png
fig_position_error.png
fig_heading_error.png
fig_rmse_comparison.png
```

The remaining figures should still be generated automatically if runtime allows.

---

## Required Tables

Generate tables automatically as `.csv` files and save them to:

```text
results/tables/
```

| Table | Filename | Contents |
|---|---|---|
| Filter RMSE summary | `table_rmse_summary.csv` | Position, heading, and velocity RMSE for each filter case and scenario |
| Sensor availability summary | `table_sensor_availability.csv` | Beacon and LiDAR availability statistics |
| Configuration summary | `table_filter_configs.csv` | Parameters used in each filter case |
| Aggregate metrics summary | `table_aggregate_metrics.csv` | Aggregate mean/median metrics across scenarios |

---

## Folder Structure

Use the following project structure:

```text
project_root/
  main.m
  generate_dataset.m
  run_filters.m
  generate_outputs.m

  config/
    load_config.m
    config_random.m
    config_maps.m
    config_beacons.m
    config_lidar.m
    config_motion.m
    config_filters.m
    config_outputs.m

  src/
    geometry/
    simulation/
    filters/
    metrics/
    plotting/
    utils/

  data/
    generated/
      scenario_001/
      scenario_002/
      scenario_003/

  results/
    filter_outputs/
      scenario_001/
      scenario_002/
      scenario_003/
    figures/
    tables/
    summary/
```

All source code must be MATLAB. Do not use Python, C++, ROS, Gazebo, or external robotics simulation frameworks.

---

## Configuration Contract

The configuration should be detailed enough that maps, sensors, trajectories, filters, and plots can be modified without changing source code.

Suggested configuration files:

```text
config/
  load_config.m
  config_random.m
  config_maps.m
  config_beacons.m
  config_lidar.m
  config_motion.m
  config_filters.m
  config_outputs.m
```

The configuration should include:

```text
random seeds
number of scenarios
map sizes
wall thickness values
beacon count
beacon range limits
beacon penetration budgets
RSSI model parameters
range noise parameters
LiDAR field of view
LiDAR beam count
LiDAR max range
LiDAR noise parameters
robot speed limits
trajectory duration
sample rates
filter process noise
filter measurement noise
output folders
figure export settings
```

Source code should not contain hard-coded scenario parameters except for small helper defaults.

---

## Dataset Generation Contract

`generate_dataset.m` is responsible for generating and saving all synthetic data used by the filters.

This script should create multiple scenarios. Each scenario must contain:

1. Indoor map geometry
2. Wall thickness and material parameters
3. Beacon placement
4. Beacon coverage limits
5. Ground-truth robot trajectory
6. Prediction inputs
7. Beacon range measurements
8. Beacon RSSI/signal-strength values
9. Beacon dropout/availability flags
10. LiDAR ray-cast scans
11. LiDAR-derived pose measurements
12. All random seeds used to generate the scenario

The dataset generation step must save all outputs to:

```text
data/generated/
```

Each scenario should be saved in its own folder:

```text
data/generated/scenario_001/
data/generated/scenario_002/
data/generated/scenario_003/
```

Each scenario folder should contain:

```text
map.mat
beacons.mat
truth.mat
prediction_inputs.mat
beacon_measurements.mat
lidar_scans.mat
lidar_pose_measurements.mat
scenario_config.mat
```

The saved data should be sufficient for the filter code to run without regenerating the map, truth trajectory, or sensor measurements.

---

## Filtering Contract

`run_filters.m` is responsible only for loading saved generated data and running filter configurations.

It should not regenerate maps, beacons, truth trajectories, or raw sensor measurements.

For each scenario, `run_filters.m` must load:

```text
map.mat
beacons.mat
truth.mat
prediction_inputs.mat
beacon_measurements.mat
lidar_scans.mat
lidar_pose_measurements.mat
scenario_config.mat
```

Then it must run the required filter configurations:

```text
dead_reckoning
beacon_fixed
beacon_adaptive
lidar_only
lidar_beacon_adaptive
```

Filter outputs should be saved to:

```text
results/filter_outputs/
```

using one folder per scenario:

```text
results/filter_outputs/scenario_001/
results/filter_outputs/scenario_002/
results/filter_outputs/scenario_003/
```

Each scenario result folder should contain:

```text
dead_reckoning.mat
beacon_fixed.mat
beacon_adaptive.mat
lidar_only.mat
lidar_beacon_adaptive.mat
```

Each filter output file should include:

```text
estimated_states
state_covariances
innovation_history
measurement_usage_history
runtime_info
metrics
```

---

## Output Generation Contract

`generate_outputs.m` is responsible for computing final metrics, exporting tables, and generating figures from saved datasets and saved filter results.

This script should not rerun the filters unless explicitly configured to do so.

It should read from:

```text
data/generated/
results/filter_outputs/
```

and write to:

```text
results/figures/
results/tables/
results/summary/
```

This separation allows figures and tables to be regenerated without changing the dataset or rerunning the full simulation pipeline.

---

## Main Script Contract

`main.m` is the full reproducibility entry point. It must regenerate all data, filter results, tables, and figures from scratch.

The command below must run without modification:

```matlab
main
```

`main.m` must:

1. Clear workspace.
2. Set the global random seed.
3. Load all configuration files.
4. Delete or overwrite prior generated outputs.
5. Run dataset generation.
6. Run all filter configurations.
7. Compute metrics.
8. Export result tables.
9. Generate and save all required figures.
10. Save all generated data needed to reproduce results.

Suggested structure:

```matlab
clear; clc; close all;

cfg = load_config();
rng(cfg.random.global_seed);

generate_dataset(cfg);
run_filters(cfg);
generate_outputs(cfg);
```

No manual file editing should be required.

---

## Random Seed Contract

The project must use fixed random seeds for reproducibility.

The global seed should be:

```matlab
rng(6505)
```

The configuration should also define scenario-specific seeds so that each map and run is repeatable:

```matlab
cfg.random.global_seed = 6505;
cfg.random.scenario_seeds = [650501, 650502, 650503, 650504, 650505];
```

Each scenario should set its own seed before generating the map, trajectory, and sensor measurements:

```matlab
rng(cfg.random.scenario_seeds(i));
```

The seed used for each scenario must be saved inside:

```text
data/generated/scenario_XXX/scenario_config.mat
```

---

## Reproducibility Requirements

The grader or reviewer should be able to run:

```matlab
main
```

and regenerate:

```text
data/generated/
results/filter_outputs/
results/figures/
results/tables/
results/summary/
```

After datasets already exist, the user should be able to run:

```matlab
run_filters
```

without regenerating maps or truth trajectories.

After filters have already been run, the user should be able to run:

```matlab
generate_outputs
```

without regenerating the dataset or rerunning filters.

---

## Plotting Style Requirements

All plots should be readable and publication-style.

General plotting requirements:

```text
Use consistent font sizes
Use clear axis labels with units
Use legends only where needed
Use equal axis scaling for map/trajectory figures
Use grid lines for time-history plots
Use descriptive titles only if useful
Export figures at high resolution
```

Recommended MATLAB export format:

```matlab
exportgraphics(gcf, fullfile("results", "figures", filename), "Resolution", 300);
```

Map and trajectory figures should use:

```matlab
axis equal
```

All filenames must match the required names exactly.

---

## Suggested Implementation Order

### Stage 1: Configuration and Folder Setup

- Create modular configuration files.
- Create output folders.
- Define random seed handling.
- Define scenario indexing and file naming.

### Stage 2: Dataset Generation

- Generate multiple indoor maps.
- Assign wall thickness and material parameters.
- Place beacons with limited coverage.
- Generate smooth ground-truth robot trajectories.
- Generate prediction inputs.
- Generate beacon measurements.
- Generate LiDAR scans.
- Generate LiDAR pose measurements.
- Save each scenario to `data/generated/scenario_XXX/`.

### Stage 3: Visualization of Generated Data

- Plot wall-thickness map.
- Plot beacon layout and coverage.
- Plot true trajectory.
- Plot example LiDAR scan.
- Plot beacon availability over time.
- Plot beacon RSSI/range profile.

### Stage 4: Filtering

- Load saved scenarios.
- Implement dead reckoning.
- Implement beacon fixed-covariance EKF.
- Implement beacon adaptive-covariance EKF.
- Implement LiDAR-only EKF.
- Implement LiDAR plus adaptive beacon EKF.
- Save filter outputs to `results/filter_outputs/scenario_XXX/`.

### Stage 5: Evaluation and Output Generation

- Compute metrics for each scenario.
- Compute aggregate metrics across scenarios.
- Export tables.
- Generate final figures.
- Save summary files.

---

## Acceptance Criteria

The project setup is complete when:

- `main.m` runs without modification.
- `generate_dataset.m` creates all maps, truth data, and sensor measurements.
- `run_filters.m` can be rerun without regenerating maps or truth.
- `generate_outputs.m` can be rerun without regenerating maps or filters.
- All source code is MATLAB.
- At least three scenarios are generated.
- At least one large indoor environment is included.
- Many beacon measurements are naturally unavailable due to range limits or attenuation.
- Synthetic truth and measurements are saved.
- At least four filter configurations are compared.
- Scenario-level and aggregate metrics are exported.
- All required figures are saved.
- The final outputs are reproducible from fixed random seeds.
