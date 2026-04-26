# Beacon Model

Beacon simulation is implemented in `src/simulation/create_beacons.m` and `src/simulation/simulate_beacon_measurements.m`.

## Main Functions

- `create_beacons.m`: creates fixed beacons from scenario positions and beacon defaults.
- `simulate_beacon_measurements.m`: applies range limits, hidden wall attenuation, RSSI thresholds, random dropout, and noisy range generation.
- `compute_sensor_availability.m`: summarizes beacon availability for output tables.

## Inputs

- Beacon positions, max ranges, and penetration budgets from `config_maps.m`.
- RSSI and noise parameters from `config_beacons.m`.
- `map` and `truth` from dataset generation.

## Outputs

The saved `beacon_measurements` structure includes:

- `t`
- `step_index`
- `beacon_id`
- `range_meas`
- `rssi`
- `available`
- `diagnostics`
- `robot_side_fields`

The robot/filter side uses only timestamp, beacon ID, measured range, RSSI, and availability.

## Noise and Availability

The simulator rejects beacon measurements when:

- true range exceeds `beacon.max_range`
- hidden attenuation exceeds `beacon.penetration_budget`
- noisy RSSI falls below `beacon.rssi_threshold`
- random dropout occurs

The adaptive filter covariance uses measured range and RSSI:

```matlab
sigma = sigma_base + sigma_gain * z + sigma_rssi_gain * max(0, rssi_ref - rssi)
```

## Important Assumptions

The filter knows beacon positions and beacon noise parameters, but it does not use true range, true attenuation, wall counts, or wall thickness crossings.
