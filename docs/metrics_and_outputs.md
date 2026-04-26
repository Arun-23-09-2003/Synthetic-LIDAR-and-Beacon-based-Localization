# Metrics and Outputs

Metric and table generation is implemented in `src/metrics/` and called by `generate_outputs.m`.

## Main Functions

- `compute_filter_metrics.m`: computes position, heading, and velocity errors plus RMSE.
- `compute_sensor_availability.m`: computes beacon and LiDAR availability statistics.
- `collect_results_tables.m`: loads saved results and builds all required tables.
- `generate_outputs.m`: writes tables, summary `.mat` files, and figures.

## Inputs

`generate_outputs.m` reads from:

- `data/generated/`
- `results/filter_outputs/`

It does not rerun the simulator or filters.

## Outputs

Tables are written to `results/tables/`:

- `table_rmse_summary.csv`
- `table_sensor_availability.csv`
- `table_filter_configs.csv`
- `table_aggregate_metrics.csv`

Summary data is written to:

- `results/summary/metrics_summary.mat`
- `results/summary/figure_source_tables.mat`

## Metrics

The code computes:

- position RMSE
- heading RMSE
- velocity RMSE
- maximum position error
- median position error
- beacon availability percentage
- average available beacons per update
- LiDAR pose availability percentage

## Important Assumptions

Metrics are computed against saved ground truth. Aggregate metrics are computed across all configured scenarios.
