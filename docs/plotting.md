# Plotting

Plotting is implemented in `src/plotting/generate_project_figures.m`.

## Main Function

`generate_project_figures(cfg, rmse_table, availability_table, aggregate_table)` generates the full figure set for every configured scenario and uses aggregate tables for summary comparisons.

## Inputs

- Saved scenario data from each `data/generated/scenario_XXX/` folder.
- Saved filter outputs from each `results/filter_outputs/scenario_XXX/` folder.
- RMSE, availability, and aggregate metric tables created by `collect_results_tables.m`.

## Outputs

Figures are written to `results/figures/` with the exact required filenames:

- `fig_map_wall_thickness.png`
- `fig_beacon_layout.png`
- `fig_true_trajectory.png`
- `fig_motion_profile.png`
- `fig_lidar_scan_example.png`
- `fig_lidar_quality.png`
- `fig_beacon_availability.png`
- `fig_beacon_signal_profile.png`
- `fig_beacon_noise_model.png`
- `fig_measurement_timeline.png`
- `fig_trajectory_comparison.png`
- `fig_position_error.png`
- `fig_heading_error.png`
- `fig_velocity_error.png`
- `fig_covariance_consistency.png`
- `fig_rmse_comparison.png`
- `fig_result_summary.png`

The same filenames are also written per scenario:

- `results/figures/scenario_001/`
- `results/figures/scenario_002/`
- `results/figures/scenario_003/`

The root `results/figures/*.png` files are copied from the configured default scenario so the original README figure contract remains valid.

## Figure Export

`src/utils/save_figure.m` uses `exportgraphics` when available and falls back to `print`. Figures are saved at the configured resolution from `config_outputs.m`.

## Important Assumptions

Legends use explicit line handles so wall geometry does not appear as thick legend blocks. Tables and RMSE comparison figures include all scenarios.
