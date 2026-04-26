# Project Structure

This repository follows the structure required by `README.md`.

## Top-Level Scripts

- `main.m`: full reproducibility entry point. It resets generated outputs, generates datasets, runs filters, exports tables, and creates figures.
- `generate_dataset.m`: generates and saves synthetic scenarios only.
- `run_filters.m`: loads saved scenario data and runs all configured filter cases.
- `generate_outputs.m`: loads saved data and filter outputs, then writes tables, summaries, and figures.
- `setup_project_paths.m`: adds only `config/`, `src/`, and the project root to the MATLAB path.

## Source Folders

- `config/`: parameter definitions for random seeds, maps, motion, beacons, LiDAR, filters, and output settings.
- `src/geometry/`: wall intersection, attenuation, and ray-casting helpers.
- `src/simulation/`: scenario generation, map creation, beacons, truth trajectories, prediction inputs, beacon measurements, and LiDAR measurements.
- `src/filters/`: NavState propagation and error-state EKF measurement updates.
- `src/metrics/`: trajectory metrics, sensor availability metrics, and table assembly.
- `src/plotting/`: automatic figure generation.
- `src/utils/`: common helpers, NavState utilities, wrapping, folder creation, and figure export.

## Generated Outputs

- `data/generated/scenario_XXX/`: saved map, beacons, truth, odometry, beacon measurements, LiDAR scans, LiDAR pose measurements, and scenario config.
- `results/filter_outputs/scenario_XXX/`: one `.mat` file per filter case.
- `results/figures/`: required PNG figures.
- `results/tables/`: required CSV tables.
- `results/summary/`: compact `.mat` summaries used for reproducibility and inspection.

## Assumptions

The repository contains only MATLAB source code for simulation, filtering, metrics, and plotting. Generated `.mat`, `.csv`, and `.png` artifacts are reproducible and may be overwritten by `main`.
