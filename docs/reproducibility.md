# Reproducibility

The project is designed to regenerate all synthetic data and outputs from fixed seeds.

## Entry Points

Run the complete pipeline:

```matlab
main
```

Run modular steps:

```matlab
generate_dataset
run_filters
generate_outputs
```

## Seeds

`config/config_random.m` defines:

```matlab
cfg.random.global_seed = 6505;
cfg.random.scenario_seeds = [650501, 650502, 650503, 650504, 650505];
```

Each scenario sets its own seed before generation. The scenario seed and relevant config values are saved in `scenario_config.mat`.

## Reset Behavior

`main.m` calls `reset_project_outputs(cfg)`, which removes and recreates:

- `data/generated/`
- `results/filter_outputs/`
- `results/figures/`
- `results/tables/`
- `results/summary/`

The modular scripts overwrite their own outputs but do not delete unrelated folders.

## Verification

After `main` runs, the repository should contain:

- three saved scenarios
- five saved filter outputs per scenario
- all required figures
- all required tables
- zero wall intersections for each saved sampled truth trajectory

`run_filters` can be rerun without regenerating datasets. `generate_outputs` can be rerun without regenerating datasets or rerunning filters.
