clear; clc; close all;

project_root = setup_project_paths();
cfg = load_config(project_root);
rng(cfg.random.global_seed);

reset_project_outputs(cfg);
generate_dataset(cfg);
run_filters(cfg);
generate_outputs(cfg);

disp('AE6505 synthetic localization project regenerated successfully.');
