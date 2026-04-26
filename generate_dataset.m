function generate_dataset(cfg)
%GENERATE_DATASET Generate and save all synthetic scenarios.

if nargin < 1 || isempty(cfg)
    project_root = setup_project_paths();
    cfg = load_config(project_root);
end

ensure_dir(cfg.paths.generated_data);

for scenario_idx = 1:cfg.num_scenarios
    scenario_cfg = cfg.scenarios(scenario_idx);
    rng(scenario_cfg.seed);
    scenario_name = scenario_folder_name(scenario_idx);
    scenario_dir = fullfile(cfg.paths.generated_data, scenario_name);
    ensure_dir(scenario_dir);

    fprintf('Generating %s: %s\n', scenario_name, scenario_cfg.name);
    generate_scenario_dataset(cfg, scenario_cfg, scenario_dir);
end
end
