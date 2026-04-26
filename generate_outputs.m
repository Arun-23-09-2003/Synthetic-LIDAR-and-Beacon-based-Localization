function generate_outputs(cfg)
%GENERATE_OUTPUTS Export metrics tables, summaries, and figures.

if nargin < 1 || isempty(cfg)
    project_root = setup_project_paths();
    cfg = load_config(project_root);
end

ensure_dir(cfg.paths.figures);
ensure_dir(cfg.paths.tables);
ensure_dir(cfg.paths.summary);

fprintf('Collecting metrics and writing tables\n');
[rmse_table, availability_table, config_table, aggregate_table] = collect_results_tables(cfg);

writetable(rmse_table, fullfile(cfg.paths.tables, 'table_rmse_summary.csv'));
writetable(availability_table, fullfile(cfg.paths.tables, 'table_sensor_availability.csv'));
writetable(config_table, fullfile(cfg.paths.tables, 'table_filter_configs.csv'));
writetable(aggregate_table, fullfile(cfg.paths.tables, 'table_aggregate_metrics.csv'));

save(fullfile(cfg.paths.summary, 'metrics_summary.mat'), ...
    'rmse_table', 'availability_table', 'config_table', 'aggregate_table');

fprintf('Generating figures\n');
generate_project_figures(cfg, rmse_table, availability_table, aggregate_table);
end
