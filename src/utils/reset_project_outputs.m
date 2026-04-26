function reset_project_outputs(cfg)
%RESET_PROJECT_OUTPUTS Remove generated artifacts and recreate folders.

targets = {cfg.paths.generated_data, cfg.paths.filter_outputs, ...
    cfg.paths.figures, cfg.paths.tables, cfg.paths.summary};

for i = 1:numel(targets)
    target = targets{i};
    if exist(target, 'dir')
        rmdir(target, 's');
    end
    ensure_dir(target);
end
end
