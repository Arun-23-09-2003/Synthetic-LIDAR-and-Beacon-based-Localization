function cfg = load_config(project_root)
%LOAD_CONFIG Build the project configuration structure.

if nargin < 1 || isempty(project_root)
    project_root = fileparts(fileparts(mfilename('fullpath')));
end

addpath(project_root);
addpath(genpath(fullfile(project_root, 'config')));
addpath(genpath(fullfile(project_root, 'src')));

cfg = struct();
cfg.project.title = 'Adaptive Error-State Kalman Filtering for Indoor Robot Localization';
cfg.paths.root = project_root;
cfg.paths.generated_data = fullfile(project_root, 'data', 'generated');
cfg.paths.results = fullfile(project_root, 'results');
cfg.paths.filter_outputs = fullfile(project_root, 'results', 'filter_outputs');
cfg.paths.figures = fullfile(project_root, 'results', 'figures');
cfg.paths.tables = fullfile(project_root, 'results', 'tables');
cfg.paths.summary = fullfile(project_root, 'results', 'summary');

cfg.random = config_random();
cfg.motion = config_motion();
cfg.beacon_defaults = config_beacons();
cfg.lidar = config_lidar();
cfg.filters = config_filters();
cfg.outputs = config_outputs();
cfg.scenarios = config_maps(cfg);
cfg.num_scenarios = numel(cfg.scenarios);
end
