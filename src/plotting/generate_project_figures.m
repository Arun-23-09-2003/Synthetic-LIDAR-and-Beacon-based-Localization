function generate_project_figures(cfg, rmse_table, availability_table, aggregate_table)
%GENERATE_PROJECT_FIGURES Create all required figures from saved outputs.

for scenario_idx = 1:cfg.num_scenarios
    scenario_name = scenario_folder_name(scenario_idx);
    scenario_fig_dir = fullfile(cfg.paths.figures, scenario_name);
    ensure_dir(scenario_fig_dir);

    scenario_cfg = cfg;
    scenario_cfg.paths.figures = scenario_fig_dir;

    data = load_plot_data(cfg, scenario_idx);
    case_results = load_plot_results(cfg, scenario_idx);
    generate_scenario_figure_set(data, case_results, scenario_cfg, ...
        aggregate_table, availability_table);
end

copy_default_scenario_figures(cfg);

save(fullfile(cfg.paths.summary, 'figure_source_tables.mat'), ...
    'rmse_table', 'availability_table', 'aggregate_table');
end

function generate_scenario_figure_set(data, case_results, cfg, aggregate_table, availability_table)
plot_map_wall_thickness(data, cfg);
plot_beacon_layout(data, cfg);
plot_true_trajectory(data, cfg);
plot_motion_profile(data, cfg);
plot_lidar_scan_example(data, cfg);
plot_lidar_quality(data, cfg);
plot_beacon_availability(data, cfg);
plot_beacon_signal_profile(data, cfg);
plot_beacon_noise_model(data, cfg);
plot_measurement_timeline(data, cfg);
plot_trajectory_comparison(data, case_results, cfg);
plot_position_error(case_results, cfg);
plot_heading_error(case_results, cfg);
plot_velocity_error(case_results, cfg);
plot_covariance_consistency(data, case_results, cfg);
plot_rmse_comparison(cfg, aggregate_table);
plot_result_summary(data, case_results, cfg, aggregate_table, availability_table);
end

function copy_default_scenario_figures(cfg)
default_name = scenario_folder_name(cfg.outputs.default_scenario_for_figures);
source_dir = fullfile(cfg.paths.figures, default_name);
files = dir(fullfile(source_dir, '*.png'));
for i = 1:numel(files)
    copyfile(fullfile(source_dir, files(i).name), ...
        fullfile(cfg.paths.figures, files(i).name));
end
end

function data = load_plot_data(cfg, scenario_idx)
scenario_name = scenario_folder_name(scenario_idx);
scenario_dir = fullfile(cfg.paths.generated_data, scenario_name);
data.scenario_name = scenario_name;
data.map = load_var(fullfile(scenario_dir, 'map.mat'), 'map');
data.beacons = load_var(fullfile(scenario_dir, 'beacons.mat'), 'beacons');
data.truth = load_var(fullfile(scenario_dir, 'truth.mat'), 'truth');
data.odom = load_var(fullfile(scenario_dir, 'prediction_inputs.mat'), 'odom');
data.beacon_measurements = load_var(fullfile(scenario_dir, 'beacon_measurements.mat'), 'beacon_measurements');
data.lidar_scan = load_var(fullfile(scenario_dir, 'lidar_scans.mat'), 'lidar_scan');
data.lidar_pose = load_var(fullfile(scenario_dir, 'lidar_pose_measurements.mat'), 'lidar_pose');
data.scenario_config = load_var(fullfile(scenario_dir, 'scenario_config.mat'), 'scenario_config');
end

function case_results = load_plot_results(cfg, scenario_idx)
scenario_name = scenario_folder_name(scenario_idx);
result_dir = fullfile(cfg.paths.filter_outputs, scenario_name);
case_results = struct('case_cfg', {}, 'estimated_states', {}, ...
    'state_covariances', {}, 'metrics', {}, 'measurement_usage_history', {});
for i = 1:numel(cfg.filters.cases)
    case_cfg = cfg.filters.cases(i);
    result = load(fullfile(result_dir, [case_cfg.id '.mat']));
    case_results(i).case_cfg = case_cfg;
    case_results(i).estimated_states = result.estimated_states;
    case_results(i).state_covariances = result.state_covariances;
    case_results(i).metrics = result.metrics;
    case_results(i).measurement_usage_history = result.measurement_usage_history;
end
end

function value = load_var(filename, variable_name)
data = load(filename);
value = data.(variable_name);
end

function plot_map_wall_thickness(data, cfg)
fig = make_fig(cfg);
axes('Parent', fig);
hold on;
plot_walls(data.map, true);
label_wall_thickness(data.map);
format_map_axes(data.map, 'Wall thickness map');
save_figure(fig, fullfile(cfg.paths.figures, 'fig_map_wall_thickness.png'), cfg);
end

function plot_beacon_layout(data, cfg)
fig = make_fig(cfg);
axes('Parent', fig);
hold on;
plot_walls(data.map, false);
for i = 1:numel(data.beacons)
    b = data.beacons(i);
    draw_circle(b.position, b.max_range, [0.1 0.45 0.85], 0.35);
    plot(b.position(1), b.position(2), 'p', 'MarkerSize', 13, ...
        'MarkerFaceColor', [0.95 0.62 0.15], 'MarkerEdgeColor', [0.2 0.2 0.2]);
    text(b.position(1) + 0.25, b.position(2) + 0.25, sprintf('B%d', b.id), ...
        'FontSize', cfg.outputs.font_size, 'FontWeight', 'bold');
end
format_map_axes(data.map, 'Beacon placement and nominal coverage');
save_figure(fig, fullfile(cfg.paths.figures, 'fig_beacon_layout.png'), cfg);
end

function plot_true_trajectory(data, cfg)
fig = make_fig(cfg);
axes('Parent', fig);
hold on;
plot_walls(data.map, false);
h_truth = plot(data.truth.p(1, :), data.truth.p(2, :), ...
    'LineWidth', 2.0, 'Color', [0.0 0.35 0.75]);
h_start = plot(data.truth.p(1, 1), data.truth.p(2, 1), 'o', ...
    'MarkerFaceColor', [0.15 0.60 0.20], 'MarkerEdgeColor', [0.05 0.25 0.08], 'MarkerSize', 8);
h_end = plot(data.truth.p(1, end), data.truth.p(2, end), 's', ...
    'MarkerFaceColor', [0.82 0.20 0.16], 'MarkerEdgeColor', [0.35 0.05 0.04], 'MarkerSize', 8);
idx = 1:round(numel(data.truth.t) / 18):numel(data.truth.t);
quiver(data.truth.p(1, idx), data.truth.p(2, idx), cos(data.truth.theta(idx)), ...
    sin(data.truth.theta(idx)), 0.55, 'Color', [0.1 0.1 0.1], ...
    'LineWidth', 0.9, 'HandleVisibility', 'off');
legend([h_truth, h_start, h_end], {'Truth', 'Start', 'End'}, 'Location', 'bestoutside');
format_map_axes(data.map, 'True trajectory');
save_figure(fig, fullfile(cfg.paths.figures, 'fig_true_trajectory.png'), cfg);
end

function plot_motion_profile(data, cfg)
fig = make_fig(cfg);
t = data.truth.t;
subplot(3, 1, 1);
plot(t, data.odom.u_true.accel_body(1, :), 'LineWidth', 1.3); hold on;
plot(t, data.odom.accel_body(1, :), 'LineWidth', 0.8);
grid on; ylabel('a_x body (m/s^2)'); legend({'True','Noisy'}, 'Location', 'best');
title('Prediction input motion profile');

subplot(3, 1, 2);
plot(t, data.odom.u_true.accel_body(2, :), 'LineWidth', 1.3); hold on;
plot(t, data.odom.accel_body(2, :), 'LineWidth', 0.8);
grid on; ylabel('a_y body (m/s^2)');

subplot(3, 1, 3);
plot(t, data.odom.u_true.omega_meas, 'LineWidth', 1.3); hold on;
plot(t, data.odom.omega_meas, 'LineWidth', 0.8);
grid on; ylabel('\omega (rad/s)'); xlabel('Time (s)');
save_figure(fig, fullfile(cfg.paths.figures, 'fig_motion_profile.png'), cfg);
end

function plot_lidar_scan_example(data, cfg)
[~, scan_idx] = max(data.lidar_scan.quality);
k = data.lidar_scan.step_index(scan_idx);
pose = [data.truth.p(:, k); data.truth.theta(k)];
angles = pose(3) + data.lidar_scan.angles;
ranges = data.lidar_scan.ranges(:, scan_idx);
valid = data.lidar_scan.valid_mask(:, scan_idx);

fig = make_fig(cfg);
axes('Parent', fig);
hold on;
plot_walls(data.map, false);
beam_stride = max(1, floor(numel(angles) / 45));
for b = 1:beam_stride:numel(angles)
    endpoint = pose(1:2) + ranges(b) * [cos(angles(b)); sin(angles(b))];
    if valid(b)
        color = [0.1 0.55 0.85];
    else
        color = [0.75 0.75 0.75];
    end
    plot([pose(1), endpoint(1)], [pose(2), endpoint(2)], '-', 'Color', color, 'LineWidth', 0.6);
end

hit_points = data.lidar_scan.hit_points(:, :, scan_idx);
plot(hit_points(1, valid), hit_points(2, valid), '.', 'Color', [0.9 0.2 0.1], 'MarkerSize', 9);
plot(pose(1), pose(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
quiver(pose(1), pose(2), cos(pose(3)), sin(pose(3)), 0.9, 'k', 'LineWidth', 1.2);
format_map_axes(data.map, 'Example LiDAR ray-cast scan');
save_figure(fig, fullfile(cfg.paths.figures, 'fig_lidar_scan_example.png'), cfg);
end

function plot_lidar_quality(data, cfg)
fig = make_fig(cfg);
subplot(2, 1, 1);
plot(data.lidar_scan.t, data.lidar_scan.quality, 'LineWidth', 1.5);
grid on; ylim([0 1]); ylabel('Scan quality');
title('LiDAR measurement quality');

subplot(2, 1, 2);
sigma_xy = squeeze(sqrt(data.lidar_pose.R(1, 1, :)));
h_cov = plot(data.lidar_pose.t, sigma_xy, 'LineWidth', 1.4); hold on;
h_available = stem(data.lidar_pose.t(data.lidar_pose.available), ...
    sigma_xy(data.lidar_pose.available), 'filled', 'MarkerSize', 3);
grid on; ylabel('\sigma_x, \sigma_y (m)'); xlabel('Time (s)');
legend([h_cov, h_available], {'Pose covariance', 'Available pose update'}, 'Location', 'best');
save_figure(fig, fullfile(cfg.paths.figures, 'fig_lidar_quality.png'), cfg);
end

function plot_beacon_availability(data, cfg)
fig = make_fig(cfg);
imagesc(data.beacon_measurements.t, data.beacon_measurements.beacon_id, ...
    data.beacon_measurements.available);
set(gca, 'YDir', 'normal');
colormap(gca, [0.92 0.92 0.92; 0.08 0.42 0.78]);
colorbar('Ticks', [0.25 0.75], 'TickLabels', {'Dropped','Available'});
xlabel('Time (s)'); ylabel('Beacon ID'); title('Beacon availability over time');
save_figure(fig, fullfile(cfg.paths.figures, 'fig_beacon_availability.png'), cfg);
end

function plot_beacon_signal_profile(data, cfg)
fig = make_fig(cfg);
ids = 1:min(3, numel(data.beacons));
t = data.beacon_measurements.t;

subplot(2, 1, 1);
hold on;
for id = ids
    plot(t, data.beacon_measurements.diagnostics.true_range(id, :), '--', 'LineWidth', 1.0);
    plot(t, data.beacon_measurements.range_meas(id, :), '.', 'MarkerSize', 8);
end
grid on; ylabel('Range (m)'); title('Beacon range and RSSI examples');

subplot(2, 1, 2);
hold on;
for id = ids
    plot(t, data.beacon_measurements.rssi(id, :), 'LineWidth', 1.1);
end
grid on; ylabel('RSSI (dB)'); xlabel('Time (s)');
legend(compose('B%d', ids), 'Location', 'best');
save_figure(fig, fullfile(cfg.paths.figures, 'fig_beacon_signal_profile.png'), cfg);
end

function plot_beacon_noise_model(data, cfg)
fig = make_fig(cfg);
hold on;
all_ranges = [];
all_sigmas = [];
all_rssi = [];
for bi = 1:numel(data.beacons)
    available = data.beacon_measurements.available(bi, :);
    z = data.beacon_measurements.range_meas(bi, available);
    rssi = data.beacon_measurements.rssi(bi, available);
    b = data.beacons(bi);
    sigma = b.range_sigma_base + b.range_sigma_gain * max(z, 0) ...
        + b.range_sigma_rssi_gain * max(0, b.rssi_ref - rssi);
    all_ranges = [all_ranges, z]; %#ok<AGROW>
    all_sigmas = [all_sigmas, sigma]; %#ok<AGROW>
    all_rssi = [all_rssi, rssi]; %#ok<AGROW>
end
scatter(all_ranges, all_sigmas, 28, all_rssi, 'filled');
colorbar; xlabel('Measured range (m)'); ylabel('Adaptive \sigma_r (m)');
title('Robot-side beacon covariance model');
grid on;
save_figure(fig, fullfile(cfg.paths.figures, 'fig_beacon_noise_model.png'), cfg);
end

function plot_measurement_timeline(data, cfg)
fig = make_fig(cfg);
hold on;
t = data.truth.t;
plot(t, ones(size(t)), '.', 'Color', [0.65 0.65 0.65], 'MarkerSize', 3);
beacon_any = any(data.beacon_measurements.available, 1);
plot(data.beacon_measurements.t(beacon_any), 2 * ones(1, nnz(beacon_any)), ...
    's', 'MarkerFaceColor', [0.1 0.45 0.85], 'MarkerEdgeColor', 'none');
plot(data.lidar_pose.t(data.lidar_pose.available), ...
    3 * ones(1, nnz(data.lidar_pose.available)), 'o', ...
    'MarkerFaceColor', [0.9 0.3 0.15], 'MarkerEdgeColor', 'none');
ylim([0.5 3.5]); yticks([1 2 3]); yticklabels({'Odometry','Beacon','LiDAR pose'});
grid on; xlabel('Time (s)'); title('Multi-rate measurement timeline');
save_figure(fig, fullfile(cfg.paths.figures, 'fig_measurement_timeline.png'), cfg);
end

function plot_trajectory_comparison(data, case_results, cfg)
fig = make_fig(cfg);
axes('Parent', fig);
hold on;
plot_walls(data.map, false);
line_handles = gobjects(1, numel(case_results) + 1);
line_handles(1) = plot(data.truth.p(1, :), data.truth.p(2, :), ...
    'k-', 'LineWidth', 2.2);
colors = filter_case_colors(numel(case_results));
for i = 1:numel(case_results)
    est = case_results(i).estimated_states;
    line_handles(i+1) = plot(est.p_hat(1, :), est.p_hat(2, :), ...
        'LineWidth', 1.4, 'Color', colors(i, :));
end
legend_labels = cell(1, numel(case_results) + 1);
legend_labels{1} = 'Truth';
for i = 1:numel(case_results)
    legend_labels{i+1} = case_results(i).case_cfg.name;
end
legend(line_handles, legend_labels, 'Location', 'bestoutside');
format_map_axes(data.map, 'Trajectory comparison');
save_figure(fig, fullfile(cfg.paths.figures, 'fig_trajectory_comparison.png'), cfg);
end

function plot_position_error(case_results, cfg)
fig = make_fig(cfg);
hold on;
colors = filter_case_colors(numel(case_results));
h_lines = gobjects(1, numel(case_results));
for i = 1:numel(case_results)
    h_lines(i) = plot(case_results(i).estimated_states.t, case_results(i).metrics.position_error, ...
        'LineWidth', 1.3, 'Color', colors(i, :));
end
grid on; xlabel('Time (s)'); ylabel('Position error (m)');
legend_case_names(case_results, h_lines);
title('Position error over time');
save_figure(fig, fullfile(cfg.paths.figures, 'fig_position_error.png'), cfg);
end

function plot_heading_error(case_results, cfg)
fig = make_fig(cfg);
hold on;
colors = filter_case_colors(numel(case_results));
h_lines = gobjects(1, numel(case_results));
for i = 1:numel(case_results)
    h_lines(i) = plot(case_results(i).estimated_states.t, rad2deg(abs(case_results(i).metrics.heading_error)), ...
        'LineWidth', 1.3, 'Color', colors(i, :));
end
grid on; xlabel('Time (s)'); ylabel('Absolute heading error (deg)');
legend_case_names(case_results, h_lines);
title('Heading error over time');
save_figure(fig, fullfile(cfg.paths.figures, 'fig_heading_error.png'), cfg);
end

function plot_velocity_error(case_results, cfg)
fig = make_fig(cfg);
hold on;
colors = filter_case_colors(numel(case_results));
h_lines = gobjects(1, numel(case_results));
for i = 1:numel(case_results)
    h_lines(i) = plot(case_results(i).estimated_states.t, case_results(i).metrics.velocity_error, ...
        'LineWidth', 1.3, 'Color', colors(i, :));
end
grid on; xlabel('Time (s)'); ylabel('Velocity error (m/s)');
legend_case_names(case_results, h_lines);
title('Velocity error over time');
save_figure(fig, fullfile(cfg.paths.figures, 'fig_velocity_error.png'), cfg);
end

function plot_covariance_consistency(data, case_results, cfg)
case_ids = cell(1, numel(case_results));
for i = 1:numel(case_results)
    case_ids{i} = case_results(i).case_cfg.id;
end
fused_idx = find(strcmp(case_ids, 'lidar_beacon_adaptive'), 1);
if isempty(fused_idx)
    fused_idx = numel(case_results);
end
result = case_results(fused_idx);
P = result.state_covariances.P;
t = result.estimated_states.t;
pos_sigma = zeros(1, numel(t));
heading_sigma = zeros(1, numel(t));
for k = 1:numel(t)
    pos_sigma(k) = sqrt(P(2, 2, k) + P(3, 3, k));
    heading_sigma(k) = sqrt(P(1, 1, k));
end

fig = make_fig(cfg);
subplot(2, 1, 1);
plot(t, result.metrics.position_error, 'LineWidth', 1.3); hold on;
plot(t, 2 * pos_sigma, '--', 'LineWidth', 1.2);
grid on; ylabel('Position (m)');
legend({'Error','2 sigma bound'}, 'Location', 'best');
title(['Covariance consistency: ' result.case_cfg.name]);

subplot(2, 1, 2);
plot(t, rad2deg(abs(result.metrics.heading_error)), 'LineWidth', 1.3); hold on;
plot(t, rad2deg(2 * heading_sigma), '--', 'LineWidth', 1.2);
grid on; ylabel('Heading (deg)'); xlabel('Time (s)');
save_figure(fig, fullfile(cfg.paths.figures, 'fig_covariance_consistency.png'), cfg);
end

function plot_rmse_comparison(cfg, aggregate_table)
fig = make_fig(cfg);
x = 1:height(aggregate_table);
bar(x, aggregate_table.mean_position_rmse_m);
set(gca, 'XTick', x, 'XTickLabel', aggregate_table.case_name);
xtickangle(25);
ylabel('Mean position RMSE (m)');
title('Aggregate RMSE comparison');
grid on;
save_figure(fig, fullfile(cfg.paths.figures, 'fig_rmse_comparison.png'), cfg);
end

function plot_result_summary(data, case_results, cfg, aggregate_table, availability_table)
fig = make_fig(cfg);

subplot(2, 2, 1);
hold on;
plot_walls(data.map, false);
plot(data.truth.p(1, :), data.truth.p(2, :), 'k-', 'LineWidth', 1.8);
fused_idx = numel(case_results);
plot(case_results(fused_idx).estimated_states.p_hat(1, :), ...
    case_results(fused_idx).estimated_states.p_hat(2, :), ...
    'Color', [0.0 0.45 0.75], 'LineWidth', 1.3);
format_map_axes(data.map, 'Fused trajectory');

subplot(2, 2, 2);
hold on;
colors = filter_case_colors(numel(case_results));
for i = 1:numel(case_results)
    plot(case_results(i).estimated_states.t, case_results(i).metrics.position_error, ...
        'LineWidth', 1.0, 'Color', colors(i, :));
end
grid on; xlabel('Time (s)'); ylabel('Position error (m)');
title('Error history');

subplot(2, 2, 3);
bar(aggregate_table.mean_position_rmse_m);
set(gca, 'XTick', 1:height(aggregate_table), 'XTickLabel', aggregate_table.case_id);
xtickangle(25); ylabel('RMSE (m)'); grid on; title('Aggregate position RMSE');

subplot(2, 2, 4);
scenario_row = strcmp(availability_table.scenario, data.scenario_name);
values = [availability_table.no_beacon_update_pct(scenario_row), ...
    availability_table.one_beacon_update_pct(scenario_row), ...
    availability_table.two_beacon_update_pct(scenario_row), ...
    availability_table.three_or_more_beacon_update_pct(scenario_row)];
bar(values);
set(gca, 'XTickLabel', {'0','1','2','3+'});
ylabel('Beacon update share (%)'); xlabel('Available beacons'); grid on;
title('Beacon geometry diversity');

save_figure(fig, fullfile(cfg.paths.figures, 'fig_result_summary.png'), cfg);
end

function fig = make_fig(cfg)
fig = figure('Visible', 'off', 'Color', 'w');
set(fig, 'Position', [100 100 cfg.outputs.figure_width cfg.outputs.figure_height]);
set(0, 'DefaultAxesFontSize', cfg.outputs.font_size);
set(0, 'DefaultLineLineWidth', cfg.outputs.line_width);
end

function plot_walls(map, color_by_thickness)
if nargin < 2
    color_by_thickness = false;
end
thicknesses = [map.walls.thickness];
if color_by_thickness
    cmap = parula(256);
    min_th = min(thicknesses);
    max_th = max(thicknesses);
end
for i = 1:numel(map.walls)
    wall = map.walls(i);
    if color_by_thickness
        idx = 1 + round(255 * (wall.thickness - min_th) / max(max_th - min_th, eps));
        color = cmap(idx, :);
    else
        color = [0.12 0.12 0.12];
    end
    plot([wall.x1 wall.x2], [wall.y1 wall.y2], '-', ...
        'Color', color, 'LineWidth', 2.0 + 9.0 * wall.thickness, ...
        'HandleVisibility', 'off');
end
if color_by_thickness
    colormap(gca, cmap);
    cb = colorbar;
    cb.Label.String = 'Wall thickness (m)';
    caxis([min_th max_th]);
end
end

function label_wall_thickness(map)
for i = 1:numel(map.walls)
    wall = map.walls(i);
    if wall.thickness >= 0.10
        mx = 0.5 * (wall.x1 + wall.x2);
        my = 0.5 * (wall.y1 + wall.y2);
        text(mx, my, sprintf('%.2f', wall.thickness), ...
            'FontSize', 8, 'HorizontalAlignment', 'center', ...
            'BackgroundColor', 'w', 'Margin', 1);
    end
end
end

function format_map_axes(map, plot_title)
bounds = map.bounds;
xlim([bounds(1) bounds(2)]);
ylim([bounds(3) bounds(4)]);
axis equal;
grid on;
xlabel('x (m)');
ylabel('y (m)');
title(plot_title);
end

function draw_circle(center, radius, color, alpha_value)
theta = linspace(0, 2*pi, 160);
x = center(1) + radius * cos(theta);
y = center(2) + radius * sin(theta);
patch(x, y, color, 'FaceAlpha', 0.08, 'EdgeColor', color, ...
    'EdgeAlpha', alpha_value, 'LineStyle', '--', 'HandleVisibility', 'off');
end

function legend_case_names(case_results, handles)
names = cell(1, numel(case_results));
for i = 1:numel(case_results)
    names{i} = case_results(i).case_cfg.name;
end
legend(handles, names, 'Location', 'best');
end

function colors = filter_case_colors(num_cases)
palette = [
    0.35 0.35 0.35
    0.00 0.45 0.74
    0.20 0.58 0.20
    0.85 0.33 0.10
    0.49 0.18 0.56
    0.30 0.75 0.93
    0.64 0.08 0.18];

if num_cases <= size(palette, 1)
    colors = palette(1:num_cases, :);
else
    colors = lines(num_cases);
end
end
