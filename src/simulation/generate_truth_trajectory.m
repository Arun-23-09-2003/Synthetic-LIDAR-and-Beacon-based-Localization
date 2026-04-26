function truth = generate_truth_trajectory(cfg, scenario_cfg, map)
%GENERATE_TRUTH_TRAJECTORY Generate smooth ground-truth NavState trajectory.

if nargin < 3 || isempty(map)
    map = create_indoor_map(scenario_cfg);
end

dt = cfg.motion.dt;
t = 0:dt:scenario_cfg.duration_s;
requested_waypoints = scenario_cfg.trajectory_waypoints;
waypoints = plan_wall_avoiding_path(map, requested_waypoints, 0.25, 0.12);

segment_lengths = sqrt(sum(diff(waypoints, 1, 2).^2, 1));
valid_segments = segment_lengths > 1e-8;
waypoints = waypoints(:, [true, valid_segments]);
segment_lengths = sqrt(sum(diff(waypoints, 1, 2).^2, 1));
cumulative_distance = [0, cumsum(segment_lengths)];
total_distance = cumulative_distance(end);
waypoint_indices = round(cumulative_distance / total_distance * (numel(t) - 1)) + 1;
waypoint_indices(1) = 1;
waypoint_indices(end) = numel(t);
[waypoint_indices, keep_waypoints] = unique_waypoint_indices(waypoint_indices);
waypoints = waypoints(:, keep_waypoints);
waypoint_times = t(waypoint_indices);

p = zeros(2, numel(t));
for dim = 1:2
    p(dim, :) = interp1(waypoint_times, waypoints(dim, :), t, 'linear');
end

bounds = scenario_cfg.bounds;
p(1, :) = clamp_value(p(1, :), bounds(1) + 0.6, bounds(2) - 0.6);
p(2, :) = clamp_value(p(2, :), bounds(3) + 0.6, bounds(4) - 0.6);

v = zeros(size(p));
v(1, :) = gradient(p(1, :), dt);
v(2, :) = gradient(p(2, :), dt);
speed = sqrt(sum(v.^2, 1));
theta = atan2(v(2, :), v(1, :));

for k = 2:numel(theta)
    if speed(k) < 1e-4
        theta(k) = theta(k-1);
    end
end
theta = unwrap(theta);
theta = smooth_vector(theta, 9);
theta = wrap_pi(theta);

X = zeros(4, 4, numel(t));
for k = 1:numel(t)
    X(:, :, k) = make_navstate(theta(k), p(:, k), v(:, k));
end

crossings = trajectory_wall_crossings(map, p);
if ~isempty(crossings)
    first = crossings(1);
    error('Generated trajectory intersects wall %d (%s) at step %d.', ...
        first.wall_index, first.wall_label, first.step);
end

truth.t = t;
truth.theta = theta;
truth.p = p;
truth.v = v;
truth.X = X;
truth.u_true = [];
truth.u_noisy = [];
truth.requested_waypoints = requested_waypoints;
truth.waypoints = waypoints;
end

function [indices, keep] = unique_waypoint_indices(indices_in)
keep = true(1, numel(indices_in));
last_index = indices_in(1);
for i = 2:numel(indices_in)
    if indices_in(i) <= last_index
        keep(i) = false;
    else
        last_index = indices_in(i);
    end
end

if ~keep(end)
    keep(end) = true;
end

indices = indices_in(keep);
indices(end) = indices_in(end);
end

function y = smooth_vector(x, window_size)
if window_size <= 1
    y = x;
    return;
end
kernel = ones(1, window_size) / window_size;
pad = floor(window_size / 2);
x_pad = [repmat(x(1), 1, pad), x, repmat(x(end), 1, pad)];
y = conv(x_pad, kernel, 'valid');
y = y(1:numel(x));
end
