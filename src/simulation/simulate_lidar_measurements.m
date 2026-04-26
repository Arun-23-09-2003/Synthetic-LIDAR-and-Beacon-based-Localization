function [lidar_scan, lidar_pose] = simulate_lidar_measurements(cfg, scenario_cfg, map, truth)
%SIMULATE_LIDAR_MEASUREMENTS Ray-cast scans and synthetic pose updates.

scan_stride = max(1, round(cfg.motion.prediction_rate_hz / cfg.lidar.scan_rate_hz));
scan_step_index = 1:scan_stride:numel(truth.t);
scan_t = truth.t(scan_step_index);
num_scans = numel(scan_t);

relative_angles = deg2rad(linspace(-cfg.lidar.fov_deg / 2, cfg.lidar.fov_deg / 2, cfg.lidar.num_beams))';
num_beams = numel(relative_angles);

ranges = cfg.lidar.max_range * ones(num_beams, num_scans);
valid_mask = false(num_beams, num_scans);
quality = zeros(1, num_scans);
hit_points = NaN(2, num_beams, num_scans);

for si = 1:num_scans
    k = scan_step_index(si);
    origin = truth.p(:, k);
    theta = truth.theta(k);
    world_angles = theta + relative_angles;
    [scan_ranges, scan_points, scan_valid] = raycast_map(map, origin, world_angles, cfg.lidar.max_range);

    noisy_ranges = scan_ranges;
    noise = cfg.lidar.range_sigma * randn(size(scan_ranges));
    noisy_ranges(scan_valid) = clamp_value(scan_ranges(scan_valid) + noise(scan_valid), 0.05, cfg.lidar.max_range);
    noisy_ranges(~scan_valid) = cfg.lidar.max_range;

    ranges(:, si) = noisy_ranges;
    valid_mask(:, si) = scan_valid;
    hit_points(:, :, si) = scan_points;
    quality(si) = compute_lidar_quality(noisy_ranges, scan_valid, cfg.lidar.max_range, scenario_cfg.map_type);
end

lidar_scan.t = scan_t;
lidar_scan.step_index = scan_step_index;
lidar_scan.ranges = ranges;
lidar_scan.angles = relative_angles;
lidar_scan.valid_mask = valid_mask;
lidar_scan.quality = quality;
lidar_scan.hit_points = hit_points;

pose_stride_scans = max(1, round(cfg.lidar.scan_rate_hz / cfg.lidar.pose_update_rate_hz));
pose_scan_index = 1:pose_stride_scans:num_scans;
pose_step_index = scan_step_index(pose_scan_index);
pose_t = truth.t(pose_step_index);
num_pose = numel(pose_t);

pose = NaN(3, num_pose);
R_pose = NaN(3, 3, num_pose);
pose_quality = quality(pose_scan_index);
pose_available = false(1, num_pose);

for pi = 1:num_pose
    k = pose_step_index(pi);
    q = pose_quality(pi);
    sigma_xy = cfg.lidar.pose_sigma_xy_base + cfg.lidar.pose_sigma_xy_gain * (1 - q)^2;
    sigma_theta = cfg.lidar.pose_sigma_theta_base + cfg.lidar.pose_sigma_theta_gain * (1 - q)^2;
    Rk = diag([sigma_xy^2, sigma_xy^2, sigma_theta^2]);
    R_pose(:, :, pi) = Rk;

    enough_returns = sum(valid_mask(:, pose_scan_index(pi))) >= cfg.lidar.min_valid_returns;
    dropout = rand() < cfg.lidar.random_dropout_probability * scenario_cfg.dropout_scale;
    is_available = q >= cfg.lidar.min_quality_for_pose && enough_returns && ~dropout;
    pose_available(pi) = is_available;

    if is_available
        noise = [sigma_xy * randn(2, 1); sigma_theta * randn()];
        pose(:, pi) = [truth.p(:, k); truth.theta(k)] + noise;
        pose(3, pi) = wrap_pi(pose(3, pi));
    end
end

lidar_pose.t = pose_t;
lidar_pose.step_index = pose_step_index;
lidar_pose.pose = pose;
lidar_pose.R = R_pose;
lidar_pose.quality = pose_quality;
lidar_pose.available = pose_available;
end

function q = compute_lidar_quality(ranges, valid_mask, max_range, map_type)
valid_fraction = mean(valid_mask);
valid_ranges = ranges(valid_mask);
if isempty(valid_ranges)
    q = 0;
    return;
end

range_spread = min(std(valid_ranges) / max_range, 1);
near_wall_fraction = mean(valid_ranges < 0.85 * max_range);
q = 0.10 + 0.50 * valid_fraction + 0.25 * range_spread + 0.15 * near_wall_fraction;

if strcmp(map_type, 'open_hallway') && range_spread < 0.16
    q = q - 0.18;
end

q = clamp_value(q, 0.05, 0.98);
end
