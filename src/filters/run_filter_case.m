function result = run_filter_case(data, case_cfg, cfg)
%RUN_FILTER_CASE Run one EKF configuration on one saved scenario.

tic_handle = tic;
truth = data.truth;
odom = data.odom;
beacon_measurements = data.beacon_measurements;
lidar_pose = data.lidar_pose;
beacons = data.beacons;

num_steps = numel(truth.t);
X = truth.X(:, :, 1);
P = cfg.filters.initial_covariance;

X_hat = zeros(4, 4, num_steps);
p_hat = zeros(2, num_steps);
v_hat = zeros(2, num_steps);
theta_hat = zeros(1, num_steps);
P_hist = zeros(5, 5, num_steps);

beacon_lookup = zeros(1, num_steps);
beacon_lookup(beacon_measurements.step_index) = 1:numel(beacon_measurements.step_index);
lidar_lookup = zeros(1, num_steps);
lidar_lookup(lidar_pose.step_index) = 1:numel(lidar_pose.step_index);

innovation_history = struct('time', {}, 'sensor', {}, 'id', {}, ...
    'residual', {}, 'nis', {}, 'sigma_used', {});
usage.beacon_updates = zeros(1, num_steps);
usage.lidar_updates = false(1, num_steps);
usage.available_beacons = zeros(1, num_steps);
usage.case_id = case_cfg.id;
usage.case_name = case_cfg.name;

for k = 1:num_steps
    if k > 1
        dt = truth.t(k) - truth.t(k-1);
        input_k.accel_body = odom.accel_body(:, k-1);
        input_k.omega_meas = odom.omega_meas(k-1);
        [X, P] = propagate_navstate(X, P, input_k, dt, cfg);
    end

    beacon_idx = beacon_lookup(k);
    if beacon_idx > 0
        usage.available_beacons(k) = sum(beacon_measurements.available(:, beacon_idx));
        if case_cfg.use_beacon
            [X, P, innovation_history, count] = apply_beacon_updates( ...
                X, P, beacon_measurements, beacon_idx, beacons, case_cfg, cfg, innovation_history);
            usage.beacon_updates(k) = count;
        end
    end

    lidar_idx = lidar_lookup(k);
    if lidar_idx > 0 && case_cfg.use_lidar && lidar_pose.available(lidar_idx)
        [X, P, innovation_history] = apply_lidar_update( ...
            X, P, lidar_pose, lidar_idx, cfg, innovation_history);
        usage.lidar_updates(k) = true;
    end

    [theta, p, v] = navstate_components(X);
    X_hat(:, :, k) = X;
    p_hat(:, k) = p;
    v_hat(:, k) = v;
    theta_hat(k) = theta;
    P_hist(:, :, k) = P;
end

estimated_states.t = truth.t;
estimated_states.X_hat = X_hat;
estimated_states.p_hat = p_hat;
estimated_states.v_hat = v_hat;
estimated_states.theta_hat = theta_hat;

state_covariances.P = P_hist;

metrics = compute_filter_metrics(truth, estimated_states);
runtime_info.case_id = case_cfg.id;
runtime_info.case_name = case_cfg.name;
runtime_info.elapsed_seconds = toc(tic_handle);
runtime_info.generated_from_saved_dataset = true;

result.name = case_cfg.name;
result.estimated_states = estimated_states;
result.state_covariances = state_covariances;
result.innovation_history = innovation_history;
result.measurement_usage_history = usage;
result.runtime_info = runtime_info;
result.metrics = metrics;
end

function [X, P, innovation_history, update_count] = apply_beacon_updates( ...
    X, P, beacon_measurements, beacon_idx, beacons, case_cfg, cfg, innovation_history)

update_count = 0;
for bi = 1:numel(beacons)
    if ~beacon_measurements.available(bi, beacon_idx)
        continue;
    end

    z = beacon_measurements.range_meas(bi, beacon_idx);
    rssi = beacon_measurements.rssi(bi, beacon_idx);
    beacon = beacons(bi);

    [~, p_hat, ~] = navstate_components(X);
    diff_vec = p_hat - beacon.position;
    d_hat = max(norm(diff_vec), cfg.filters.min_range_for_jacobian);
    residual = z - d_hat;
    H = [0, diff_vec(1) / d_hat, diff_vec(2) / d_hat, 0, 0];

    if case_cfg.beacon_adaptive
        sigma = beacon.range_sigma_base ...
            + beacon.range_sigma_gain * max(z, 0) ...
            + beacon.range_sigma_rssi_gain * max(0, beacon.rssi_ref - rssi);
    else
        sigma = cfg.beacon_defaults.fixed_filter_sigma;
    end
    sigma = max(sigma, 0.05);
    R_meas = sigma^2;

    [X, P, ~, nis] = ekf_tangent_update(X, P, residual, H, R_meas, cfg);
    update_count = update_count + 1;
    innovation_history(end+1) = make_innovation( ...
        beacon_measurements.t(beacon_idx), 'beacon', beacon.id, residual, nis, sigma); %#ok<AGROW>
end
end

function [X, P, innovation_history] = apply_lidar_update( ...
    X, P, lidar_pose, lidar_idx, cfg, innovation_history)

z = lidar_pose.pose(:, lidar_idx);
[theta_hat, p_hat, ~] = navstate_components(X);
residual = [z(1:2) - p_hat; wrap_pi(z(3) - theta_hat)];
H = [0 1 0 0 0;
     0 0 1 0 0;
     1 0 0 0 0];
R_meas = lidar_pose.R(:, :, lidar_idx);

[X, P, ~, nis] = ekf_tangent_update(X, P, residual, H, R_meas, cfg);
sigma_used = sqrt(diag(R_meas));
innovation_history(end+1) = make_innovation( ...
    lidar_pose.t(lidar_idx), 'lidar', 0, residual, nis, sigma_used); %#ok<AGROW>
end

function item = make_innovation(time, sensor, id, residual, nis, sigma_used)
item.time = time;
item.sensor = sensor;
item.id = id;
item.residual = residual;
item.nis = nis;
item.sigma_used = sigma_used;
end
