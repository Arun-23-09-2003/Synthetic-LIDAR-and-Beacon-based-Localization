function beacon_measurements = simulate_beacon_measurements(cfg, scenario_cfg, map, beacons, truth)
%SIMULATE_BEACON_MEASUREMENTS Generate range/RSSI/dropout data.

rate = cfg.beacon_defaults.rate_hz;
stride = max(1, round(cfg.motion.prediction_rate_hz / rate));
step_index = 1:stride:numel(truth.t);
t = truth.t(step_index);

num_beacons = numel(beacons);
num_times = numel(t);
range_meas = NaN(num_beacons, num_times);
rssi = NaN(num_beacons, num_times);
available = false(num_beacons, num_times);

diagnostics.true_range = NaN(num_beacons, num_times);
diagnostics.sim_sigma = NaN(num_beacons, num_times);
diagnostics.attenuation = NaN(num_beacons, num_times);
diagnostics.rejection_reason = strings(num_beacons, num_times);

for ti = 1:num_times
    k = step_index(ti);
    robot_p = truth.p(:, k);

    for bi = 1:num_beacons
        beacon = beacons(bi);
        d = norm(robot_p - beacon.position);
        [attenuation, ~] = wall_attenuation(map, robot_p, beacon.position);
        diagnostics.true_range(bi, ti) = d;
        diagnostics.attenuation(bi, ti) = attenuation;

        if d > beacon.max_range
            diagnostics.rejection_reason(bi, ti) = "range";
            continue;
        end
        if attenuation > beacon.penetration_budget
            diagnostics.rejection_reason(bi, ti) = "wall";
            continue;
        end

        rssi_clean = beacon.rssi_ref ...
            - cfg.beacon_defaults.alpha_distance * d ...
            - cfg.beacon_defaults.alpha_wall * attenuation;
        rssi_noisy = rssi_clean + beacon.rssi_sigma * randn();

        if rssi_noisy < beacon.rssi_threshold
            rssi(bi, ti) = rssi_noisy;
            diagnostics.rejection_reason(bi, ti) = "rssi";
            continue;
        end

        sigma = beacon.range_sigma_base ...
            + beacon.range_sigma_gain * d ...
            + beacon.range_sigma_rssi_gain * max(0, beacon.rssi_ref - rssi_noisy);
        diagnostics.sim_sigma(bi, ti) = sigma;

        dropout_probability = cfg.beacon_defaults.random_dropout_probability * scenario_cfg.dropout_scale;
        if rand() < dropout_probability
            rssi(bi, ti) = rssi_noisy;
            diagnostics.rejection_reason(bi, ti) = "random";
            continue;
        end

        range_meas(bi, ti) = d + sigma * randn();
        rssi(bi, ti) = rssi_noisy;
        available(bi, ti) = true;
        diagnostics.rejection_reason(bi, ti) = "available";
    end
end

beacon_measurements.t = t;
beacon_measurements.step_index = step_index;
beacon_measurements.beacon_id = [beacons.id];
beacon_measurements.range_meas = range_meas;
beacon_measurements.rssi = rssi;
beacon_measurements.available = available;
beacon_measurements.diagnostics = diagnostics;
beacon_measurements.robot_side_fields = {'timestamp', 'beacon_id', 'range_meas', 'rssi', 'available'};
end
