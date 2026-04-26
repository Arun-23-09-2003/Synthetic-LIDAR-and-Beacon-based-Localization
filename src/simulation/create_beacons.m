function beacons = create_beacons(cfg, scenario_cfg)
%CREATE_BEACONS Create fixed beacon definitions for a scenario.

positions = scenario_cfg.beacon_positions;
num_beacons = size(positions, 2);
defaults = cfg.beacon_defaults;

beacons = struct('id', {}, 'position', {}, 'max_range', {}, ...
    'penetration_budget', {}, 'tx_power', {}, 'rssi_ref', {}, ...
    'rssi_threshold', {}, 'path_loss_exponent', {}, ...
    'range_sigma_base', {}, 'range_sigma_gain', {}, ...
    'range_sigma_rssi_gain', {}, 'rssi_sigma', {});

for i = 1:num_beacons
    beacon.id = i;
    beacon.position = positions(:, i);
    beacon.max_range = scenario_cfg.beacon_max_ranges(i);
    beacon.penetration_budget = scenario_cfg.beacon_penetration_budgets(i);
    beacon.tx_power = defaults.tx_power;
    beacon.rssi_ref = defaults.rssi_ref + 2.0 * sin(0.7 * i);
    beacon.rssi_threshold = defaults.rssi_threshold;
    beacon.path_loss_exponent = defaults.path_loss_exponent;
    beacon.range_sigma_base = defaults.range_sigma_base;
    beacon.range_sigma_gain = defaults.range_sigma_gain;
    beacon.range_sigma_rssi_gain = defaults.range_sigma_rssi_gain;
    beacon.rssi_sigma = defaults.rssi_sigma;
    beacons(end+1) = beacon; %#ok<AGROW>
end
end
