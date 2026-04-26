function beacon_cfg = config_beacons()
%CONFIG_BEACONS Default beacon simulation and filter-side parameters.

beacon_cfg.rate_hz = 10;
beacon_cfg.alpha_distance = 3.0;        % RSSI loss per meter
beacon_cfg.alpha_wall = 34.0;           % RSSI loss per attenuation unit
beacon_cfg.default_max_range = 9.0;
beacon_cfg.default_penetration_budget = 0.30;
beacon_cfg.tx_power = 0.0;
beacon_cfg.rssi_ref = -42.0;
beacon_cfg.rssi_threshold = -82.0;
beacon_cfg.path_loss_exponent = 2.0;
beacon_cfg.range_sigma_base = 0.12;
beacon_cfg.range_sigma_gain = 0.025;
beacon_cfg.range_sigma_rssi_gain = 0.006;
beacon_cfg.rssi_sigma = 2.2;
beacon_cfg.random_dropout_probability = 0.04;
beacon_cfg.fixed_filter_sigma = 0.55;
end
