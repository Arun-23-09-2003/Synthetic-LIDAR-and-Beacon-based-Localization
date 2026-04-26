function availability = compute_sensor_availability(beacon_measurements, lidar_pose)
%COMPUTE_SENSOR_AVAILABILITY Summarize beacon and LiDAR availability.

beacon_available = beacon_measurements.available;
availability.beacon_measurement_availability_pct = 100 * nnz(beacon_available) / numel(beacon_available);
availability.avg_available_beacons_per_update = mean(sum(beacon_available, 1));
availability.no_beacon_update_pct = 100 * mean(sum(beacon_available, 1) == 0);
availability.one_beacon_update_pct = 100 * mean(sum(beacon_available, 1) == 1);
availability.two_beacon_update_pct = 100 * mean(sum(beacon_available, 1) == 2);
availability.three_or_more_beacon_update_pct = 100 * mean(sum(beacon_available, 1) >= 3);
availability.lidar_pose_availability_pct = 100 * nnz(lidar_pose.available) / numel(lidar_pose.available);
availability.avg_lidar_quality = mean(lidar_pose.quality);
end
