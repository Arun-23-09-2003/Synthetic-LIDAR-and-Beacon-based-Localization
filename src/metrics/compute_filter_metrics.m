function metrics = compute_filter_metrics(truth, estimated_states)
%COMPUTE_FILTER_METRICS Compute trajectory errors and RMSE metrics.

position_error_vector = truth.p - estimated_states.p_hat;
position_error = sqrt(sum(position_error_vector.^2, 1));
heading_error = wrap_pi(truth.theta - estimated_states.theta_hat);
velocity_error_vector = truth.v - estimated_states.v_hat;
velocity_error = sqrt(sum(velocity_error_vector.^2, 1));

metrics.position_error = position_error;
metrics.heading_error = heading_error;
metrics.velocity_error = velocity_error;
metrics.position_error_vector = position_error_vector;
metrics.velocity_error_vector = velocity_error_vector;
metrics.rmse_position = sqrt(mean(position_error.^2));
metrics.rmse_heading = sqrt(mean(heading_error.^2));
metrics.rmse_velocity = sqrt(mean(velocity_error.^2));
metrics.max_position_error = max(position_error);
metrics.median_position_error = median(position_error);
end
