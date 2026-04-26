function y = clamp_value(x, lower_bound, upper_bound)
%CLAMP_VALUE Clamp values between scalar bounds.

y = min(max(x, lower_bound), upper_bound);
end
