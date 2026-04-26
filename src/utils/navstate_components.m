function [theta, p, v] = navstate_components(X)
%NAVSTATE_COMPONENTS Extract heading, position, and velocity from NavState.

theta = atan2(X(2, 1), X(1, 1));
p = X(1:2, 3);
v = X(1:2, 4);
end
