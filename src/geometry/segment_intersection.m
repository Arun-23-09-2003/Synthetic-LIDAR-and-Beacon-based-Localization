function [does_intersect, point, t, u] = segment_intersection(p1, p2, q1, q2)
%SEGMENT_INTERSECTION Intersect two 2D line segments.
%
% p(t) = p1 + t (p2 - p1), q(u) = q1 + u (q2 - q1).

p1 = p1(:);
p2 = p2(:);
q1 = q1(:);
q2 = q2(:);

r = p2 - p1;
s = q2 - q1;
denom = cross2(r, s);

does_intersect = false;
point = [NaN; NaN];
t = NaN;
u = NaN;

if abs(denom) < 1e-10
    return;
end

qmp = q1 - p1;
t = cross2(qmp, s) / denom;
u = cross2(qmp, r) / denom;

tol = 1e-9;
if t >= -tol && t <= 1 + tol && u >= -tol && u <= 1 + tol
    does_intersect = true;
    t = min(max(t, 0), 1);
    u = min(max(u, 0), 1);
    point = p1 + t * r;
end
end

function z = cross2(a, b)
z = a(1) * b(2) - a(2) * b(1);
end
