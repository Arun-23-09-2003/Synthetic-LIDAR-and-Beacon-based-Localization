function d = point_segment_distance(point, a, b)
%POINT_SEGMENT_DISTANCE Euclidean distance from a point to a segment.

point = point(:);
a = a(:);
b = b(:);
ab = b - a;
den = dot(ab, ab);
if den < eps
    d = norm(point - a);
    return;
end

t = dot(point - a, ab) / den;
t = min(max(t, 0), 1);
closest = a + t * ab;
d = norm(point - closest);
end
