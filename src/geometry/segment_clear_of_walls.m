function is_clear = segment_clear_of_walls(map, p1, p2, clearance)
%SEGMENT_CLEAR_OF_WALLS True when a path segment does not cross or skim walls.

if nargin < 4
    clearance = 0.05;
end

is_clear = true;
p1 = p1(:);
p2 = p2(:);

for w = 1:numel(map.walls)
    wall = map.walls(w);
    q1 = [wall.x1; wall.y1];
    q2 = [wall.x2; wall.y2];
    [hit, ~, t, ~] = segment_intersection(p1, p2, q1, q2);
    if hit && t > 1e-8 && t < 1 - 1e-8
        is_clear = false;
        return;
    end
end

segment_length = norm(p2 - p1);
num_samples = max(2, ceil(segment_length / max(clearance, 0.03)));
for i = 1:num_samples
    alpha = (i - 1) / (num_samples - 1);
    sample = (1 - alpha) * p1 + alpha * p2;
    for w = 1:numel(map.walls)
        wall = map.walls(w);
        d = point_segment_distance(sample, [wall.x1; wall.y1], [wall.x2; wall.y2]);
        if d < clearance
            is_clear = false;
            return;
        end
    end
end
end
