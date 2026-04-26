function crossings = trajectory_wall_crossings(map, p)
%TRAJECTORY_WALL_CROSSINGS Return all trajectory segment/wall intersections.

crossings = struct('step', {}, 'wall_index', {}, 'wall_label', {}, ...
    'point', {}, 'segment_fraction', {});

for k = 1:(size(p, 2) - 1)
    p1 = p(:, k);
    p2 = p(:, k+1);
    if norm(p2 - p1) < 1e-10
        continue;
    end

    for w = 1:numel(map.walls)
        wall = map.walls(w);
        [hit, point, t, ~] = segment_intersection(p1, p2, ...
            [wall.x1; wall.y1], [wall.x2; wall.y2]);
        if hit && t > 1e-8 && t < 1 - 1e-8
            item.step = k;
            item.wall_index = w;
            item.wall_label = wall.label;
            item.point = point;
            item.segment_fraction = t;
            crossings(end+1) = item; %#ok<AGROW>
        end
    end
end
end
