function [ranges, hit_points, valid_mask, hit_wall_index] = raycast_map(map, origin, world_angles, max_range)
%RAYCAST_MAP Cast 2D rays against wall segments.

num_beams = numel(world_angles);
ranges = max_range * ones(num_beams, 1);
hit_points = NaN(2, num_beams);
valid_mask = false(num_beams, 1);
hit_wall_index = zeros(num_beams, 1);

for b = 1:num_beams
    dir_vec = [cos(world_angles(b)); sin(world_angles(b))];
    ray_end = origin(:) + max_range * dir_vec;
    best_range = max_range;
    best_point = [NaN; NaN];
    best_wall = 0;

    for w = 1:numel(map.walls)
        wall = map.walls(w);
        q1 = [wall.x1; wall.y1];
        q2 = [wall.x2; wall.y2];
        [hit, point, t, ~] = segment_intersection(origin, ray_end, q1, q2);
        if hit
            range = t * max_range;
            if range > 1e-5 && range < best_range
                best_range = range;
                best_point = point;
                best_wall = w;
            end
        end
    end

    if best_wall > 0
        ranges(b) = best_range;
        hit_points(:, b) = best_point;
        valid_mask(b) = true;
        hit_wall_index(b) = best_wall;
    end
end
end
