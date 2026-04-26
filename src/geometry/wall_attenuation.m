function [attenuation, crossed_labels] = wall_attenuation(map, p1, p2)
%WALL_ATTENUATION Simulator-side hidden attenuation between two points.

attenuation = 0;
crossed_labels = {};

for i = 1:numel(map.walls)
    wall = map.walls(i);
    q1 = [wall.x1; wall.y1];
    q2 = [wall.x2; wall.y2];
    [hit, ~, t, ~] = segment_intersection(p1, p2, q1, q2);
    if hit && t > 1e-6 && t < 1 - 1e-6
        attenuation = attenuation + wall.thickness * wall.material_factor;
        crossed_labels{end+1} = wall.label; %#ok<AGROW>
    end
end
end
