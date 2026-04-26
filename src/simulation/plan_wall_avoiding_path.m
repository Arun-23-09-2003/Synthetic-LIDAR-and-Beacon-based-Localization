function path = plan_wall_avoiding_path(map, requested_waypoints, resolution, clearance)
%PLAN_WALL_AVOIDING_PATH Route requested waypoints through wall openings.

if nargin < 3 || isempty(resolution)
    resolution = 0.25;
end
if nargin < 4 || isempty(clearance)
    clearance = 0.12;
end

grid = build_occupancy_grid(map, resolution, clearance);
path = [];

for i = 1:(size(requested_waypoints, 2) - 1)
    start_rc = nearest_free_cell(grid, requested_waypoints(:, i));
    goal_rc = nearest_free_cell(grid, requested_waypoints(:, i+1));
    cell_path = bfs_grid_path(grid, map, start_rc, goal_rc);
    segment_path = cells_to_points(grid, cell_path);
    segment_path = shortcut_path(map, segment_path, clearance);

    if isempty(path)
        path = segment_path;
    else
        path = [path, segment_path(:, 2:end)]; %#ok<AGROW>
    end
end

path = remove_nearly_duplicate_points(path, resolution * 0.25);
end

function grid = build_occupancy_grid(map, resolution, clearance)
bounds = map.bounds;
grid.x = (bounds(1) + resolution/2):resolution:(bounds(2) - resolution/2);
grid.y = (bounds(3) + resolution/2):resolution:(bounds(4) - resolution/2);
grid.resolution = resolution;
grid.clearance = clearance;
grid.occupied = false(numel(grid.y), numel(grid.x));

for r = 1:numel(grid.y)
    for c = 1:numel(grid.x)
        point = [grid.x(c); grid.y(r)];
        for w = 1:numel(map.walls)
            wall = map.walls(w);
            d = point_segment_distance(point, [wall.x1; wall.y1], [wall.x2; wall.y2]);
            if d <= clearance
                grid.occupied(r, c) = true;
                break;
            end
        end
    end
end
end

function rc = nearest_free_cell(grid, point)
[~, c0] = min(abs(grid.x - point(1)));
[~, r0] = min(abs(grid.y - point(2)));

if is_free(grid, r0, c0)
    rc = [r0, c0];
    return;
end

best_dist = inf;
best_rc = [];
for r = 1:numel(grid.y)
    for c = 1:numel(grid.x)
        if ~grid.occupied(r, c)
            d = hypot(grid.x(c) - point(1), grid.y(r) - point(2));
            if d < best_dist
                best_dist = d;
                best_rc = [r, c];
            end
        end
    end
end

if isempty(best_rc)
    error('No free grid cell exists for trajectory planning.');
end
rc = best_rc;
end

function cell_path = bfs_grid_path(grid, map, start_rc, goal_rc)
num_rows = numel(grid.y);
num_cols = numel(grid.x);
num_cells = num_rows * num_cols;
start_idx = sub2ind([num_rows, num_cols], start_rc(1), start_rc(2));
goal_idx = sub2ind([num_rows, num_cols], goal_rc(1), goal_rc(2));

visited = false(num_rows, num_cols);
parent = zeros(num_rows, num_cols);
queue = zeros(num_cells, 1);
head = 1;
tail = 1;
queue(tail) = start_idx;
visited(start_rc(1), start_rc(2)) = true;

neighbor_offsets = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];

while head <= tail
    current_idx = queue(head);
    head = head + 1;
    if current_idx == goal_idx
        break;
    end

    [r, c] = ind2sub([num_rows, num_cols], current_idx);
    for n = 1:size(neighbor_offsets, 1)
        nr = r + neighbor_offsets(n, 1);
        nc = c + neighbor_offsets(n, 2);
        if ~is_free(grid, nr, nc) || visited(nr, nc)
            continue;
        end
        if abs(nr - r) == 1 && abs(nc - c) == 1
            if ~is_free(grid, r, nc) || ~is_free(grid, nr, c)
                continue;
            end
        end
        current_point = [grid.x(c); grid.y(r)];
        neighbor_point = [grid.x(nc); grid.y(nr)];
        if ~segment_clear_of_walls(map, current_point, neighbor_point, grid.clearance)
            continue;
        end

        visited(nr, nc) = true;
        parent(nr, nc) = current_idx;
        tail = tail + 1;
        queue(tail) = sub2ind([num_rows, num_cols], nr, nc);
    end
end

if ~visited(goal_rc(1), goal_rc(2))
    error('No wall-avoiding grid path found between requested waypoints.');
end

path_indices = goal_idx;
current_idx = goal_idx;
while current_idx ~= start_idx
    [r, c] = ind2sub([num_rows, num_cols], current_idx);
    current_idx = parent(r, c);
    path_indices(end+1) = current_idx; %#ok<AGROW>
end
path_indices = fliplr(path_indices);

cell_path = zeros(numel(path_indices), 2);
for i = 1:numel(path_indices)
    [r, c] = ind2sub([num_rows, num_cols], path_indices(i));
    cell_path(i, :) = [r, c];
end
end

function tf = is_free(grid, r, c)
tf = r >= 1 && r <= numel(grid.y) && c >= 1 && c <= numel(grid.x) ...
    && ~grid.occupied(r, c);
end

function points = cells_to_points(grid, cell_path)
points = zeros(2, size(cell_path, 1));
for i = 1:size(cell_path, 1)
    r = cell_path(i, 1);
    c = cell_path(i, 2);
    points(:, i) = [grid.x(c); grid.y(r)];
end
end

function path = shortcut_path(map, path_in, clearance)
path = path_in(:, 1);
idx = 1;
while idx < size(path_in, 2)
    next_idx = [];
    for candidate = size(path_in, 2):-1:(idx + 1)
        if segment_clear_of_walls(map, path_in(:, idx), path_in(:, candidate), clearance)
            next_idx = candidate;
            break;
        end
    end
    if isempty(next_idx)
        error('Wall-avoiding path contains an invalid adjacent segment.');
    end
    path(:, end+1) = path_in(:, next_idx); %#ok<AGROW>
    idx = next_idx;
end
end

function path = remove_nearly_duplicate_points(path_in, min_distance)
keep = true(1, size(path_in, 2));
last = 1;
for i = 2:size(path_in, 2)
    if norm(path_in(:, i) - path_in(:, last)) < min_distance
        keep(i) = false;
    else
        last = i;
    end
end
path = path_in(:, keep);
end
