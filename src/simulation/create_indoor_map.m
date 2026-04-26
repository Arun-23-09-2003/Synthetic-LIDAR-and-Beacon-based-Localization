function map = create_indoor_map(scenario_cfg)
%CREATE_INDOOR_MAP Create wall-segment floor plans for synthetic scenarios.

walls = empty_wall_array();
bounds = scenario_cfg.bounds;
xmin = bounds(1); xmax = bounds(2); ymin = bounds(3); ymax = bounds(4);

walls = add_wall(walls, xmin, ymin, xmax, ymin, 0.35, 2.4, 'outer boundary');
walls = add_wall(walls, xmax, ymin, xmax, ymax, 0.35, 2.4, 'outer boundary');
walls = add_wall(walls, xmax, ymax, xmin, ymax, 0.35, 2.4, 'outer boundary');
walls = add_wall(walls, xmin, ymax, xmin, ymin, 0.35, 2.4, 'outer boundary');

room_labels = struct('label', {}, 'position', {});

switch scenario_cfg.map_type
    case 'open_hallway'
        walls = add_wall(walls, 0, 7, 5.5, 7, 0.16, 1.4, 'hall wall');
        walls = add_wall(walls, 7.5, 7, 18.0, 7, 0.16, 1.4, 'hall wall');
        walls = add_wall(walls, 20.0, 7, 32.0, 7, 0.16, 1.4, 'hall wall');
        walls = add_wall(walls, 0, 11, 9.0, 11, 0.16, 1.4, 'hall wall');
        walls = add_wall(walls, 11.0, 11, 24.0, 11, 0.16, 1.4, 'hall wall');
        walls = add_wall(walls, 26.0, 11, 32.0, 11, 0.16, 1.4, 'hall wall');
        walls = add_wall(walls, 6, 0, 6, 7, 0.08, 1.0, 'thin partition');
        walls = add_wall(walls, 18, 0, 18, 7, 0.10, 1.1, 'partition');
        walls = add_wall(walls, 25, 11, 25, 18, 0.08, 1.0, 'thin partition');
        walls = add_wall(walls, 12, 11, 12, 18, 0.10, 1.1, 'partition');
        room_labels = make_room_labels({'south rooms','main hallway','north rooms'}, [8 3; 16 9; 19 15]);

    case 'multi_room'
        walls = add_wall(walls, 10, 0, 10, 7.8, 0.18, 1.5, 'main interior wall');
        walls = add_wall(walls, 10, 10.2, 10, 20, 0.18, 1.5, 'main interior wall');
        walls = add_wall(walls, 20, 0, 20, 8.2, 0.18, 1.5, 'main interior wall');
        walls = add_wall(walls, 20, 11.0, 20, 20, 0.18, 1.5, 'main interior wall');
        walls = add_wall(walls, 0, 6.5, 8.0, 6.5, 0.12, 1.2, 'partition');
        walls = add_wall(walls, 12.0, 6.5, 18.5, 6.5, 0.12, 1.2, 'partition');
        walls = add_wall(walls, 21.5, 6.5, 30, 6.5, 0.12, 1.2, 'partition');
        walls = add_wall(walls, 0, 13.5, 8.0, 13.5, 0.12, 1.2, 'partition');
        walls = add_wall(walls, 12.0, 13.5, 18.0, 13.5, 0.12, 1.2, 'partition');
        walls = add_wall(walls, 22.0, 13.5, 30, 13.5, 0.12, 1.2, 'partition');
        walls = add_wall(walls, 15, 6.5, 15, 13.5, 0.05, 0.8, 'glass divider');
        room_labels = make_room_labels({'office A','office B','office C','north hall'}, [5 3; 15 4; 25 4; 16 17]);

    case 'large_dense'
        walls = add_wall(walls, 8, 0, 8, 9.0, 0.16, 1.4, 'main interior wall');
        walls = add_wall(walls, 8, 12.0, 8, 18.6, 0.16, 1.4, 'main interior wall');
        walls = add_wall(walls, 8, 21.4, 8, 26, 0.16, 1.4, 'main interior wall');
        walls = add_wall(walls, 16, 0, 16, 5.5, 0.12, 1.2, 'partition');
        walls = add_wall(walls, 16, 8.5, 16, 19.0, 0.12, 1.2, 'partition');
        walls = add_wall(walls, 16, 21.5, 16, 26, 0.12, 1.2, 'partition');
        walls = add_wall(walls, 25, 0, 25, 11.0, 0.18, 1.5, 'main interior wall');
        walls = add_wall(walls, 25, 14.0, 25, 26, 0.18, 1.5, 'main interior wall');
        walls = add_wall(walls, 34, 0, 34, 7.5, 0.12, 1.2, 'partition');
        walls = add_wall(walls, 34, 10.5, 34, 26, 0.12, 1.2, 'partition');
        walls = add_wall(walls, 0, 7, 6.0, 7, 0.10, 1.0, 'thin partition');
        walls = add_wall(walls, 10.0, 7, 23.0, 7, 0.10, 1.0, 'thin partition');
        walls = add_wall(walls, 27.0, 7, 42, 7, 0.10, 1.0, 'thin partition');
        walls = add_wall(walls, 0, 13, 12.0, 13, 0.16, 1.4, 'main interior wall');
        walls = add_wall(walls, 18.0, 13, 31.0, 13, 0.16, 1.4, 'main interior wall');
        walls = add_wall(walls, 36.0, 13, 42, 13, 0.16, 1.4, 'main interior wall');
        walls = add_wall(walls, 0, 20, 6.0, 20, 0.10, 1.0, 'thin partition');
        walls = add_wall(walls, 9.5, 20, 22.0, 20, 0.10, 1.0, 'thin partition');
        walls = add_wall(walls, 28.0, 20, 42, 20, 0.10, 1.0, 'thin partition');
        walls = add_wall(walls, 30, 7, 30, 13, 0.05, 0.8, 'glass divider');
        room_labels = make_room_labels({'west lab','central hall','east rooms','north wing'}, [5 5; 20 12; 36 6; 24 22]);

    otherwise
        error('Unknown map type: %s', scenario_cfg.map_type);
end

map.walls = walls;
map.bounds = bounds;
map.wall_thickness_values = unique([walls.thickness]);
map.room_labels = room_labels;
map.name = scenario_cfg.name;
end

function walls = empty_wall_array()
walls = struct('x1', {}, 'y1', {}, 'x2', {}, 'y2', {}, ...
    'thickness', {}, 'material_factor', {}, 'label', {});
end

function walls = add_wall(walls, x1, y1, x2, y2, thickness, material_factor, label)
wall.x1 = x1;
wall.y1 = y1;
wall.x2 = x2;
wall.y2 = y2;
wall.thickness = thickness;
wall.material_factor = material_factor;
wall.label = label;
walls(end+1) = wall;
end

function room_labels = make_room_labels(labels, positions)
room_labels = struct('label', {}, 'position', {});
for i = 1:numel(labels)
    room_labels(i).label = labels{i};
    room_labels(i).position = positions(i, :)';
end
end
