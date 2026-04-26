function save_figure(fig, filename, cfg)
%SAVE_FIGURE Save a figure as a high-resolution PNG.

set(fig, 'Color', 'w');
if isfield(cfg.outputs, 'figure_width') && isfield(cfg.outputs, 'figure_height')
    set(fig, 'Position', [100 100 cfg.outputs.figure_width cfg.outputs.figure_height]);
end

if exist('exportgraphics', 'file') == 2
    exportgraphics(fig, filename, 'Resolution', cfg.outputs.figure_resolution);
else
    print(fig, filename, '-dpng', ['-r' num2str(cfg.outputs.figure_resolution)]);
end
close(fig);
end
