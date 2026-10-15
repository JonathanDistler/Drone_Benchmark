% Plot local drone trajectories with a shared elapsed-time color scale.
% Set csvPath below, or leave it empty to choose positions.csv interactively.
csvPath = "";
outputPath = ""; % Optional PNG path, e.g. "drone_trajectories.png".

if strlength(csvPath) == 0
    scriptDir = fileparts(mfilename('fullpath'));
    [file, folder] = uigetfile('*.csv', 'Select positions.csv', ...
        fullfile(scriptDir, '..', 'results', '*.csv'));
    if isequal(file, 0)
        return
    end
    csvPath = string(fullfile(folder, file));
end

opts = detectImportOptions(csvPath, 'TextType', 'string');
required = {'received_unix_ns', 'topic', 'drone_id', ...
    'north_m', 'east_m', 'down_m'};
if ~all(ismember(required, opts.VariableNames))
    error('DronePlot:Schema', 'Select a positions.csv produced by the ROS recorder.');
end
% Subtract integer timestamps before converting to seconds to preserve precision.
opts = setvartype(opts, 'received_unix_ns', 'uint64');
opts = setvartype(opts, 'topic', 'string');
opts = setvartype(opts, {'drone_id', 'north_m', 'east_m', 'down_m'}, 'double');
data = readtable(csvPath, opts);
validId = isfinite(data.drone_id) & data.drone_id >= 0 & ...
    data.drone_id == fix(data.drone_id);
data = data(validId & data.received_unix_ns > 0, :);

% Prefer each drone's own state; use the fleet array only when state is absent.
selected = false(height(data), 1);
for id = unique(data.drone_id)'
    ownState = data.drone_id == id & ...
        endsWith(data.topic, "/drone_" + string(id) + "/state");
    fleetState = data.drone_id == id & data.topic == "/swarm/positions_ned";
    if any(ownState)
        sources = unique(data.topic(ownState));
        if numel(sources) > 1
            error('DronePlot:Sources', ...
                'Drone %g has multiple state topics; filter the CSV to one source.', id);
        end
        selected = selected | ownState;
    else
        selected = selected | fleetState;
    end
end
data = data(selected, :);
finitePosition = isfinite(data.north_m) & isfinite(data.east_m) & isfinite(data.down_m);
if ~any(finitePosition)
    error('DronePlot:Empty', 'No finite drone state or fleet positions found.');
end

ids = unique(data.drone_id(finitePosition));
data = sortrows(data, 'received_unix_ns');
elapsed = double(data.received_unix_ns - min(data.received_unix_ns)) / 1e9;
colorLimits = [0, max(elapsed)];
if colorLimits(2) == 0
    colorLimits(2) = 1;
end
columns = ceil(sqrt(numel(ids)));
fig = figure('Name', 'Drone positions over time', 'Color', 'w');
layout = tiledlayout(fig, ceil(numel(ids) / columns), columns, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, 'Drone trajectories in individual local frames');
colormap(fig, parula(256));

for id = ids'
    ax = nexttile(layout);
    hold(ax, 'on');
    rows = data.drone_id == id;
    north = data.north_m(rows);
    east = data.east_m(rows);
    up = -data.down_m(rows);
    seconds = elapsed(rows);
    valid = isfinite(north) & isfinite(east) & isfinite(up);
    % NaNs break the line where peer positions were unavailable.
    north(~valid) = NaN;
    east(~valid) = NaN;
    up(~valid) = NaN;
    if numel(seconds) > 1
        surface(ax, [north'; north'], [east'; east'], [up'; up'], ...
            [seconds'; seconds'], 'FaceColor', 'none', ...
            'EdgeColor', 'interp', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
    scatter3(ax, north(valid), east(valid), up(valid), 16, seconds(valid), ...
        'filled', 'HandleVisibility', 'off');
    first = find(valid, 1, 'first');
    last = find(valid, 1, 'last');
    plot3(ax, north(first), east(first), up(first), 'ko', ...
        'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Start');
    plot3(ax, north(last), east(last), up(last), 'kx', ...
        'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'End');
    title(ax, sprintf('Drone %g', id));
    xlabel(ax, 'North (m)');
    ylabel(ax, 'East (m)');
    zlabel(ax, 'Up (m)');
    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 3);
    caxis(ax, colorLimits);
    legend(ax, 'Location', 'best');
end
bar = colorbar(ax);
bar.Layout.Tile = 'east';
bar.Label.String = 'Elapsed receipt time (s)';

if strlength(outputPath) > 0
    exportgraphics(fig, outputPath, 'Resolution', 200);
end
