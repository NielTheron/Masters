function AnimatedSatelliteOrbit(x_true, dt)
% AnimatedSatelliteOrbit - Creates an animated 3D visualization of satellite orbit
%
% Inputs:
%   x_true - State vector array [13 x n_samples] containing position data
%   dt     - Time step between samples (optional, default = 0.1)
%
% The function shows:
% - Earth with texture mapping
% - Animated satellite model
% - Real-time orbital path tracking
% - Orbital information display

if nargin < 2
    dt = 0.1; % Default time step
end

% Earth parameters
R_earth = 6371; % Earth radius in km

% Create figure and setup
fig = figure('Name', 'Animated Satellite Orbit', 'Color', 'k', ...
    'Position', [100, 100, 1200, 800]);
ax = axes('Parent', fig, 'Color', 'k');
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
set(ax, 'GridColor', 'w', 'GridAlpha', 0.3);

% Create Earth
[X, Y, Z] = ellipsoid(0, 0, 0, R_earth, R_earth, R_earth, 50);
earth_surf = surf(ax, X, Y, Z, 'EdgeColor', 'none', 'FaceAlpha', 0.9);

% Try to load Earth texture, use default coloring if not available
try
    earth_img = imread('Earth.jpg');
    set(earth_surf, 'FaceColor', 'texturemap', 'CData', earth_img);
catch
    % Use blue-green coloring if image not available
    set(earth_surf, 'FaceColor', [0.2, 0.5, 0.8]);
end

% Extract position data
pos_data = x_true(1:3, :);
n_points = size(pos_data, 2);

% Calculate orbital parameters for display
altitude = sqrt(sum(pos_data.^2, 1)) - R_earth;
mean_altitude = mean(altitude);
max_altitude = max(altitude);
min_altitude = min(altitude);

% Initialize orbital path
orbit_path = plot3(ax, NaN, NaN, NaN, 'y-', 'LineWidth', 2, ...
    'DisplayName', 'Orbital Path');

% Create satellite model (simple geometric shape) at first position
satellite = createSatelliteModel(ax, pos_data(:,1));

% Initialize current position marker
current_pos = plot3(ax, pos_data(1,1), pos_data(2,1), pos_data(3,1), ...
    'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', ...
    'DisplayName', 'Current Position');

% Setup lighting
lighting(ax, 'gouraud');
material(ax, 'dull');
light('Position', [1, 1, 1], 'Style', 'infinite', 'Color', 'w');

% Set axis properties
axis(ax, 'vis3d');
max_range = max(max(abs(pos_data(:)))) * 1.2;
xlim(ax, [-max_range, max_range]);
ylim(ax, [-max_range, max_range]);
zlim(ax, [-max_range, max_range]);

% Labels and title
xlabel(ax, 'X Position (km)', 'Color', 'w', 'FontSize', 12);
ylabel(ax, 'Y Position (km)', 'Color', 'w', 'FontSize', 12);
zlabel(ax, 'Z Position (km)', 'Color', 'w', 'FontSize', 12);
title(ax, 'Satellite Orbital Animation', 'Color', 'w', 'FontSize', 16);

% Create info text box
info_text = text(ax, 0.02, 0.98, 0, '', 'Units', 'normalized', ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
    'Color', 'w', 'FontSize', 10, 'BackgroundColor', [0, 0, 0, 0.7], ...
    'EdgeColor', 'w', 'Margin', 5);

% Animation parameters
trail_length = min(200, n_points); % Number of points to show in trail
update_interval = max(1, round(0.05 / dt)); % Update every ~50ms

% Animation loop
for i = 1:update_interval:n_points
    % Update satellite position
    current_sat_pos = pos_data(:, i);
    updateSatellitePosition(satellite, current_sat_pos);
    
    % Update current position marker
    set(current_pos, 'XData', current_sat_pos(1), ...
                     'YData', current_sat_pos(2), ...
                     'ZData', current_sat_pos(3));
    
    % Update orbital trail
    trail_start = max(1, i - trail_length);
    trail_x = pos_data(1, trail_start:i);
    trail_y = pos_data(2, trail_start:i);
    trail_z = pos_data(3, trail_start:i);
    
    set(orbit_path, 'XData', trail_x, 'YData', trail_y, 'ZData', trail_z);
    
    % Update info display
    current_time = (i-1) * dt;
    current_alt = altitude(i);
    
    info_str = sprintf(['Time: %.1f s\n' ...
                       'Current Altitude: %.1f km\n' ...
                       'Mean Altitude: %.1f km\n' ...
                       'Max Altitude: %.1f km\n' ...
                       'Min Altitude: %.1f km\n' ...
                       'Points Displayed: %d/%d'], ...
                      current_time, current_alt, mean_altitude, ...
                      max_altitude, min_altitude, i, n_points);
    set(info_text, 'String', info_str);
    
    % Auto-rotate view for better visualization
    view(ax, current_time * 2, 20); % Slow rotation
    
    % Force graphics update
    drawnow;
    
    % Small pause for animation timing
    pause(0.01);
end

% Add legend
legend(ax, [orbit_path, current_pos], 'Location', 'northeast', ...
    'TextColor', 'w', 'EdgeColor', 'w');

% Final message
fprintf('Animation completed!\n');
fprintf('Total simulation time: %.2f seconds\n', (n_points-1) * dt);
fprintf('Mean orbital altitude: %.2f km\n', mean_altitude);

end

function satellite = createSatelliteModel(ax, initial_pos)
% Create a simple satellite model using geometric shapes at specified position

if nargin < 2
    initial_pos = [0; 0; 0]; % Default to origin
end

% Main body (rectangular prism)
[x_body, y_body, z_body] = createRectPrism(50, 50, 100);
body = patch(ax, 'XData', x_body + initial_pos(1), 'YData', y_body + initial_pos(2), 'ZData', z_body + initial_pos(3), ...
    'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', 'k', 'LineWidth', 1);

% Solar panels (flat rectangles)
panel_width = 200;
panel_height = 100;
panel_thickness = 5;

% Left panel
[x_panel1, y_panel1, z_panel1] = createRectPrism(panel_thickness, panel_width, panel_height);
panel1 = patch(ax, 'XData', x_panel1 - 75 + initial_pos(1), 'YData', y_panel1 + initial_pos(2), 'ZData', z_panel1 + initial_pos(3), ...
    'FaceColor', [0.2, 0.2, 0.8], 'EdgeColor', 'k', 'LineWidth', 1);

% Right panel
panel2 = patch(ax, 'XData', x_panel1 + 75 + initial_pos(1), 'YData', y_panel1 + initial_pos(2), 'ZData', z_panel1 + initial_pos(3), ...
    'FaceColor', [0.2, 0.2, 0.8], 'EdgeColor', 'k', 'LineWidth', 1);

% Antenna (simple line)
antenna = plot3(ax, [0, 0] + initial_pos(1), [0, 0] + initial_pos(2), [50, 120] + initial_pos(3), 'r-', 'LineWidth', 3);

% Store initial position for reference
satellite = struct('body', body, 'panel1', panel1, 'panel2', panel2, 'antenna', antenna, ...
                  'initial_pos', initial_pos);
end

function [x, y, z] = createRectPrism(width, height, depth)
% Create vertices for a rectangular prism

w = width/2; h = height/2; d = depth/2;

% Define vertices
vertices = [-w -h -d; w -h -d; w h -d; -w h -d; ...  % bottom face
           -w -h  d; w -h  d; w h  d; -w h  d];      % top face

% Define faces
faces = [1 2 3 4; 5 8 7 6; 1 5 6 2; 2 6 7 3; 3 7 8 4; 4 8 5 1];

x = vertices(faces', 1);
y = vertices(faces', 2);
z = vertices(faces', 3);
end

function updateSatellitePosition(satellite, position)
% Update satellite model position

% Calculate offset from initial position
if isfield(satellite, 'initial_pos')
    offset = position - satellite.initial_pos;
else
    offset = position; % Fallback if no initial position stored
end

% Get component names (excluding initial_pos field)
components = fieldnames(satellite);
components = components(~strcmp(components, 'initial_pos'));

for i = 1:length(components)
    comp = satellite.(components{i});
    
    if strcmp(get(comp, 'Type'), 'patch')
        % For patch objects, get original vertices and apply offset
        x_data_orig = get(comp, 'XData');
        y_data_orig = get(comp, 'YData');
        z_data_orig = get(comp, 'ZData');
        
        % Apply the position offset to move satellite
        if ~isempty(x_data_orig)
            % Remove previous offset and apply new one
            if isfield(satellite, 'last_position')
                last_offset = satellite.last_position - satellite.initial_pos;
                x_data_orig = x_data_orig - last_offset(1);
                y_data_orig = y_data_orig - last_offset(2);
                z_data_orig = z_data_orig - last_offset(3);
            end
            
            % Apply new offset
            x_data = x_data_orig + offset(1);
            y_data = y_data_orig + offset(2);
            z_data = z_data_orig + offset(3);
            
            set(comp, 'XData', x_data, 'YData', y_data, 'ZData', z_data);
        end
        
    elseif strcmp(get(comp, 'Type'), 'line')
        % For line objects (antenna)
        x_data_orig = get(comp, 'XData');
        y_data_orig = get(comp, 'YData');
        z_data_orig = get(comp, 'ZData');
        
        if ~isempty(x_data_orig)
            % Remove previous offset and apply new one
            if isfield(satellite, 'last_position')
                last_offset = satellite.last_position - satellite.initial_pos;
                x_data_orig = x_data_orig - last_offset(1);
                y_data_orig = y_data_orig - last_offset(2);
                z_data_orig = z_data_orig - last_offset(3);
            end
            
            % Apply new offset
            x_data = x_data_orig + offset(1);
            y_data = y_data_orig + offset(2);
            z_data = z_data_orig + offset(3);
            
            set(comp, 'XData', x_data, 'YData', y_data, 'ZData', z_data);
        end
    end
end

% Store current position for next update
satellite.last_position = position;
end

% Example usage function (for testing without your data)
function testAnimation()
% Generate sample orbital data for testing
t = 0:0.1:60; % 60 seconds
R_earth = 6371;
altitude = 500; % 500 km altitude

% Simple circular orbit
R_orbit = R_earth + altitude;
omega = sqrt(398600 / R_orbit^3); % Orbital angular velocity

x_orbit = R_orbit * cos(omega * t);
y_orbit = R_orbit * sin(omega * t);
z_orbit = zeros(size(t)); % Equatorial orbit

% Create state vector (only position needed for this visualization)
x_true_test = [x_orbit; y_orbit; z_orbit; zeros(10, length(t))];

% Run animation
AnimatedSatelliteOrbit(x_true_test, 0.1);
end