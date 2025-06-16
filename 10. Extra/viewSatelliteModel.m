function viewSatelliteModel()
% Create and display a satellite model in its own figure

figure;
set(gcf, 'Color', 'k'); % Black background
ax = gca;
hold on

% Create satellite at origin
initial_pos = [0; 0; 0];
satellite = createSatelliteModel(ax, initial_pos);

% Set up lighting and appearance
lighting gouraud
light('Position', [1, 1, 1], 'Style', 'infinite', 'Color', 'w');
light('Position', [-1, -1, -1], 'Style', 'infinite', 'Color', [0.3, 0.3, 0.3]);

% Set axis properties
axis equal
grid on
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', 'GridColor', 'w');

% Set reasonable view limits
xlim([-300, 300])
ylim([-300, 300])
zlim([-100, 200])

% Labels and title
title('Satellite Model', 'Color', 'w', 'FontSize', 16)
xlabel('X (km)', 'Color', 'w')
ylabel('Y (km)', 'Color', 'w')
zlabel('Z (km)', 'Color', 'w')

% Set a good viewing angle
view(45, 20)

hold off
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