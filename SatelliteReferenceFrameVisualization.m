function SatelliteReferenceFrameVisualization(x_true)
%==========================================================================
% SatelliteReferenceFrameVisualization
%==========================================================================
% Purpose: Visualize satellite with wire globe, position vector, and all 
%          reference frames (ECI, Orbital LVLH, Body)
%
% Inputs:
%   r_i               - Satellite position vector in ECI [3x1] (km)
%   v_i               - Satellite velocity vector in ECI [3x1] (km/s)  
%   q_inertial_to_body - Quaternion from inertial to body frame [4x1] [w,x,y,z]
%
% Frame Definitions:
%   ECI Frame (Green):     X=Vernal Equinox, Y=90°E, Z=North Pole
%   Orbital Frame (Red):   X=Along-track, Y=Cross-track, Z=Nadir (LVLH)
%   Body Frame (Blue):     At t=0 aligned with orbital, then rotated by quaternion
%==========================================================================


if nargin < 3
    % Default test case - circular orbit
    r_i = x_true(1:3,1);           % km
    v_i = x_true(4:6,1);               % km/s
    q_inertial_to_body = x_true(7:10,1); % no rotation
end

% Ensure column vectors
r_i = r_i(:);
v_i = v_i(:);
q_inertial_to_body = q_inertial_to_body(:);

% Constants
EARTH_RADIUS = 6371; % km
FRAME_LENGTH = 1500; % km

%==========================================================================
% Create Figure
%==========================================================================
figure('Name', 'Satellite Reference Frame Visualization', ...
       'Color', 'k', 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8]);
ax = axes('Color', 'k');
hold on;
grid on;
axis equal;
view(3);

% Set axis colors and labels
ax.XColor = 'w'; ax.YColor = 'w'; ax.ZColor = 'w';
ax.GridColor = 'w'; ax.GridAlpha = 0.3;
xlabel('X (km)', 'Color', 'w', 'FontSize', 12);
ylabel('Y (km)', 'Color', 'w', 'FontSize', 12);
zlabel('Z (km)', 'Color', 'w', 'FontSize', 12);
title('Satellite Reference Frame Visualization', 'Color', 'w', 'FontSize', 14);

%==========================================================================
% 1. Create Wire Globe (Earth)
%==========================================================================
[X, Y, Z] = sphere(50);
X = X * EARTH_RADIUS;
Y = Y * EARTH_RADIUS; 
Z = Z * EARTH_RADIUS;

% Wire frame Earth
surf(X, Y, Z, 'FaceColor', 'none', 'EdgeColor', 'white', ...
     'EdgeAlpha', 0.3, 'LineWidth', 0.5);

% Add equatorial plane
theta = linspace(0, 2*pi, 100);
eq_x = EARTH_RADIUS * cos(theta);
eq_y = EARTH_RADIUS * sin(theta);
eq_z = zeros(size(theta));
plot3(eq_x, eq_y, eq_z, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);

% Add prime meridian and 90°E meridian for reference
phi = linspace(-pi/2, pi/2, 50);
% Prime meridian (0°)
mer_x = EARTH_RADIUS * cos(phi);
mer_y = zeros(size(phi));
mer_z = EARTH_RADIUS * sin(phi);
plot3(mer_x, mer_y, mer_z, 'Color', [0.3 0.3 0.3], 'LineWidth', 1);
% 90°E meridian
mer_x_90 = zeros(size(phi));
mer_y_90 = EARTH_RADIUS * cos(phi);
mer_z_90 = EARTH_RADIUS * sin(phi);
plot3(mer_x_90, mer_y_90, mer_z_90, 'Color', [0.3 0.3 0.3], 'LineWidth', 1);

%==========================================================================
% 2. Create ECI Reference Frame (Green) - Fixed at Earth Center
%==========================================================================
eci_colors = [0 1 0; 0 0.7 0; 0 0.5 0]; % Different shades of green for X,Y,Z
eci_labels = {'X_{ECI} (Vernal Equinox)', 'Y_{ECI} (90°E)', 'Z_{ECI} (North Pole)'};

for i = 1:3
    axis_vec = zeros(3,1);
    axis_vec(i) = FRAME_LENGTH;
    
    % Draw axis line
    plot3([0 axis_vec(1)], [0 axis_vec(2)], [0 axis_vec(3)], ...
          'Color', eci_colors(i,:), 'LineWidth', 3);
    
    % Add arrowhead
    drawArrowHead([0;0;0], axis_vec, eci_colors(i,:), FRAME_LENGTH*0.05);
    
    % Add label
    text(axis_vec(1)*1.1, axis_vec(2)*1.1, axis_vec(3)*1.1, ...
         eci_labels{i}, 'Color', eci_colors(i,:), 'FontSize', 10, 'FontWeight', 'bold');
end

%==========================================================================
% 3. Position Vector (Magenta)
%==========================================================================
plot3([0 r_i(1)], [0 r_i(2)], [0 r_i(3)], ...
      'Color', 'm', 'LineWidth', 2, 'LineStyle', '--');
text(r_i(1)*0.5, r_i(2)*0.5, r_i(3)*0.5, 'Position Vector r_i', ...
     'Color', 'm', 'FontSize', 10);

%==========================================================================
% 4. Create Satellite (Yellow)
%==========================================================================
% Simple satellite representation as a box
sat_size = 100; % km
sat_vertices = sat_size * [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; ...
                          -1 -1 1;  1 -1 1;  1 1 1;  -1 1 1];

% Translate to satellite position
sat_vertices = sat_vertices + r_i';

% Draw satellite edges
sat_edges = [1 2; 2 3; 3 4; 4 1; ... % bottom face
             5 6; 6 7; 7 8; 8 5; ... % top face  
             1 5; 2 6; 3 7; 4 8];    % vertical edges

for i = 1:size(sat_edges, 1)
    p1 = sat_vertices(sat_edges(i,1), :);
    p2 = sat_vertices(sat_edges(i,2), :);
    plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], ...
          'Color', 'y', 'LineWidth', 2);
end

%==========================================================================
% 5. Calculate and Draw Orbital Frame (Red) - LVLH
%==========================================================================
% Based on your thesis: X=along-track, Y=cross-track, Z=nadir
[R_ECI_to_LVLH, e_x_LVLH, e_y_LVLH, e_z_LVLH] = calculateLVLHFrame(r_i, v_i);

orbital_colors = [1 0 0; 0.7 0 0; 0.5 0 0]; % Different shades of red
orbital_labels = {'X_{LVLH} (Along-track)', 'Y_{LVLH} (Cross-track)', 'Z_{LVLH} (Nadir)'};
orbital_vectors = [e_x_LVLH, e_y_LVLH, e_z_LVLH];

for i = 1:3
    axis_start = r_i;
    axis_end = r_i + FRAME_LENGTH * orbital_vectors(:,i);
    
    % Draw axis line
    plot3([axis_start(1) axis_end(1)], [axis_start(2) axis_end(2)], ...
          [axis_start(3) axis_end(3)], 'Color', orbital_colors(i,:), 'LineWidth', 3);
    
    % Add arrowhead
    drawArrowHead(axis_start, FRAME_LENGTH * orbital_vectors(:,i), ...
                  orbital_colors(i,:), FRAME_LENGTH*0.05);
    
    % Add label
    label_pos = axis_end + 200 * orbital_vectors(:,i);
    text(label_pos(1), label_pos(2), label_pos(3), orbital_labels{i}, ...
         'Color', orbital_colors(i,:), 'FontSize', 10, 'FontWeight', 'bold');
end

%==========================================================================
% 6. Calculate and Draw Body Frame (Blue)
%==========================================================================
% At t=0, body frame = LVLH frame, then rotated by quaternion
R_quat = quaternionToRotationMatrix(q_inertial_to_body);
R_body_to_ECI = R_quat;

body_colors = [0 0 1; 0 0 0.7; 0 0 0.5]; % Different shades of blue
body_labels = {'X_{Body} (Roll)', 'Y_{Body} (Yaw)', 'Z_{Body} (Pitch)'};

for i = 1:3
    body_axis_ECI = R_body_to_ECI(:, i);
    axis_start = r_i;
    axis_end = r_i + FRAME_LENGTH * 0.8 * body_axis_ECI; % Slightly shorter
    
    % Draw axis line
    plot3([axis_start(1) axis_end(1)], [axis_start(2) axis_end(2)], ...
          [axis_start(3) axis_end(3)], 'Color', body_colors(i,:), 'LineWidth', 3);
    
    % Add arrowhead
    drawArrowHead(axis_start, FRAME_LENGTH * 0.8 * body_axis_ECI, ...
                  body_colors(i,:), FRAME_LENGTH*0.04);
    
    % Add label
    label_pos = axis_end + 150 * body_axis_ECI;
    text(label_pos(1), label_pos(2), label_pos(3), body_labels{i}, ...
         'Color', body_colors(i,:), 'FontSize', 10, 'FontWeight', 'bold');
end

%==========================================================================
% 7. Add Information Text
%==========================================================================
info_text = sprintf(['Satellite Position: [%.1f, %.1f, %.1f] km\n' ...
                    'Satellite Velocity: [%.2f, %.2f, %.2f] km/s\n' ...
                    'Quaternion q_I2B: [%.3f, %.3f, %.3f, %.3f]\n' ...
                    'Altitude: %.1f km'], ...
                   r_i(1), r_i(2), r_i(3), ...
                   v_i(1), v_i(2), v_i(3), ...
                   q_inertial_to_body(1), q_inertial_to_body(2), ...
                   q_inertial_to_body(3), q_inertial_to_body(4), ...
                   norm(r_i) - EARTH_RADIUS);

text(0.02, 0.98, info_text, 'Units', 'normalized', 'Color', 'w', ...
     'FontSize', 10, 'VerticalAlignment', 'top', 'FontName', 'Courier');

%==========================================================================
% 8. Add Legend
%==========================================================================
legend_text = {
    'ECI Frame (Green): X=Vernal Equinox, Y=90°E, Z=North Pole'
    'Orbital LVLH (Red): X=Along-track, Y=Cross-track, Z=Nadir'  
    'Body Frame (Blue): At t=0→LVLH, then rotated by quaternion'
    'Position Vector (Magenta): From Earth center to satellite'
    'Satellite (Yellow): Current satellite position and orientation'
};

for i = 1:length(legend_text)
    text(0.02, 0.15 - (i-1)*0.025, legend_text{i}, 'Units', 'normalized', ...
         'Color', 'w', 'FontSize', 9);
end

% Set axis limits
max_dist = max(norm(r_i) * 1.2, 8000);
xlim([-max_dist max_dist]);
ylim([-max_dist max_dist]); 
zlim([-max_dist max_dist]);

% Interactive controls info
text(0.98, 0.02, 'Mouse: Rotate | Scroll: Zoom | Click+Drag: Pan', ...
     'Units', 'normalized', 'Color', 'w', 'FontSize', 9, ...
     'HorizontalAlignment', 'right');

end

%==========================================================================
% Helper Functions
%==========================================================================

function [R_ECI_to_LVLH, e_x, e_y, e_z] = calculateLVLHFrame(r_i, v_i)
    % Calculate LVLH frame based on thesis definition
    % X = along-track (tangent to orbit, direction of motion)
    % Y = cross-track (out of orbital plane, angular momentum direction)  
    % Z = nadir (toward Earth center)
    
    % Nadir direction (toward Earth center)
    e_z = -r_i / norm(r_i);
    
    % Angular momentum vector (cross-track direction)
    h = cross(e_z, v_i);
    e_y = h / norm(h);
    
    % Along-track direction (completes right-handed system)
    e_x = cross(e_y, e_z);
    
    % Rotation matrix from ECI to LVLH
    R_ECI_to_LVLH = [e_x'; e_y'; e_z'];
end

function R = quaternionToRotationMatrix(q)
    % Convert quaternion [w, x, y, z] to rotation matrix
    q = q / norm(q); % Normalize
    w = q(1); x = q(2); y = q(3); z = q(4);
    
    R = [1-2*(y^2+z^2),  2*(x*y-w*z),  2*(x*z+w*y);
         2*(x*y+w*z),   1-2*(x^2+z^2), 2*(y*z-w*x);
         2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x^2+y^2)];
end

function drawArrowHead(start_pos, direction, color, size)
    % Draw a simple arrowhead
    direction = direction / norm(direction);
    end_pos = start_pos + direction * norm(direction);
    
    % Create two perpendicular vectors for arrowhead
    if abs(direction(3)) < 0.9
        perp1 = cross(direction, [0; 0; 1]);
    else
        perp1 = cross(direction, [1; 0; 0]);
    end
    perp1 = perp1 / norm(perp1);
    perp2 = cross(direction, perp1);
    perp2 = perp2 / norm(perp2);
    
    % Arrowhead points
    arrow_length = size;
    arrow_width = size * 0.5;
    
    p1 = end_pos - arrow_length * direction + arrow_width * perp1;
    p2 = end_pos - arrow_length * direction - arrow_width * perp1;
    p3 = end_pos - arrow_length * direction + arrow_width * perp2;
    p4 = end_pos - arrow_length * direction - arrow_width * perp2;
    
    % Draw arrowhead lines
    plot3([end_pos(1) p1(1)], [end_pos(2) p1(2)], [end_pos(3) p1(3)], ...
          'Color', color, 'LineWidth', 2);
    plot3([end_pos(1) p2(1)], [end_pos(2) p2(2)], [end_pos(3) p2(3)], ...
          'Color', color, 'LineWidth', 2);
    plot3([end_pos(1) p3(1)], [end_pos(2) p3(2)], [end_pos(3) p3(3)], ...
          'Color', color, 'LineWidth', 2);
    plot3([end_pos(1) p4(1)], [end_pos(2) p4(2)], [end_pos(3) p4(3)], ...
          'Color', color, 'LineWidth', 2);
end

% Example usage with test cases:
% SatelliteReferenceFrameVisualization(); % Default case
% SatelliteReferenceFrameVisualization([6800; 0; 0], [0; 7.5; 0], [1; 0; 0; 0]);
% SatelliteReferenceFrameVisualization([0; 6800; 0], [-7.5; 0; 0], [0.7071; 0; 0; 0.7071]);