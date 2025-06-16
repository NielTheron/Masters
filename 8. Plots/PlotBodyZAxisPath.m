function PlotBodyZAxisPath(quaternions, dt, options)
%==========================================================================
% Niel Theron
% Plot Body Frame Axes Paths from Quaternion Data
%==========================================================================
% Purpose: Visualize the paths traced by the satellite body's X, Y, Z axes 
%          in 3D space using quaternion attitude data
%
% Inputs:
% quaternions - [4 x N] matrix of quaternions [qw; qx; qy; qz] or [N x 4]
% dt         - Time step (s)
% options    - (optional) struct with plotting options:
%              .show_sphere - true/false to show unit sphere (default: true)
%              .show_start_end - true/false to mark start/end points (default: true)
%              .show_all_axes - true/false to show X,Y,Z paths (default: true)
%              .show_cube - true/false to show satellite cube (default: true)
%              .line_width - line width for path (default: 2)
%              .alpha - transparency for sphere (default: 0.3)
%              .cube_size - size of satellite cube (default: 0.1)
%              .title_text - custom title (default: auto-generated)
%
% Output:
% Creates a 3D plot showing the paths of the body axes and satellite orientation
%==========================================================================

% Handle input arguments
if nargin < 3
    options = struct();
end

% Set default options
if ~isfield(options, 'show_sphere'), options.show_sphere = true; end
if ~isfield(options, 'show_start_end'), options.show_start_end = true; end
if ~isfield(options, 'show_all_axes'), options.show_all_axes = true; end
if ~isfield(options, 'show_cube'), options.show_cube = true; end
if ~isfield(options, 'line_width'), options.line_width = 2; end
if ~isfield(options, 'alpha'), options.alpha = 0.3; end
if ~isfield(options, 'cube_size'), options.cube_size = 0.1; end
if ~isfield(options, 'title_text'), options.title_text = 'Satellite Body Frame Orientation in 3D Space'; end

% Handle quaternion input format
if size(quaternions, 1) == 4
    % Input is [4 x N] format
    q_data = quaternions;
elseif size(quaternions, 2) == 4
    % Input is [N x 4] format, transpose it
    q_data = quaternions';
else
    error('Quaternion data must be either [4 x N] or [N x 4] format');
end

[~, N] = size(q_data);

% Initialize body frame axis direction vectors
x_body_vectors = zeros(3, N);
y_body_vectors = zeros(3, N);
z_body_vectors = zeros(3, N);

% Body frame axes in body coordinates
x_body_local = [1; 0; 0];
y_body_local = [0; 1; 0];
z_body_local = [0; 0; 1];

% Store rotation matrices for cube orientation
R_matrices = zeros(3, 3, N);

% Calculate body frame axes in inertial frame for each time step
for i = 1:N
    % Extract quaternion [qw, qx, qy, qz]
    q = q_data(:, i);
    
    % Normalize quaternion
    q = q / norm(q);
    
    % Extract components
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    
    % Create rotation matrix from body to inertial frame
    R_body_to_inertial = [1-2*(qy^2+qz^2),  2*(qx*qy-qw*qz),   2*(qx*qz+qw*qy);
                          2*(qx*qy+qw*qz),  1-2*(qx^2+qz^2),   2*(qy*qz-qw*qx);
                          2*(qx*qz-qw*qy),  2*(qy*qz+qw*qx),   1-2*(qx^2+qy^2)];
    
    % Store rotation matrix for cube orientation
    R_matrices(:, :, i) = R_body_to_inertial;
    
    % Transform all body axes to inertial frame
    x_body_vectors(:, i) = R_body_to_inertial * x_body_local;
    y_body_vectors(:, i) = R_body_to_inertial * y_body_local;
    z_body_vectors(:, i) = R_body_to_inertial * z_body_local;
end

% Create the plot
figure('Name', 'Satellite Body Frame Orientation', 'Position', [100, 100, 1000, 700]);
hold on;

% Plot unit sphere if requested
if options.show_sphere
    [X_sphere, Y_sphere, Z_sphere] = sphere(50);
    surf(X_sphere, Y_sphere, Z_sphere, 'FaceColor', [0.8 0.8 0.8], ...
         'EdgeColor', 'none', 'FaceAlpha', options.alpha);
end

% Plot the paths of all body frame axes if requested
if options.show_all_axes
    % X-axis path (red)
    plot3(x_body_vectors(1, :), x_body_vectors(2, :), x_body_vectors(3, :), ...
          'r-', 'LineWidth', options.line_width, 'DisplayName', 'X-axis path');
    
    % Y-axis path (green)
    plot3(y_body_vectors(1, :), y_body_vectors(2, :), y_body_vectors(3, :), ...
          'g-', 'LineWidth', options.line_width, 'DisplayName', 'Y-axis path');
    
    % Z-axis path (blue)
    plot3(z_body_vectors(1, :), z_body_vectors(2, :), z_body_vectors(3, :), ...
          'b-', 'LineWidth', options.line_width, 'DisplayName', 'Z-axis path');
else
    % Only plot Z-axis path
    plot3(z_body_vectors(1, :), z_body_vectors(2, :), z_body_vectors(3, :), ...
          'b-', 'LineWidth', options.line_width, 'DisplayName', 'Z-axis path');
end

% Mark start and end points if requested
if options.show_start_end
    if options.show_all_axes
        % Start points
        plot3(x_body_vectors(1, 1), x_body_vectors(2, 1), x_body_vectors(3, 1), ...
              'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
        plot3(y_body_vectors(1, 1), y_body_vectors(2, 1), y_body_vectors(3, 1), ...
              'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        plot3(z_body_vectors(1, 1), z_body_vectors(2, 1), z_body_vectors(3, 1), ...
              'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
        
        % End points
        plot3(x_body_vectors(1, end), x_body_vectors(2, end), x_body_vectors(3, end), ...
              'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        plot3(y_body_vectors(1, end), y_body_vectors(2, end), y_body_vectors(3, end), ...
              'gs', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
        plot3(z_body_vectors(1, end), z_body_vectors(2, end), z_body_vectors(3, end), ...
              'bs', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    else
        % Start point (green circle)
        plot3(z_body_vectors(1, 1), z_body_vectors(2, 1), z_body_vectors(3, 1), ...
              'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
        
        % End point (red square)
        plot3(z_body_vectors(1, end), z_body_vectors(2, end), z_body_vectors(3, end), ...
              'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'End');
    end
end

% Add satellite cube at the final orientation if requested
if options.show_cube
    addSatelliteCube([0, 0, 0], R_matrices(:, :, end), options.cube_size);
end

% Set axis properties
axis equal;
xlabel('X (Inertial Frame)');
ylabel('Y (Inertial Frame)');
zlabel('Z (Inertial Frame)');
title(options.title_text);
grid on;

% Add legend
if options.show_all_axes || options.show_start_end
    legend('Location', 'best');
end

% Set view angle for better visualization
view(45, 30);

% Add time information to title if dt is provided
if nargin >= 2 && ~isempty(dt)
    total_time = (N-1) * dt;
    title(sprintf('%s\nTotal Time: %.1f s, Time Step: %.3f s', ...
          options.title_text, total_time, dt));
end

% Add inertial coordinate frame reference arrows
quiver3(0, 0, 0, 1.2, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.1);
quiver3(0, 0, 0, 0, 1.2, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.1);
quiver3(0, 0, 0, 0, 0, 1.2, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.1);
text(1.3, 0, 0, 'X_{inertial}', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');
text(0, 1.3, 0, 'Y_{inertial}', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'g');
text(0, 0, 1.3, 'Z_{inertial}', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'b');

% Set axis limits
axis([-1.5 1.5 -1.5 1.5 -1.5 1.5]);

hold off;

% Print some statistics
fprintf('=== Satellite Attitude Dynamics Analysis ===\n');
fprintf('Total simulation time: %.2f seconds\n', (N-1) * dt);
fprintf('Number of attitude samples: %d\n', N);

if options.show_all_axes
    % Calculate average angular displacements for all axes
    x_angular_disp = mean(acosd(dot(x_body_vectors(:, 1:end-1), x_body_vectors(:, 2:end))));
    y_angular_disp = mean(acosd(dot(y_body_vectors(:, 1:end-1), y_body_vectors(:, 2:end))));
    z_angular_disp = mean(acosd(dot(z_body_vectors(:, 1:end-1), z_body_vectors(:, 2:end))));
    
    fprintf('Average angular displacement per step:\n');
    fprintf('  X-axis: %.4f degrees\n', x_angular_disp);
    fprintf('  Y-axis: %.4f degrees\n', y_angular_disp);
    fprintf('  Z-axis: %.4f degrees\n', z_angular_disp);
    
    % Calculate total angular travels
    total_x_travel = 0; total_y_travel = 0; total_z_travel = 0;
    for i = 2:N
        total_x_travel = total_x_travel + acosd(dot(x_body_vectors(:, i-1), x_body_vectors(:, i)));
        total_y_travel = total_y_travel + acosd(dot(y_body_vectors(:, i-1), y_body_vectors(:, i)));
        total_z_travel = total_z_travel + acosd(dot(z_body_vectors(:, i-1), z_body_vectors(:, i)));
    end
    
    fprintf('Total angular travel:\n');
    fprintf('  X-axis: %.2f degrees\n', total_x_travel);
    fprintf('  Y-axis: %.2f degrees\n', total_y_travel);
    fprintf('  Z-axis: %.2f degrees\n', total_z_travel);
else
    z_angular_disp = mean(acosd(dot(z_body_vectors(:, 1:end-1), z_body_vectors(:, 2:end))));
    fprintf('Average Z-axis angular displacement per step: %.4f degrees\n', z_angular_disp);
    
    total_z_travel = 0;
    for i = 2:N
        total_z_travel = total_z_travel + acosd(dot(z_body_vectors(:, i-1), z_body_vectors(:, i)));
    end
    fprintf('Total Z-axis angular travel: %.2f degrees\n', total_z_travel);
end
fprintf('==========================================\n');

end

function addSatelliteCube(center, R_matrix, cube_size)
%==========================================================================
% Add a small grey cube representing the satellite body
%==========================================================================
% Inputs:
% center - [x, y, z] center position of cube
% R_matrix - 3x3 rotation matrix for cube orientation
% cube_size - size of the cube
%==========================================================================

% Define cube vertices in body frame (centered at origin)
s = cube_size / 2;
vertices_body = [
    -s, -s, -s;  % 1
     s, -s, -s;  % 2
     s,  s, -s;  % 3
    -s,  s, -s;  % 4
    -s, -s,  s;  % 5
     s, -s,  s;  % 6
     s,  s,  s;  % 7
    -s,  s,  s   % 8
];

% Rotate vertices to inertial frame
vertices_inertial = (R_matrix * vertices_body')' + center;

% Define cube faces (each row defines a face using vertex indices)
faces = [
    1, 2, 3, 4;  % bottom face (-z)
    5, 6, 7, 8;  % top face (+z)
    1, 2, 6, 5;  % front face (-y)
    3, 4, 8, 7;  % back face (+y)
    1, 4, 8, 5;  % left face (-x)
    2, 3, 7, 6   % right face (+x)
];

% Plot the cube
for i = 1:size(faces, 1)
    face_vertices = vertices_inertial(faces(i, :), :);
    face_vertices(end+1, :) = face_vertices(1, :); % Close the face
    
    if i <= 2
        % Top and bottom faces - darker grey
        patch(face_vertices(:, 1), face_vertices(:, 2), face_vertices(:, 3), ...
              [0.4, 0.4, 0.4], 'EdgeColor', 'k', 'LineWidth', 0.5, 'FaceAlpha', 0.8);
    else
        % Side faces - lighter grey
        patch(face_vertices(:, 1), face_vertices(:, 2), face_vertices(:, 3), ...
              [0.6, 0.6, 0.6], 'EdgeColor', 'k', 'LineWidth', 0.5, 'FaceAlpha', 0.8);
    end
end

% Add body frame axes on the cube for reference
axis_length = cube_size * 1.5;
cube_center = center;

% X-axis (red)
x_axis_end = cube_center + (R_matrix * [axis_length; 0; 0])';
quiver3(cube_center(1), cube_center(2), cube_center(3), ...
        x_axis_end(1) - cube_center(1), x_axis_end(2) - cube_center(2), x_axis_end(3) - cube_center(3), ...
        'r', 'LineWidth', 2, 'MaxHeadSize', 0.3);

% Y-axis (green)
y_axis_end = cube_center + (R_matrix * [0; axis_length; 0])';
quiver3(cube_center(1), cube_center(2), cube_center(3), ...
        y_axis_end(1) - cube_center(1), y_axis_end(2) - cube_center(2), y_axis_end(3) - cube_center(3), ...
        'g', 'LineWidth', 2, 'MaxHeadSize', 0.3);

% Z-axis (blue)
z_axis_end = cube_center + (R_matrix * [0; 0; axis_length])';
quiver3(cube_center(1), cube_center(2), cube_center(3), ...
        z_axis_end(1) - cube_center(1), z_axis_end(2) - cube_center(2), z_axis_end(3) - cube_center(3), ...
        'b', 'LineWidth', 2, 'MaxHeadSize', 0.3);

% Add body frame labels
text(x_axis_end(1), x_axis_end(2), x_axis_end(3), 'X_{body}', ...
     'FontSize', 8, 'FontWeight', 'bold', 'Color', 'r');
text(y_axis_end(1), y_axis_end(2), y_axis_end(3), 'Y_{body}', ...
     'FontSize', 8, 'FontWeight', 'bold', 'Color', 'g');
text(z_axis_end(1), z_axis_end(2), z_axis_end(3), 'Z_{body}', ...
     'FontSize', 8, 'FontWeight', 'bold', 'Color', 'b');

end