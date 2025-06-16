function PlotBodyFrameCorrected(position_eci, velocity_eci, quaternions_body_to_eci, dt, options)
%==========================================================================
% Niel Theron
% Plot Body Frame Corrected for Orbital Motion
%==========================================================================
% Purpose: Plot body frame relative to the MOVING orbital reference frame
%          This removes the natural orbital rotation to show just the 
%          attitude maneuvers relative to the local orbital frame
%
% Inputs:
% position_eci            - [3 x N] satellite position in ECI (km)
% velocity_eci            - [3 x N] satellite velocity in ECI (km/s) 
% quaternions_body_to_eci - [4 x N] quaternions from body to ECI [qw; qx; qy; qz]
% dt                      - Time step (s)
% options                 - (optional) struct with plotting options
%==========================================================================

if nargin < 5
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
if ~isfield(options, 'show_orbital_drift'), options.show_orbital_drift = false; end
if ~isfield(options, 'title_text'), options.title_text = 'Body Frame Relative to Orbital Frame'; end

% Handle quaternion input format
if size(quaternions_body_to_eci, 1) == 4
    q_body_eci = quaternions_body_to_eci;
elseif size(quaternions_body_to_eci, 2) == 4
    q_body_eci = quaternions_body_to_eci';
else
    error('Quaternion data must be either [4 x N] or [N x 4] format');
end

[~, N] = size(q_body_eci);

% Initialize arrays
x_body_orbital = zeros(3, N);  % Body axes relative to orbital frame
y_body_orbital = zeros(3, N);
z_body_orbital = zeros(3, N);
x_body_eci = zeros(3, N);      % Body axes in ECI (for comparison)
y_body_eci = zeros(3, N);
z_body_eci = zeros(3, N);

% Body frame unit vectors
x_body_local = [1; 0; 0];
y_body_local = [0; 1; 0];  
z_body_local = [0; 0; 1];

fprintf('=== ORBITAL MOTION CORRECTION ANALYSIS ===\n');

% Calculate transformations for each time step
for i = 1:N
    %% Step 1: Calculate current orbital frame (LVLH) in ECI
    r_sat = position_eci(:, i);
    v_sat = velocity_eci(:, i);
    
    % LVLH frame vectors in ECI coordinates
    z_orbital_eci = -r_sat / norm(r_sat);                    % Nadir (toward Earth)
    y_orbital_eci = cross(z_orbital_eci, v_sat);             % Cross-track (normal to orbit)
    y_orbital_eci = y_orbital_eci / norm(y_orbital_eci);
    x_orbital_eci = cross(y_orbital_eci, z_orbital_eci);     % Along-track (velocity direction)
    x_orbital_eci = x_orbital_eci / norm(x_orbital_eci);
    
    % Rotation matrix: ECI to orbital frame
    R_eci_to_orbital = [x_orbital_eci'; y_orbital_eci'; z_orbital_eci'];
    
    %% Step 2: Get body orientation in ECI
    q = q_body_eci(:, i);
    q = q / norm(q);
    
    % Extract quaternion components [qw, qx, qy, qz]
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    
    % Rotation matrix: body to ECI
    R_body_to_eci = [1-2*(qy^2+qz^2),  2*(qx*qy-qw*qz),   2*(qx*qz+qw*qy);
                     2*(qx*qy+qw*qz),  1-2*(qx^2+qz^2),   2*(qy*qz-qw*qx);
                     2*(qx*qz-qw*qy),  2*(qy*qz+qw*qx),   1-2*(qx^2+qy^2)];
    
    %% Step 3: Transform body axes to ECI (for comparison)
    x_body_eci(:, i) = R_body_to_eci * x_body_local;
    y_body_eci(:, i) = R_body_to_eci * y_body_local;
    z_body_eci(:, i) = R_body_to_eci * z_body_local;
    
    %% Step 4: Transform body axes to orbital frame (this removes orbital drift!)
    x_body_orbital(:, i) = R_eci_to_orbital * (R_body_to_eci * x_body_local);
    y_body_orbital(:, i) = R_eci_to_orbital * (R_body_to_eci * y_body_local);
    z_body_orbital(:, i) = R_eci_to_orbital * (R_body_to_eci * z_body_local);
end

% Print analysis
orbital_x_change = norm(x_body_eci(:,end) - x_body_eci(:,1));
corrected_x_change = norm(x_body_orbital(:,end) - x_body_orbital(:,1));

fprintf('X-axis change in ECI frame: %.3f\n', orbital_x_change);
fprintf('X-axis change in orbital frame: %.3f\n', corrected_x_change);
fprintf('Orbital drift removed: %.1f%% reduction\n', 100*(1-corrected_x_change/orbital_x_change));
fprintf('===========================================\n\n');

%% Create comparison plots
if options.show_orbital_drift
    % Show both ECI and orbital-relative plots
    figure('Name', 'Body Frame Comparison', 'Position', [100, 100, 1600, 700]);
    
    % Plot 1: Body frame in ECI (with orbital drift)
    subplot(1,2,1);
    hold on;
    plotAxes(x_body_eci, y_body_eci, z_body_eci, options, 'Body Frame in ECI (With Orbital Drift)');
    
    % Plot 2: Body frame relative to orbital frame (drift removed)
    subplot(1,2,2);
    hold on;
    plotAxes(x_body_orbital, y_body_orbital, z_body_orbital, options, 'Body Frame Relative to Orbital Frame');
    
else
    % Show only the corrected orbital-relative plot
    figure('Name', 'Body Frame Relative to Orbital Frame', 'Position', [100, 100, 1000, 700]);
    hold on;
    plotAxes(x_body_orbital, y_body_orbital, z_body_orbital, options, options.title_text);
end

% Print corrected statistics
fprintf('=== CORRECTED BODY FRAME ANALYSIS ===\n');
fprintf('Total simulation time: %.2f seconds\n', (N-1) * dt);

if options.show_all_axes
    % Calculate average angular displacements (orbital-relative)
    x_angular_disp = mean(acosd(abs(dot(x_body_orbital(:, 1:end-1), x_body_orbital(:, 2:end)))));
    y_angular_disp = mean(acosd(abs(dot(y_body_orbital(:, 1:end-1), y_body_orbital(:, 2:end)))));
    z_angular_disp = mean(acosd(abs(dot(z_body_orbital(:, 1:end-1), z_body_orbital(:, 2:end)))));
    
    fprintf('Average angular displacement per step (orbital-relative):\n');
    fprintf('  X-axis: %.4f degrees\n', x_angular_disp);
    fprintf('  Y-axis: %.4f degrees\n', y_angular_disp);
    fprintf('  Z-axis: %.4f degrees\n', z_angular_disp);
    
    fprintf('Start/End orientations (orbital frame):\n');
    fprintf('X-axis: [%.3f,%.3f,%.3f] → [%.3f,%.3f,%.3f]\n', ...
            x_body_orbital(:,1), x_body_orbital(:,end));
    fprintf('Y-axis: [%.3f,%.3f,%.3f] → [%.3f,%.3f,%.3f]\n', ...
            y_body_orbital(:,1), y_body_orbital(:,end));
    fprintf('Z-axis: [%.3f,%.3f,%.3f] → [%.3f,%.3f,%.3f]\n', ...
            z_body_orbital(:,1), z_body_orbital(:,end));
end
fprintf('=====================================\n');

end

function plotAxes(x_vectors, y_vectors, z_vectors, options, title_text)
%==========================================================================
% Helper function to plot the axes
%==========================================================================

% Plot unit sphere if requested
if options.show_sphere
    [X_sphere, Y_sphere, Z_sphere] = sphere(50);
    surf(X_sphere, Y_sphere, Z_sphere, 'FaceColor', [0.8 0.8 0.8], ...
         'EdgeColor', 'none', 'FaceAlpha', options.alpha);
end

% Plot the paths of body frame axes
if options.show_all_axes
    % X-axis path (red) - Along-track
    plot3(x_vectors(1, :), x_vectors(2, :), x_vectors(3, :), ...
          'r-', 'LineWidth', options.line_width, 'DisplayName', 'X-axis (Along-track)');
    
    % Y-axis path (green) - Cross-track
    plot3(y_vectors(1, :), y_vectors(2, :), y_vectors(3, :), ...
          'g-', 'LineWidth', options.line_width, 'DisplayName', 'Y-axis (Cross-track)');
    
    % Z-axis path (blue) - Nadir
    plot3(z_vectors(1, :), z_vectors(2, :), z_vectors(3, :), ...
          'b-', 'LineWidth', options.line_width, 'DisplayName', 'Z-axis (Nadir)');
end

% Mark start and end points
if options.show_start_end && options.show_all_axes
    % Start points (circles)
    plot3(x_vectors(1, 1), x_vectors(2, 1), x_vectors(3, 1), ...
          'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
    plot3(y_vectors(1, 1), y_vectors(2, 1), y_vectors(3, 1), ...
          'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
    plot3(z_vectors(1, 1), z_vectors(2, 1), z_vectors(3, 1), ...
          'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
    
    % End points (squares)  
    plot3(x_vectors(1, end), x_vectors(2, end), x_vectors(3, end), ...
          'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    plot3(y_vectors(1, end), y_vectors(2, end), y_vectors(3, end), ...
          'gs', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot3(z_vectors(1, end), z_vectors(2, end), z_vectors(3, end), ...
          'bs', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
end

% Add satellite cube at final position if requested
if options.show_cube
    % For orbital-relative frame, we need to compute the relative rotation matrix
    % This is simplified - using identity for now
    R_final = eye(3);  % You might want to calculate this properly
    addSatelliteCube([0, 0, 0], R_final, options.cube_size);
end

% Set axis properties
axis equal;
xlabel('X');
ylabel('Y'); 
zlabel('Z');
title(title_text);
grid on;

% Add legend
if options.show_all_axes
    legend('Location', 'best');
end

% Set view and limits
view(45, 30);
axis([-1.5 1.5 -1.5 1.5 -1.5 1.5]);

% Add coordinate frame arrows
quiver3(0, 0, 0, 1.2, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.1);
quiver3(0, 0, 0, 0, 1.2, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.1);
quiver3(0, 0, 0, 0, 0, 1.2, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.1);
text(1.3, 0, 0, 'X', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');
text(0, 1.3, 0, 'Y', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'g');
text(0, 0, 1.3, 'Z', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'b');

end

function addSatelliteCube(center, R_matrix, cube_size)
%==========================================================================
% Add a small grey cube representing the satellite body
%==========================================================================

% Define cube vertices in body frame
s = cube_size / 2;
vertices_body = [
    -s, -s, -s; s, -s, -s; s, s, -s; -s, s, -s;
    -s, -s, s; s, -s, s; s, s, s; -s, s, s
];

% Rotate vertices 
vertices = (R_matrix * vertices_body')' + center;

% Define cube faces
faces = [
    1, 2, 3, 4; 5, 6, 7, 8; 1, 2, 6, 5;
    3, 4, 8, 7; 1, 4, 8, 5; 2, 3, 7, 6
];

% Plot cube faces
for i = 1:size(faces, 1)
    face_vertices = vertices(faces(i, :), :);
    face_vertices(end+1, :) = face_vertices(1, :);
    
    if i <= 2
        color = [0.4, 0.4, 0.4];  % Darker for top/bottom
    else
        color = [0.6, 0.6, 0.6];  % Lighter for sides
    end
    
    patch(face_vertices(:, 1), face_vertices(:, 2), face_vertices(:, 3), ...
          color, 'EdgeColor', 'k', 'LineWidth', 0.5, 'FaceAlpha', 0.8);
end

end