function PlotBodyToECI(quaternions_body_to_eci, dt, options)
%==========================================================================
% Niel Theron
% Plot Body Frame Axes in ECI Frame (Direct transformation)
%==========================================================================
% Purpose: Visualize body frame axes in ECI using direct body-to-ECI quaternions
%          with moment of inertia coupling effects
%
% Inputs:
% quaternions_body_to_eci - [4 x N] quaternions from body to ECI [qw; qx; qy; qz]
% dt                      - Time step (s)
% options                 - (optional) struct with plotting options
%
% Body Frame Convention (LVLH-based):
% X-axis: Along-track (forward)
% Y-axis: Cross-track (down/perpendicular to orbit)
% Z-axis: Nadir (toward Earth)
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
if ~isfield(options, 'debug_roll'), options.debug_roll = false; end
if ~isfield(options, 'title_text'), options.title_text = 'Body Frame in ECI (Direct Transform)'; end

% Moment of inertia parameters - add these new options
if ~isfield(options, 'include_inertia'), options.include_inertia = true; end
if ~isfield(options, 'I_body'), options.I_body = [1.0; 1.0; 1.0]; end  % Default inertia values [Ixx; Iyy; Izz] kg*m^2
if ~isfield(options, 'omega_body'), options.omega_body = [0.1; 0; 0]; end  % Default angular velocity [wx; wy; wz] rad/s
if ~isfield(options, 'show_inertia_effects'), options.show_inertia_effects = true; end

% Handle quaternion input format
if size(quaternions_body_to_eci, 1) == 4
    q_data = quaternions_body_to_eci;
elseif size(quaternions_body_to_eci, 2) == 4
    q_data = quaternions_body_to_eci';
else
    error('Quaternion data must be either [4 x N] or [N x 4] format');
end

[~, N] = size(q_data);

% Initialize body frame axis direction vectors in ECI
x_body_eci = zeros(3, N);
y_body_eci = zeros(3, N);
z_body_eci = zeros(3, N);

% Body frame axes in body coordinates
x_body_local = [1; 0; 0];  % Along-track
y_body_local = [0; 1; 0];  % Cross-track  
z_body_local = [0; 0; 1];  % Nadir

% Store rotation matrices for debugging
R_matrices = zeros(3, 3, N);

% Initialize moment of inertia coupling calculations
if options.include_inertia
    I = options.I_body;  % [Ixx; Iyy; Izz]
    omega = options.omega_body;  % [wx; wy; wz]
    
    % Calculate moment of inertia coupling torques
    % Euler's equation: M = I*omega_dot + omega x (I*omega)
    % For coupling: M_coupling = omega x (I*omega)
    I_omega = [I(1)*omega(1); I(2)*omega(2); I(3)*omega(3)];
    
    % Cross product: omega x (I*omega)
    M_coupling = [omega(2)*I_omega(3) - omega(3)*I_omega(2);
                  omega(3)*I_omega(1) - omega(1)*I_omega(3);
                  omega(1)*I_omega(2) - omega(2)*I_omega(1)];
    
    % Calculate angular acceleration due to coupling
    % omega_dot = inv(I) * (M_external - omega x (I*omega))
    % For free motion (M_external = 0): omega_dot = -inv(I) * (omega x (I*omega))
    omega_dot_coupling = [-M_coupling(1)/I(1); -M_coupling(2)/I(2); -M_coupling(3)/I(3)];
    
    % Store inertia coupling data for visualization
    inertia_data.I = I;
    inertia_data.omega = omega;
    inertia_data.M_coupling = M_coupling;
    inertia_data.omega_dot_coupling = omega_dot_coupling;
    
    fprintf('\n=== MOMENT OF INERTIA COUPLING ANALYSIS ===\n');
    fprintf('Moment of Inertia [kg*m^2]: Ixx=%.3f, Iyy=%.3f, Izz=%.3f\n', I(1), I(2), I(3));
    fprintf('Angular Velocity [rad/s]: wx=%.6f, wy=%.6f, wz=%.6f\n', omega(1), omega(2), omega(3));
    fprintf('Coupling Torque [N*m]: Mx=%.6f, My=%.6f, Mz=%.6f\n', M_coupling(1), M_coupling(2), M_coupling(3));
    fprintf('Coupling Angular Acceleration [rad/s^2]: wx_dot=%.6f, wy_dot=%.6f, wz_dot=%.6f\n', ...
            omega_dot_coupling(1), omega_dot_coupling(2), omega_dot_coupling(3));
    
    % Calculate magnitude of coupling effects
    coupling_torque_mag = norm(M_coupling);
    coupling_accel_mag = norm(omega_dot_coupling);
    
    fprintf('Coupling Torque Magnitude: %.6f N*m\n', coupling_torque_mag);
    fprintf('Coupling Angular Acceleration Magnitude: %.6f rad/s^2\n', coupling_accel_mag);
    fprintf('===============================================\n\n');
end

% Transform body axes to ECI for each time step
for i = 1:N
    % Extract and normalize quaternion [qw, qx, qy, qz]
    q = q_data(:, i);
    q = q / norm(q);
    
    % Extract components
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    
    % Create rotation matrix from body to ECI
    % Using standard quaternion-to-rotation matrix conversion
    R_body_to_eci = [1-2*(qy^2+qz^2),  2*(qx*qy-qw*qz),   2*(qx*qz+qw*qy);
                     2*(qx*qy+qw*qz),  1-2*(qx^2+qz^2),   2*(qy*qz-qw*qx);
                     2*(qx*qz-qw*qy),  2*(qy*qz+qw*qx),   1-2*(qx^2+qy^2)];
    
    % Store for debugging
    R_matrices(:, :, i) = R_body_to_eci;
    
    % Transform body axes to ECI
    x_body_eci(:, i) = R_body_to_eci * x_body_local;
    y_body_eci(:, i) = R_body_to_eci * y_body_local;
    z_body_eci(:, i) = R_body_to_eci * z_body_local;
end

%% Debug output for roll investigation
if options.debug_roll
    fprintf('\n=== ROLL DEBUG ANALYSIS ===\n');
    
    % Check quaternion magnitudes
    q_mags = sqrt(sum(q_data.^2, 1));
    fprintf('Quaternion magnitude range: %.6f to %.6f\n', min(q_mags), max(q_mags));
    
    % Check for quaternion flips (q and -q represent same rotation)
    q_dots = zeros(1, N-1);
    for i = 1:N-1
        q_dots(i) = dot(q_data(:,i), q_data(:,i+1));
    end
    fprintf('Quaternion continuity (dot products): %.3f to %.3f\n', min(q_dots), max(q_dots));
    
    % Analyze X-axis behavior
    x_dots = zeros(1, N-1);
    x_angles = zeros(1, N-1);
    for i = 1:N-1
        x_dots(i) = dot(x_body_eci(:,i), x_body_eci(:,i+1));
        x_angles(i) = acosd(abs(x_dots(i))); % abs to handle numerical issues
    end
    fprintf('X-axis angular changes: %.3f° to %.3f° per step\n', min(x_angles), max(x_angles));
    
    % Check if X-axis has large jumps
    large_jumps = find(x_angles > 10); % >10 degree jumps
    if ~isempty(large_jumps)
        fprintf('WARNING: Large X-axis jumps at time steps: %s\n', mat2str(large_jumps));
    end
    
    % Show start and end orientations
    fprintf('\nStart X-axis (ECI): [%.3f, %.3f, %.3f]\n', x_body_eci(:,1));
    fprintf('End X-axis (ECI):   [%.3f, %.3f, %.3f]\n', x_body_eci(:,end));
    fprintf('============================\n\n');
end

%% Create the plot
figure('Name', 'Body Frame in ECI', 'Position', [100, 100, 1000, 700]);
hold on;

% Plot unit sphere if requested
if options.show_sphere
    [X_sphere, Y_sphere, Z_sphere] = sphere(50);
    surf(X_sphere, Y_sphere, Z_sphere, 'FaceColor', [0.8 0.8 0.8], ...
         'EdgeColor', 'none', 'FaceAlpha', options.alpha);
end

% Plot the paths of body frame axes
if options.show_all_axes
    % X-axis path (red) - Along-track
    plot3(x_body_eci(1, :), x_body_eci(2, :), x_body_eci(3, :), ...
          'r-', 'LineWidth', options.line_width, 'DisplayName', 'X-axis (Along-track)');
    
    % Y-axis path (green) - Cross-track
    plot3(y_body_eci(1, :), y_body_eci(2, :), y_body_eci(3, :), ...
          'g-', 'LineWidth', options.line_width, 'DisplayName', 'Y-axis (Cross-track)');
    
    % Z-axis path (blue) - Nadir
    plot3(z_body_eci(1, :), z_body_eci(2, :), z_body_eci(3, :), ...
          'b-', 'LineWidth', options.line_width, 'DisplayName', 'Z-axis (Nadir)');
else
    % Only plot Z-axis path (nadir)
    plot3(z_body_eci(1, :), z_body_eci(2, :), z_body_eci(3, :), ...
          'b-', 'LineWidth', options.line_width, 'DisplayName', 'Z-axis (Nadir)');
end

% Mark start and end points if requested
if options.show_start_end
    if options.show_all_axes
        % Start points (circles)
        plot3(x_body_eci(1, 1), x_body_eci(2, 1), x_body_eci(3, 1), ...
              'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
        plot3(y_body_eci(1, 1), y_body_eci(2, 1), y_body_eci(3, 1), ...
              'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        plot3(z_body_eci(1, 1), z_body_eci(2, 1), z_body_eci(3, 1), ...
              'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
        
        % End points (squares)
        plot3(x_body_eci(1, end), x_body_eci(2, end), x_body_eci(3, end), ...
              'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        plot3(y_body_eci(1, end), y_body_eci(2, end), y_body_eci(3, end), ...
              'gs', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
        plot3(z_body_eci(1, end), z_body_eci(2, end), z_body_eci(3, end), ...
              'bs', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    end
end

% Add satellite cube at the final orientation if requested
if options.show_cube
    if options.include_inertia
        addSatelliteCube([0, 0, 0], R_matrices(:, :, end), options.cube_size, inertia_data);
    else
        addSatelliteCube([0, 0, 0], R_matrices(:, :, end), options.cube_size);
    end
end

% Set axis properties
axis equal;
xlabel('X (ECI)');
ylabel('Y (ECI)');
zlabel('Z (ECI)');
title(options.title_text);
grid on;

% Set view angle for better visualization
view(45, 30);

% Add time information to title if dt is provided
if nargin >= 2 && ~isempty(dt)
    total_time = (N-1) * dt;
    if options.include_inertia
        title(sprintf('%s\nTotal Time: %.1f s, Time Step: %.3f s\nInertia Coupling: |M|=%.3e N⋅m', ...
              options.title_text, total_time, dt, norm(inertia_data.M_coupling)));
    else
        title(sprintf('%s\nTotal Time: %.1f s, Time Step: %.3f s', ...
              options.title_text, total_time, dt));
    end
end

% Add ECI coordinate frame reference arrows
quiver3(0, 0, 0, 1.2, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.1);
quiver3(0, 0, 0, 0, 1.2, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.1);
quiver3(0, 0, 0, 0, 0, 1.2, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.1);
text(1.3, 0, 0, 'X_{ECI}', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');
text(0, 1.3, 0, 'Y_{ECI}', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'g');
text(0, 0, 1.3, 'Z_{ECI}', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'b');

% Set axis limits
axis([-1.5 1.5 -1.5 1.5 -1.5 1.5]);

hold off;

% Print statistics
fprintf('=== Body Frame in ECI Analysis ===\n');
fprintf('Total simulation time: %.2f seconds\n', (N-1) * dt);
fprintf('Number of attitude samples: %d\n', N);

if options.show_all_axes
    % Calculate average angular displacements for all axes
    x_angular_disp = mean(acosd(abs(dot(x_body_eci(:, 1:end-1), x_body_eci(:, 2:end)))));
    y_angular_disp = mean(acosd(abs(dot(y_body_eci(:, 1:end-1), y_body_eci(:, 2:end)))));
    z_angular_disp = mean(acosd(abs(dot(z_body_eci(:, 1:end-1), z_body_eci(:, 2:end)))));
    
    fprintf('Average angular displacement per step:\n');
    fprintf('  X-axis: %.4f degrees\n', x_angular_disp);
    fprintf('  Y-axis: %.4f degrees\n', y_angular_disp);
    fprintf('  Z-axis: %.4f degrees\n', z_angular_disp);
end

% Print inertia coupling summary if enabled
if options.include_inertia
    fprintf('\nInertia Coupling Summary:\n');
    fprintf('  Principal moments: [%.3f, %.3f, %.3f] kg⋅m²\n', I(1), I(2), I(3));
    fprintf('  Coupling torque magnitude: %.6f N⋅m\n', norm(inertia_data.M_coupling));
    fprintf('  Angular acceleration magnitude: %.6f rad/s²\n', norm(inertia_data.omega_dot_coupling));
end

fprintf('======================================\n');

end

function addSatelliteCube(center, R_matrix, cube_size, varargin)
%==========================================================================
% Add a small grey cube representing the satellite body
% Optional: Include inertia tensor visualization
%==========================================================================

% Check if inertia data is provided
if nargin > 3
    inertia_data = varargin{1};
    show_inertia = true;
else
    show_inertia = false;
end

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

% Rotate vertices to ECI frame
vertices_eci = (R_matrix * vertices_body')' + center;

% Define cube faces
faces = [
    1, 2, 3, 4;  % bottom face (-z)
    5, 6, 7, 8;  % top face (+z)
    1, 2, 6, 5;  % front face (-y)
    3, 4, 8, 7;  % back face (+y)
    1, 4, 8, 5;  % left face (-x)
    2, 3, 7, 6   % right face (+x)
];

% Plot the cube with different colors based on inertia if available
for i = 1:size(faces, 1)
    face_vertices = vertices_eci(faces(i, :), :);
    face_vertices(end+1, :) = face_vertices(1, :); % Close the face
    
    if show_inertia
        % Color faces based on moment of inertia magnitude
        I = inertia_data.I;
        max_I = max(I);
        min_I = min(I);
        
        if i <= 2
            % Top and bottom faces (Z-axis) - color by Izz
            intensity = (I(3) - min_I) / (max_I - min_I);
            color = [0.4 + 0.4*intensity, 0.4, 0.4];
        elseif i <= 4
            % Front and back faces (Y-axis) - color by Iyy
            intensity = (I(2) - min_I) / (max_I - min_I);
            color = [0.4, 0.4 + 0.4*intensity, 0.4];
        else
            % Left and right faces (X-axis) - color by Ixx
            intensity = (I(1) - min_I) / (max_I - min_I);
            color = [0.4, 0.4, 0.4 + 0.4*intensity];
        end
        
        patch(face_vertices(:, 1), face_vertices(:, 2), face_vertices(:, 3), ...
              color, 'EdgeColor', 'k', 'LineWidth', 0.5, 'FaceAlpha', 0.8);
    else
        % Standard coloring
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
end

% Add body frame axes on the cube
axis_length = cube_size * 1.5;
cube_center = center;

% X-axis (red) - Along-track
x_axis_end = cube_center + (R_matrix * [axis_length; 0; 0])';
quiver3(cube_center(1), cube_center(2), cube_center(3), ...
        x_axis_end(1) - cube_center(1), x_axis_end(2) - cube_center(2), x_axis_end(3) - cube_center(3), ...
        'r', 'LineWidth', 2, 'MaxHeadSize', 0.3);

% Y-axis (green) - Cross-track
y_axis_end = cube_center + (R_matrix * [0; axis_length; 0])';
quiver3(cube_center(1), cube_center(2), cube_center(3), ...
        y_axis_end(1) - cube_center(1), y_axis_end(2) - cube_center(2), y_axis_end(3) - cube_center(3), ...
        'g', 'LineWidth', 2, 'MaxHeadSize', 0.3);

% Z-axis (blue) - Nadir
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

% Add inertia visualization if available
if show_inertia
    % Add coupling torque vector
    torque_scale = 100;  % Scale factor for visualization
    M_scaled = inertia_data.M_coupling * torque_scale;
    M_eci = R_matrix * M_scaled;
    
    % Plot coupling torque vector in magenta
    quiver3(cube_center(1), cube_center(2), cube_center(3), ...
            M_eci(1), M_eci(2), M_eci(3), ...
            'm', 'LineWidth', 3, 'MaxHeadSize', 0.2);
    
    % Add torque label
    torque_end = cube_center + M_eci';
    text(torque_end(1), torque_end(2), torque_end(3), 'M_{coupling}', ...
         'FontSize', 8, 'FontWeight', 'bold', 'Color', 'm');
    
    % Add inertia information text
    text(cube_center(1) + 0.3, cube_center(2) + 0.3, cube_center(3) + 0.3, ...
         sprintf('I=[%.2f,%.2f,%.2f]', inertia_data.I(1), inertia_data.I(2), inertia_data.I(3)), ...
         'FontSize', 7, 'Color', 'k', 'BackgroundColor', 'w');
end

end