%==========================================================================
% Complete Earth Tracker Test with Satellite Visualization
% Niel Theron
%==========================================================================

% Load and process image
image = imread("Paris.png");
n_f = 20;

% Satellite parameters
sat_quat = [1, 0, 0, 0].'; % Identity quaternion (no rotation)
focal_length_m = 0.58;
pixel_size_m = 17.4e-6;
pixel_height = 720;
pixel_width = 720;

% Detect features
[feature, grayImage, feature_pixels] = Feature_Pixel_Detection(image, n_f);

% Calculate Earth Tracker measurements (no transpose needed)
ET_measurement = EarthTracker(feature, sat_quat, focal_length_m, pixel_size_m, pixel_height, pixel_width);

% Debug: Check dimensions
fprintf('Number of features detected: %d\n', size(feature, 2));
fprintf('ET_measurement dimensions: %d x %d\n', size(ET_measurement, 1), size(ET_measurement, 2));

% Original plot
figure('Name', 'Original Earth Tracker Plot');
PlotEarthTracker(ET_measurement);
title('Original Feature Vector Visualization');

% Enhanced satellite field of view plot
PlotSatelliteFieldOfView(ET_measurement, image, feature, sat_quat, ...
                        focal_length_m, pixel_size_m, pixel_height, pixel_width);

% Additional analysis plot showing vector magnitudes
figure('Name', 'Feature Vector Analysis');
subplot(2,1,1);
vector_magnitudes = sqrt(sum(ET_measurement.^2, 1));
bar(1:length(vector_magnitudes), vector_magnitudes);
xlabel('Feature Index');
ylabel('Vector Magnitude');
title('Feature Vector Magnitudes');
grid on;

subplot(2,1,2);
% Plot vector components
plot(1:size(ET_measurement,2), ET_measurement(1,:), 'r-o', 'LineWidth', 2);
hold on;
plot(1:size(ET_measurement,2), ET_measurement(2,:), 'g-s', 'LineWidth', 2);
plot(1:size(ET_measurement,2), ET_measurement(3,:), 'b-^', 'LineWidth', 2);
xlabel('Feature Index');
ylabel('Component Value');
title('Feature Vector Components');
legend('X Component', 'Y Component', 'Z Component');
grid on;
hold off;

%% Original PlotEarthTracker function (embedded)
function PlotEarthTracker(ET_Measurement)
    d = size(ET_Measurement,2);
    
    % Origin points for all vectors (at camera location)
    X = zeros(1,d);
    Y = zeros(1,d);
    Z = zeros(1,d);
    
    % Create the quiver plot
    quiver3(X,Y,Z,ET_Measurement(1,:),ET_Measurement(2,:),ET_Measurement(3,:), 0)
    
    % Add labels and formatting
    xlabel('X (right)');
    ylabel('Y (down)');
    zlabel('Z (forward/nadir)');
    title('Feature Rays in Camera Frame');
    grid on;
    axis equal;
    
    % Set view to see the rays properly
    view(45, -30);
end

%% Enhanced PlotSatelliteFieldOfView function (embedded)
function PlotSatelliteFieldOfView(ET_Measurement, image, feature_pixel_locations, sat_quat, focal_length_m, pixel_size_m, pixel_height, pixel_width)
    %==========================================================================
    % Enhanced visualization showing satellite above field of view
    % with feature vectors pointing from camera to features
    % Niel Theron - Enhanced version
    %==========================================================================
    
    figure('Position', [100, 100, 1200, 800]);
    
    %% Parameters
    % Camera parameters
    cx = pixel_width / 2;
    cy = pixel_height / 2;
    fx = focal_length_m / pixel_size_m;
    
    % Satellite position (arbitrary height above ground)
    sat_height = 500; % km (adjust as needed)
    sat_position = [0, 0, sat_height];
    
    % Ground plane size (based on satellite height and FOV)
    fov_angle = 2 * atan(pixel_width * pixel_size_m / (2 * focal_length_m)); % radians
    ground_extent = 2 * sat_height * tan(fov_angle/2);
    
    %% Create quaternion rotation matrix
    q = sat_quat / norm(sat_quat);
    qs = q(1); qx = q(2); qy = q(3); qz = q(4);
    
    R_body_to_eci = [1 - 2*(qy^2 + qz^2),  2*(qx*qy - qs*qz),  2*(qx*qz + qs*qy);
                     2*(qx*qy + qs*qz),  1 - 2*(qx^2 + qz^2),  2*(qy*qz - qs*qx);
                     2*(qx*qz - qs*qy),  2*(qy*qz + qs*qx),  1 - 2*(qx^2 + qy^2)];
    
    %% Plot ground plane with image
    subplot(1,2,1);
    hold on;
    
    % Create ground plane mesh
    [X_ground, Y_ground] = meshgrid(linspace(-ground_extent/2, ground_extent/2, 100));
    Z_ground = zeros(size(X_ground));
    
    % Map image to ground plane
    if ~isempty(image)
        % Flip image to match coordinate system
        flipped_image = flipud(image);
        
        % Create surface with image texture
        surf(X_ground, Y_ground, Z_ground, 'CData', flipped_image, ...
             'FaceColor', 'texturemap', 'EdgeColor', 'none', 'FaceAlpha', 0.8);
    else
        % If no image, create a simple ground plane
        surf(X_ground, Y_ground, Z_ground, 'FaceColor', [0.8 0.8 0.8], ...
             'EdgeColor', 'none', 'FaceAlpha', 0.5);
    end
    
    %% Draw satellite
    % Satellite body (simple box representation)
    sat_size = 20; % Adjust size as needed
    [X_sat, Y_sat, Z_sat] = box3d([-sat_size/2, sat_size/2], ...
                                   [-sat_size/2, sat_size/2], ...
                                   [-sat_size/4, sat_size/4]);
    
    % Transform satellite to position
    X_sat = X_sat + sat_position(1);
    Y_sat = Y_sat + sat_position(2);
    Z_sat = Z_sat + sat_height;
    
    % Draw satellite body
    surf(X_sat, Y_sat, Z_sat, 'FaceColor', [0.7 0.7 0.7], ...
         'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Draw solar panels (simplified)
    panel_width = sat_size * 2;
    panel_height = sat_size * 0.8;
    panel_thickness = 1;
    
    % Left panel
    [X_panel, Y_panel, Z_panel] = box3d([-panel_width-sat_size/2, -sat_size/2], ...
                                        [-panel_height/2, panel_height/2], ...
                                        [-panel_thickness/2, panel_thickness/2]);
    X_panel = X_panel + sat_position(1);
    Y_panel = Y_panel + sat_position(2);
    Z_panel = Z_panel + sat_height;
    surf(X_panel, Y_panel, Z_panel, 'FaceColor', [0 0 0.8], ...
         'EdgeColor', 'k', 'LineWidth', 1);
    
    % Right panel
    [X_panel, Y_panel, Z_panel] = box3d([sat_size/2, panel_width+sat_size/2], ...
                                        [-panel_height/2, panel_height/2], ...
                                        [-panel_thickness/2, panel_thickness/2]);
    X_panel = X_panel + sat_position(1);
    Y_panel = Y_panel + sat_position(2);
    Z_panel = Z_panel + sat_height;
    surf(X_panel, Y_panel, Z_panel, 'FaceColor', [0 0 0.8], ...
         'EdgeColor', 'k', 'LineWidth', 1);
    
    %% Draw camera FOV cone
    % Corner rays of the FOV
    corners = [0, 0; pixel_width, 0; pixel_width, pixel_height; 0, pixel_height; 0, 0];
    fov_lines_x = [];
    fov_lines_y = [];
    fov_lines_z = [];
    
    for i = 1:size(corners, 1)
        u = corners(i, 1);
        v = corners(i, 2);
        
        % Convert pixel to camera ray
        ray_cam = [(u - cx) / fx; -(v - cy) / fx; 1];
        ray_cam = ray_cam / norm(ray_cam);
        
        % Transform to world coordinates
        ray_world = R_body_to_eci * ray_cam;
        
        % Project to ground (assuming nadir pointing for visualization)
        t = -sat_height / ray_world(3);
        ground_point = sat_position' + t * ray_world;
        
        fov_lines_x = [fov_lines_x; sat_position(1); ground_point(1); NaN];
        fov_lines_y = [fov_lines_y; sat_position(2); ground_point(2); NaN];
        fov_lines_z = [fov_lines_z; sat_position(3); ground_point(3); NaN];
    end
    
    % Draw FOV cone
    plot3(fov_lines_x, fov_lines_y, fov_lines_z, 'b--', 'LineWidth', 1.5);
    
    %% Draw feature vectors
    n_features = size(ET_Measurement, 2);
    feature_colors = hsv(n_features); % Different color for each feature
    
    for i = 1:n_features
        % Get feature ray direction
        ray_direction = ET_Measurement(:, i);
        
        % Scale the ray for visualization
        ray_length = sat_height * 0.8; % Extend most of the way to ground
        ray_end = sat_position' + ray_length * ray_direction;
        
        % Draw ray from satellite to feature
        quiver3(sat_position(1), sat_position(2), sat_position(3), ...
                ray_direction(1)*ray_length, ray_direction(2)*ray_length, ...
                ray_direction(3)*ray_length, 0, ...
                'Color', feature_colors(i,:), 'LineWidth', 2, ...
                'MaxHeadSize', 0.5);
        
        % Mark feature location on ground (approximate)
        if ray_direction(3) < 0 % Only if pointing downward
            t_ground = -sat_height / ray_direction(3);
            ground_hit = sat_position' + t_ground * ray_direction;
            plot3(ground_hit(1), ground_hit(2), 0, 'o', ...
                  'MarkerSize', 8, 'MarkerFaceColor', feature_colors(i,:), ...
                  'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
        end
    end
    
    % Add coordinate axes at satellite
    axis_length = 50;
    quiver3(sat_position(1), sat_position(2), sat_position(3), ...
            axis_length, 0, 0, 0, 'r', 'LineWidth', 2); % X-axis
    quiver3(sat_position(1), sat_position(2), sat_position(3), ...
            0, axis_length, 0, 0, 'g', 'LineWidth', 2); % Y-axis
    quiver3(sat_position(1), sat_position(2), sat_position(3), ...
            0, 0, -axis_length, 0, 'b', 'LineWidth', 2); % Z-axis (pointing down)
    
    % Labels and formatting
    xlabel('X (km)', 'FontSize', 12);
    ylabel('Y (km)', 'FontSize', 12);
    zlabel('Z (km)', 'FontSize', 12);
    title('Satellite Field of View with Feature Vectors', 'FontSize', 14);
    grid on;
    axis equal;
    view(45, 30); % Adjust viewing angle
    set(gca, 'ZDir', 'reverse'); % Z pointing down
    
    % Add legend
    legend_entries = cell(n_features + 4, 1);
    legend_entries{1} = 'FOV Boundary';
    legend_entries{2} = 'X-axis';
    legend_entries{3} = 'Y-axis';
    legend_entries{4} = 'Z-axis';
    for i = 1:n_features
        legend_entries{i+4} = sprintf('Feature %d', i);
    end
    legend(legend_entries, 'Location', 'bestoutside');
    
    %% Subplot 2: Feature pixel locations on image
    subplot(1,2,2);
    if ~isempty(image)
        imshow(image);
        hold on;
        
        % Plot feature locations
        for i = 1:n_features
            plot(feature_pixel_locations(1,i), feature_pixel_locations(2,i), ...
                 'o', 'MarkerSize', 10, 'MarkerEdgeColor', feature_colors(i,:), ...
                 'MarkerFaceColor', feature_colors(i,:), 'LineWidth', 2);
            text(feature_pixel_locations(1,i)+10, feature_pixel_locations(2,i), ...
                 sprintf('%d', i), 'Color', feature_colors(i,:), ...
                 'FontSize', 12, 'FontWeight', 'bold');
        end
        
        title('Detected Features on Image', 'FontSize', 14);
        xlabel('Pixel X');
        ylabel('Pixel Y');
    end
    
    hold off;
end

%% Helper function to create 3D box
function [X, Y, Z] = box3d(x_range, y_range, z_range)
    % Create vertices of a box
    vertices = [
        x_range(1), y_range(1), z_range(1);
        x_range(2), y_range(1), z_range(1);
        x_range(2), y_range(2), z_range(1);
        x_range(1), y_range(2), z_range(1);
        x_range(1), y_range(1), z_range(2);
        x_range(2), y_range(1), z_range(2);
        x_range(2), y_range(2), z_range(2);
        x_range(1), y_range(2), z_range(2)
    ];
    
    % Define faces
    faces = [
        1 2 3 4;    % Bottom
        5 6 7 8;    % Top
        1 2 6 5;    % Front
        3 4 8 7;    % Back
        1 4 8 5;    % Left
        2 3 7 6     % Right
    ];
    
    % Create surface data
    X = zeros(4, 6);
    Y = zeros(4, 6);
    Z = zeros(4, 6);
    
    for i = 1:6
        face_vertices = vertices(faces(i,:), :);
        X(:,i) = face_vertices(:,1);
        Y(:,i) = face_vertices(:,2);
        Z(:,i) = face_vertices(:,3);
    end
end