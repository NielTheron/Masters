function ET_measurement = EarthTracker1(feature, sat_pos_eci, sat_quat, focal_length_m, pixel_size_m, pixel_height, pixel_width)
%==========================================================================
% Fixed EarthTracker - Uses Proper Ray Intersection with Earth
%==========================================================================
% Inputs:
% - feature        : [N x 2] matrix of [x, y] pixel locations
% - sat_pos_eci    : [3 x 1] satellite position in ECI (km)
% - sat_quat       : [4 x 1] quaternion body-to-inertial (scalar first: [w x y z])
% - focal_length_m : Focal length in meters
% - pixel_size_m   : Pixel size in meters  
% - pixel_height   : Image height (pixels)
% - pixel_width    : Image width (pixels)
%
% Output:
% - ET_measurement : [3 x N] matrix of feature vectors in camera frame (km)
%==========================================================================

% Input validation and debugging
if isempty(feature)
    warning('EarthTracker: No features provided');
    ET_measurement = [];
    return;
end

% Ensure proper dimensions
sat_pos_eci = sat_pos_eci(:);  % Force column vector
sat_quat = sat_quat(:);        % Force column vector

% Validate satellite position
R_earth = 6378;  % km
sat_altitude = norm(sat_pos_eci) - R_earth;
if sat_altitude < 0
    error('EarthTracker: Satellite appears to be below Earth surface (alt = %.2f km)', sat_altitude);
elseif sat_altitude < 100
    warning('EarthTracker: Very low satellite altitude (%.2f km)', sat_altitude);
end

% Validate quaternion
sat_quat = sat_quat / norm(sat_quat);  % Normalize quaternion
if abs(norm(sat_quat) - 1.0) > 1e-6
    warning('EarthTracker: Quaternion normalization issue');
end

N = size(feature, 1);
ET_measurement = zeros(3, N);

% Camera intrinsic parameters
cx = pixel_width / 2;   % Principal point x
cy = pixel_height / 2;  % Principal point y
fx = focal_length_m / pixel_size_m;  % Focal length in pixels
fy = fx;  % Assume square pixels

% Debug camera parameters
fprintf('Camera Parameters:\n');
fprintf('  Focal length: %.6f m (%.1f pixels)\n', focal_length_m, fx);
fprintf('  Pixel size: %.2e m\n', pixel_size_m);
fprintf('  Image size: %d x %d pixels\n', pixel_width, pixel_height);
fprintf('  Principal point: (%.1f, %.1f)\n', cx, cy);

% Convert quaternion to rotation matrix
% Assuming quaternion format: [w, x, y, z] (scalar first)
q = sat_quat;
qs = q(1); qx = q(2); qy = q(3); qz = q(4);

% Body-to-ECI rotation matrix
R_body_to_eci = [1 - 2*(qy^2 + qz^2),  2*(qx*qy - qs*qz),  2*(qx*qz + qs*qy);
                 2*(qx*qy + qs*qz),  1 - 2*(qx^2 + qz^2),  2*(qy*qz - qs*qx);
                 2*(qx*qz - qs*qy),  2*(qy*qz + qs*qx),  1 - 2*(qx^2 + qy^2)];

% Verify rotation matrix is orthogonal
if abs(det(R_body_to_eci) - 1.0) > 1e-6
    warning('EarthTracker: Rotation matrix determinant = %.6f (should be 1.0)', det(R_body_to_eci));
end

% Debug satellite state
fprintf('Satellite State:\n');
fprintf('  Position ECI: [%.1f, %.1f, %.1f] km\n', sat_pos_eci);
fprintf('  Altitude: %.1f km\n', sat_altitude);
fprintf('  Quaternion: [%.4f, %.4f, %.4f, %.4f]\n', sat_quat);

% Initialize counters for debugging
valid_intersections = 0;
forward_rays = 0;

for i = 1:N
    % Get pixel coordinates
    u = feature(i, 1);
    v = feature(i, 2);
    
    % Check if pixel coordinates are within image bounds
    if u < 0 || u >= pixel_width || v < 0 || v >= pixel_height
        warning('EarthTracker: Feature %d pixel coordinates [%.1f, %.1f] outside image bounds', i, u, v);
        continue;
    end
    
    % Convert pixel to normalized camera coordinates
    % Standard pinhole camera model
    x_norm = (u - cx) / fx;
    y_norm = (v - cy) / fy;
    
    % Ray direction in camera frame
    % Camera looks along +Z axis, with +X right and +Y down
    ray_cam = [x_norm; y_norm; 1];  % Forward ray along +Z
    ray_cam = ray_cam / norm(ray_cam);  % Normalize
    
    % Transform ray to ECI frame
    ray_eci = R_body_to_eci * ray_cam;
    
    % Ray-sphere intersection with Earth
    % Ray equation: P(t) = sat_pos_eci + t * ray_eci
    % Sphere equation: |P(t)|^2 = R_earth^2
    
    % Expand: |sat_pos + t*ray|^2 = R_earth^2
    % (sat_pos + t*ray) · (sat_pos + t*ray) = R_earth^2
    % sat_pos·sat_pos + 2*t*(sat_pos·ray) + t^2*(ray·ray) = R_earth^2
    
    a = dot(ray_eci, ray_eci);  % Should be ≈1 for normalized ray
    b = 2 * dot(sat_pos_eci, ray_eci);
    c = dot(sat_pos_eci, sat_pos_eci) - R_earth^2;
    
    discriminant = b^2 - 4*a*c;
    
    if discriminant >= 0
        % Two intersection points
        sqrt_disc = sqrt(discriminant);
        t1 = (-b - sqrt_disc) / (2*a);  % Closer intersection
        t2 = (-b + sqrt_disc) / (2*a);  % Farther intersection
        
        % Choose the positive intersection closest to the satellite
        if t1 > 0 && t2 > 0
            t = min(t1, t2);  % Both positive, take closer
        elseif t1 > 0
            t = t1;
        elseif t2 > 0
            t = t2;
        else
            t = -1;  % No forward intersection
        end
        
        if t > 0
            forward_rays = forward_rays + 1;
            
            % Calculate intersection point in ECI
            ground_point_eci = sat_pos_eci + t * ray_eci;
            
            % Verify intersection is on Earth surface
            distance_from_center = norm(ground_point_eci);
            if abs(distance_from_center - R_earth) > 1.0  % 1 km tolerance
                warning('EarthTracker: Feature %d intersection error: distance = %.2f km', i, distance_from_center);
            end
            
            % Convert to camera frame (relative to satellite)
            ground_relative_eci = ground_point_eci - sat_pos_eci;
            R_eci_to_body = R_body_to_eci';  % Transpose for inverse rotation
            ET_measurement(:, i) = R_eci_to_body * ground_relative_eci;
            
            valid_intersections = valid_intersections + 1;
            
            % Debug first few features
            if i <= 3
                fprintf('Feature %d:\n', i);
                fprintf('  Pixel: [%.1f, %.1f]\n', u, v);
                fprintf('  Ray (cam): [%.4f, %.4f, %.4f]\n', ray_cam);
                fprintf('  Ray (ECI): [%.4f, %.4f, %.4f]\n', ray_eci);
                fprintf('  Intersection distance: %.1f km\n', t);
                fprintf('  Ground point: [%.1f, %.1f, %.1f] km\n', ground_point_eci);
                fprintf('  Measurement (cam): [%.1f, %.1f, %.1f] km\n', ET_measurement(:, i));
            end
        else
            % Ray doesn't hit Earth in forward direction
            ET_measurement(:, i) = [0; 0; 0];
            if i <= 3
                fprintf('Feature %d: No forward intersection (t1=%.2f, t2=%.2f)\n', i, t1, t2);
            end
        end
    else
        % No intersection with Earth sphere
        ET_measurement(:, i) = [0; 0; 0];
        if i <= 3
            fprintf('Feature %d: No intersection (discriminant=%.2f)\n', i, discriminant);
        end
    end
end

% Summary statistics
fprintf('\nEarthTracker Results:\n');
fprintf('  Total features: %d\n', N);
fprintf('  Forward rays: %d\n', forward_rays);
fprintf('  Valid intersections: %d\n', valid_intersections);
fprintf('  Success rate: %.1f%%\n', 100 * valid_intersections / N);

% Check if all measurements are zero
if all(ET_measurement(:) == 0)
    warning('EarthTracker: All measurements are zero! Check satellite position and camera orientation.');
    
    % Additional diagnostic information
    fprintf('\nDiagnostic Information:\n');
    
    % Check if satellite can see Earth
    sat_to_earth = -sat_pos_eci;  % Vector from sat to Earth center
    sat_to_earth_unit = sat_to_earth / norm(sat_to_earth);
    
    % Camera boresight in ECI frame (assuming camera looks along +Z in body frame)
    camera_boresight_body = [0; 0; 1];
    camera_boresight_eci = R_body_to_eci * camera_boresight_body;
    
    angle_to_earth = acosd(dot(camera_boresight_eci, sat_to_earth_unit));
    fprintf('  Camera boresight angle to Earth center: %.1f degrees\n', angle_to_earth);
    
    % Earth angular radius as seen from satellite
    earth_angular_radius = asind(R_earth / norm(sat_pos_eci));
    fprintf('  Earth angular radius from satellite: %.1f degrees\n', earth_angular_radius);
    
    if angle_to_earth > earth_angular_radius + 10  % 10 degree margin
        fprintf('  -> Camera is likely pointing away from Earth!\n');
    end
    
    % Check camera FOV
    fov_x = 2 * atand((pixel_width * pixel_size_m / 2) / focal_length_m);
    fov_y = 2 * atand((pixel_height * pixel_size_m / 2) / focal_length_m);
    fprintf('  Camera FOV: %.1f x %.1f degrees\n', fov_x, fov_y);
end

end