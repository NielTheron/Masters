function ET_measurement = EarthTracker(feature, sat_pos_eci, sat_quat, focal_length_m, pixel_size_m, pixel_height, pixel_width)
%==========================================================================
% EarthTracker with Earth Surface Intersection
% Computes actual vectors from satellite to features on Earth's surface
% Niel Theron - Enhanced version
%==========================================================================
% Inputs:
%   feature        - Feature pixel locations [Nx2] or [2xN]
%   sat_pos_eci    - Satellite position in ECI [3x1] (km)
%   sat_quat       - Satellite attitude quaternion [4x1]
%   focal_length_m - Camera focal length (m)
%   pixel_size_m   - Pixel size (m)
%   pixel_height   - Image height in pixels
%   pixel_width    - Image width in pixels
%
% Output:
%   ET_measurement - Feature vectors in body frame [3xN] (km)
%==========================================================================

% Earth parameters
R_earth = 6371; % Earth radius in km

% Determine number of features
if size(feature, 1) == 2
    N = size(feature, 2);
    feature = feature'; % Convert to Nx2
else
    N = size(feature, 1);
end

ET_measurement = zeros(3, N);

% Camera parameters
cx = pixel_width / 2;
cy = pixel_height / 2;
fx = focal_length_m / pixel_size_m;

% Quaternion to rotation matrix (body to ECI)
q = sat_quat / norm(sat_quat);
qs = q(1); qx = q(2); qy = q(3); qz = q(4);

R_body_to_eci = [1 - 2*(qy^2 + qz^2),  2*(qx*qy - qs*qz),  2*(qx*qz + qs*qy);
                 2*(qx*qy + qs*qz),  1 - 2*(qx^2 + qz^2),  2*(qy*qz - qs*qx);
                 2*(qx*qz - qs*qy),  2*(qy*qz + qs*qx),  1 - 2*(qx^2 + qy^2)];

% Process each feature
for i = 1:N
    u = feature(i, 1);
    v = feature(i, 2);
    
    % Convert pixel to camera ray (normalized)
    ray_cam = [(u - cx) / fx; -(v - cy) / fx; 1];
    ray_cam = ray_cam / norm(ray_cam);
    
    % Transform ray to ECI frame
    ray_eci = R_body_to_eci * ray_cam;
    
    % Find intersection with Earth surface
    % Solve: ||sat_pos + t*ray|| = R_earth
    % This gives quadratic: t^2 + 2*t*(sat_pos·ray) + (||sat_pos||^2 - R_earth^2) = 0
    
    a = dot(ray_eci, ray_eci); % Should be 1 if normalized
    b = 2 * dot(sat_pos_eci, ray_eci);
    c = dot(sat_pos_eci, sat_pos_eci) - R_earth^2;
    
    discriminant = b^2 - 4*a*c;
    
    if discriminant < 0
        % Ray doesn't intersect Earth - feature is beyond horizon
        fprintf('Warning: Feature %d ray does not intersect Earth\n', i);
        ET_measurement(:, i) = [NaN; NaN; NaN];
        continue;
    end
    
    % Take the closer intersection (smaller t)
    t1 = (-b - sqrt(discriminant)) / (2*a);
    t2 = (-b + sqrt(discriminant)) / (2*a);
    
    % Use the intersection in front of the satellite
    if t1 > 0
        t = t1;
    elseif t2 > 0
        t = t2;
    else
        % Both intersections behind satellite
        fprintf('Warning: Feature %d is behind satellite\n', i);
        ET_measurement(:, i) = [NaN; NaN; NaN];
        continue;
    end
    
    % Calculate intersection point in ECI
    intersection_eci = sat_pos_eci + t * ray_eci;
    
    % Vector from satellite to feature in ECI
    vector_eci = intersection_eci - sat_pos_eci;
    
    % Transform back to body frame
    ET_measurement(:, i) = R_body_to_eci' * vector_eci;
end

% Display statistics
valid_features = ~isnan(ET_measurement(1,:));
if any(valid_features)
    distances = sqrt(sum(ET_measurement(:,valid_features).^2, 1));
    fprintf('\nEarthTracker Statistics:\n');
    fprintf('  Valid features: %d/%d\n', sum(valid_features), N);
    fprintf('  Satellite altitude: %.2f km\n', norm(sat_pos_eci) - R_earth);
    fprintf('  Range to features: %.2f - %.2f km\n', min(distances), max(distances));
    fprintf('  Mean range: %.2f km\n', mean(distances));
end

end