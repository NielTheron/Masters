function ET_measurement = EarthTracker(feature, sat_pos_eci, sat_quat, focal_length_m, pixel_size_m, pixel_height, pixel_width)
%==========================================================================
% Improved EarthTracker - Uses Proper Ray Intersection with Earth
%==========================================================================
% Inputs:
% - feature        : [N x 2] matrix of [x, y] pixel locations
% - sat_pos_eci    : [3 x 1] satellite position in ECI (km)
% - sat_quat       : [4 x 1] quaternion body-to-inertial
% - focal_length_m : Focal length in meters
% - pixel_size_m   : Pixel size in meters  
% - pixel_height   : Image height (pixels)
% - pixel_width    : Image width (pixels)
%
% Output:
% - ET_measurement : [3 x N] matrix of feature vectors in camera frame (km)
%==========================================================================

N = size(feature, 1);
ET_measurement = zeros(3, N);

% Camera intrinsic parameters
cx = pixel_width / 2;   % Principal point x
cy = pixel_height / 2;  % Principal point y
fx = focal_length_m / pixel_size_m;  % Focal length in pixels
fy = fx;  % Assume square pixels

for i = 1:N
    % Convert pixel to normalized camera coordinates
    u = feature(i, 1);
    v = feature(i, 2);
    
    % Ray direction in camera frame (normalized)
    ray_cam = [(u - cx) / fx; -(v - cy) / fy; 1];  % Note: -v for image coordinate flip
    ray_cam = ray_cam / norm(ray_cam);
    
    % Transform ray to ECI frame using quaternion
    % First convert quaternion to rotation matrix (body-to-inertial)
    q = sat_quat / norm(sat_quat);  % Normalize
    qs = q(1); qx = q(2); qy = q(3); qz = q(4);
    
    R_body_to_eci = [1 - 2*(qy^2 + qz^2),  2*(qx*qy - qs*qz),  2*(qx*qz + qs*qy);
                     2*(qx*qy + qs*qz),  1 - 2*(qx^2 + qz^2),  2*(qy*qz - qs*qx);
                     2*(qx*qz - qs*qy),  2*(qy*qz + qs*qx),  1 - 2*(qx^2 + qy^2)];
    
    % Transform ray to ECI frame (camera frame = body frame)
    ray_eci = R_body_to_eci * ray_cam;
    
    % Ray-sphere intersection with Earth
    % Ray: P = sat_pos_eci + t * ray_eci
    % Sphere: |P|^2 = R_earth^2
    R_earth = 6378;  % km
    
    % Solve quadratic equation: |sat_pos + t*ray|^2 = R_earth^2
    a = dot(ray_eci, ray_eci);  % Should be 1 for normalized ray
    b = 2 * dot(sat_pos_eci, ray_eci);
    c = dot(sat_pos_eci, sat_pos_eci) - R_earth^2;
    
    discriminant = b^2 - 4*a*c;
    
    if discriminant >= 0
        % Take the closer intersection (smaller t)
        t1 = (-b - sqrt(discriminant)) / (2*a);
        t2 = (-b + sqrt(discriminant)) / (2*a);
        t = min(t1, t2);  % Closer intersection
        
        if t > 0  % Ray hits Earth in forward direction
            % Ground point in ECI
            ground_point_eci = sat_pos_eci + t * ray_eci;
            
            % Convert back to camera frame
            ground_relative_eci = ground_point_eci - sat_pos_eci;
            R_eci_to_body = R_body_to_eci';  % Transpose for inverse
            ET_measurement(:, i) = R_eci_to_body * ground_relative_eci;
        else
            % Ray doesn't hit Earth - set to zero
            ET_measurement(:, i) = [0; 0; 0];
        end
    else
        % No intersection with Earth - set to zero  
        ET_measurement(:, i) = [0; 0; 0];
    end
end

end