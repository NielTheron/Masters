function ET_measurement = EarthTracker(feature, sat_quat, focal_length_m, pixel_size_m, pixel_height, pixel_width)
%==========================================================================
% Simplified EarthTracker - Direct ray projection
% Niel Theron - 12-06-2025
%==========================================================================

N = size(feature, 1);
ET_measurement = zeros(3, N);

% Camera parameters
cx = pixel_width / 2;
cy = pixel_height / 2;
fx = focal_length_m / pixel_size_m;

% Quaternion to rotation matrix
q = sat_quat / norm(sat_quat);
qs = q(1); qx = q(2); qy = q(3); qz = q(4);

R_body_to_eci = [1 - 2*(qy^2 + qz^2),  2*(qx*qy - qs*qz),  2*(qx*qz + qs*qy);
                 2*(qx*qy + qs*qz),  1 - 2*(qx^2 + qz^2),  2*(qy*qz - qs*qx);
                 2*(qx*qz - qs*qy),  2*(qy*qz + qs*qx),  1 - 2*(qx^2 + qy^2)];

for i = 1:N
    u = feature(i, 1);
    v = feature(i, 2);
    
    % Convert pixel to camera ray
    ray_cam = [(u - cx) / fx; -(v - cy) / fx; 1];
    ray_cam = ray_cam / norm(ray_cam);
    
    % Transform to ECI and back to body frame
    ray_eci = R_body_to_eci * ray_cam;
    ET_measurement(:, i) = R_body_to_eci' * ray_eci;
end

end