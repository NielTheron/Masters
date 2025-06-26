function featureGeoLocations = FeatureGeolocation(features, image, r_I, q_I2B, focalLength, pixelSize)
%==========================================================================
% FeatureGeolocation - Pinhole Camera Model Geolocation
%==========================================================================
% Purpose: Convert detected features to geographic coordinates using
%          pinhole camera model and satellite pose
%
% Inputs:
%   features      - [2×n] matrix of pixel coordinates [x; y]
%   image         - Satellite image (for dimensions)
%   r_I           - Satellite position in ECI frame [3×1] (km)
%   q_I2B         - Inertial-to-body quaternion [4×1] [qs, qx, qy, qz]
%   focalLength   - Camera focal length (m)
%   pixelSize     - Pixel size (m)
%
% Output:
%   featureGeoLocations - [3×n] matrix of [lat; lon; alt] in degrees
%
% Author: D.P. Theron
% Date: June 2025
%==========================================================================

%% Get Image Dimensions ==================================================
if ndims(image) == 3
    [imageHeight, imageWidth, ~] = size(image);
else
    [imageHeight, imageWidth] = size(image);
end

%% Validate Inputs =======================================================
if size(features, 1) ~= 2
    error('features must be a [2×n] matrix');
end
numFeatures = size(features, 2);

%% Calculate Satellite Altitude ==========================================
% Convert satellite position to geodetic coordinates
r_ecef = r_I * 1000;  % Convert km to meters
lla_sat = ecef2lla(r_ecef.', 'WGS84');
satAlt = lla_sat(3);  % Altitude in meters
satLat = lla_sat(1);  % Latitude in degrees
satLon = lla_sat(2);  % Longitude in degrees

%% Calculate Ground Sample Distance ======================================
GSD = (pixelSize * satAlt) / focalLength;  % meters/pixel

%% Initialize Output =====================================================
featureGeoLocations = zeros(3, numFeatures);

%% Compute Camera Orientation ============================================
% Extract quaternion components and normalize
q = q_I2B(:) / norm(q_I2B(:));
qs = q(1); qx = q(2); qy = q(3); qz = q(4);

% Build rotation matrix from inertial to body frame
R_I2B = [
    1 - 2*(qy^2 + qz^2),  2*(qx*qy - qs*qz),  2*(qx*qz + qs*qy);
    2*(qx*qy + qs*qz),   1 - 2*(qx^2 + qz^2),  2*(qy*qz - qs*qx);
    2*(qx*qz - qs*qy),   2*(qy*qz + qs*qx),   1 - 2*(qx^2 + qy^2)
];

%% Process Each Feature ==================================================
for i = 1:numFeatures
    % Get pixel coordinates (1-based indexing)
    px = features(1, i);
    py = features(2, i);
    
    % Convert to image plane coordinates (camera frame)
    % Origin at image center, x-right, y-down
    x_img = (px - imageWidth/2) * pixelSize;   % meters
    y_img = (py - imageHeight/2) * pixelSize;  % meters
    
    % Create ray vector in camera/body frame
    % Camera looks along +Z in body frame
    ray_body = [x_img; y_img; focalLength];
    ray_body = ray_body / norm(ray_body);  % Normalize
    
    % Transform ray to ECI frame
    ray_eci = R_I2B * ray_body;  % R_I2B' = R_B2I
    
    % Intersect ray with Earth surface (simplified spherical Earth)
    % Ray equation: P = r_I + t * ray_eci
    % Sphere equation: ||P|| = R_earth
    
    R_earth = 6371;  % Earth radius in km
    
    % Solve quadratic equation for intersection
    a = dot(ray_eci, ray_eci);
    b = 2 * dot(r_I, ray_eci);
    c = dot(r_I, r_I) - R_earth^2;
    
    discriminant = b^2 - 4*a*c;
    
    % Take the closer intersection (smaller t)
    t1 = (-b - sqrt(discriminant)) / (2*a);
    t2 = (-b + sqrt(discriminant)) / (2*a);
    
    % Choose the intersection in front of the satellite
    if t1 > 0
        t = t1;
    elseif t2 > 0
        t = t2;
    end
    
    % Calculate intersection point in ECI
    P_eci = r_I + t * ray_eci;  % km
    
    % Convert to ECEF (assuming no Earth rotation for now)
    % For better accuracy, you should account for Earth rotation
    P_ecef = P_eci * 1000;  % Convert to meters
    
    % Convert ECEF to geodetic coordinates
    lla = ecef2lla(P_ecef.', 'WGS84');
    
    featureGeoLocations(:, i) = [lla(1); lla(2); lla(3)/1000];  % lat, lon (deg), alt (km)
end

end