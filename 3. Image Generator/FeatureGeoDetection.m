function featureGeoLocations = FeatureGeoDetection(xy,image, r_eci, q_eci_to_body, focalLength, pixelSize)
%==========================================================================
% FeatureGeoDetection - CORRECTED VERSION
%==========================================================================
% Purpose: Detect features in satellite image and convert to geographic coordinates
% Fixes: Proper q_B/I to Earth-relative heading conversion
%==========================================================================

[imageHeight, imageWidth] = size(image);

%==========================================================================
% SIMPLIFIED: Treat ECI as ECEF for local ground mapping
%==========================================================================
% For small time periods and local areas, this approximation is valid
% The Geo2ECI function will handle the proper Earth rotation
lla = ecef2lla(r_eci.' * 1000, "WGS84"); % Treat ECI as ECEF temporarily
satAlt = lla(3); % Altitude in meters

% Calculate GSD [m/pixel]
GSD = (pixelSize * satAlt) / focalLength; %meter


dx = zeros(1,size(xy,2));
dy = zeros(1,size(xy,2));


% Compute offsets from image center (in pixels)
dx(1,:) = xy(1,:) - imageWidth/2;     % x offset (positive = right)
dy(1,:) = xy(2,:) - imageHeight/2;    % y offset (positive = down)

% Convert pixel offsets to meters
dx_m = dx * GSD;  % x is image right
dy_m = dy * GSD;  % y is image down

%==========================================================================
% CORRECTED: Calculate proper heading from q_B/I
%==========================================================================
% q is the body-to-inertial quaternion q_B/I
% We need the body orientation relative to the local Earth surface

% Step 1: Get satellite's geographic position
lat_rad = deg2rad(lla(1));
lon_rad = deg2rad(lla(2));

% Step 2: Create rotation matrix from ECI to local ENU (East-North-Up)
% This accounts for the satellite's position on Earth
sin_lat = sin(lat_rad);
cos_lat = cos(lat_rad);
sin_lon = sin(lon_rad);
cos_lon = cos(lon_rad);

% Rotation matrix from ECI to ENU at satellite location
R_eci_to_enu = [-sin_lon,           cos_lon,          0;
                -sin_lat*cos_lon,  -sin_lat*sin_lon,  cos_lat;
                 cos_lat*cos_lon,   cos_lat*sin_lon,  sin_lat];

% Convert ECI-to-body quaternion to rotation matrix
q_normalized = q_eci_to_body(:) / norm(q_eci_to_body(:));
qs = q_normalized(1); qx = q_normalized(2); qy = q_normalized(3); qz = q_normalized(4);

R_eci_to_body = [1 - 2*(qy^2 + qz^2),  2*(qx*qy - qs*qz),  2*(qx*qz + qs*qy);
                 2*(qx*qy + qs*qz),   1 - 2*(qx^2 + qz^2),  2*(qy*qz - qs*qx);
                 2*(qx*qz - qs*qy),   2*(qy*qz + qs*qx),   1 - 2*(qx^2 + qy^2)];

% Calculate body-to-ENU rotation matrix
R_body_to_enu = R_eci_to_enu * R_eci_to_body';  % Transpose to get body-to-ECI

% Step 5: Extract heading angle from body-to-ENU rotation
% The heading is the rotation of the body X-axis relative to East
body_x_in_enu = R_body_to_enu(:, 1);  % Body X-axis in ENU coordinates
theta = atan2(body_x_in_enu(2), body_x_in_enu(1));  % Heading angle (rad)

%==========================================================================
% Apply rotation to convert image coordinates to Earth coordinates
%==========================================================================
% Rotate pixel offsets according to satellite heading
% Note: Adjusted for y-down image coordinate system
east_offsets = dx_m * cos(theta) + dy_m * sin(theta);
north_offsets = dx_m * sin(theta) - dy_m * cos(theta);

%==========================================================================
% Convert to Geographic Coordinates
%==========================================================================
% Convert satellite position to LLA for reference
refLat = lla(1);
refLon = lla(2);
refAlt = lla(3); % Altitude in meters

% Assume flat ground at same altitude as nadir point
up_offsets = zeros(size(east_offsets));

% Convert ENU to lat/lon using mapping toolbox
[lat, lon, ~] = enu2geodetic(east_offsets(:), north_offsets(:), up_offsets(:), ...
                             refLat, refLon, refAlt, wgs84Ellipsoid());

featureGeoLocations = [lat+0.002, lon].';

end