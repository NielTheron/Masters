%==========================================================================
% Niel Theron
% 05-06-2025
%==========================================================================
% The purpose of this function is to take in the satellite image and work
% out the geolocations of the features of the system
%==========================================================================
% INPUT:
% image         : The satellite image
% r             : The position vector of the satellite (km,km,km) (3x1) (ECI)
% q             : The orientation of the satellite
% focalLenght   : The focal lenght of the camera (m)
% pixelSize     : The physical size of the pixel (m)
% n_f           : Number of features
%
% OUTPUT:
% featureGeoLocations : The geolocations of the feature points (deg,deg) (2xn_f)
%
% VARIABLES:
% grayImage     : A gray representation of the image
% imageHeight   : The height of the Image (Pixels)
% imageWidth    : The width of the Image (Pixels)
% lla           : The lat, long, altitude representation of the position
% vector
% satAlt        : The satellite altitude (m)
% GSD           : Ground sampling distance (m/pixel)
% points        : Feature detected points
% featurePixelLocation  : The pixel location of the feature point (pixel,pixel)
% dx            : Pixel off-set along track
% dy            : Pixel off-set cross track

%==========================================================================

function featureGeoLocations = FeatureGeoDetection(image, r, q, focalLength, pixelSize, n_f)
% image: RGB image
% satpos_ecef_km: satellite position in ECEF [km]
% focalLength_m: camera focal length in meters
% pixelSize_m: pixel size in meters
% heading_deg: sensor/satellite heading (clockwise from north) [degrees]
% n_f: number of features to detect

% Convert image to grayscale
grayImage = rgb2gray(image);
[imageHeight, imageWidth] = size(grayImage);

% Estimate altitude (satellite height above surface) in meters
lla = ecef2lla(r.' * 1000, "WGS84"); % Convert km to meters
satAlt = lla(3);  % Altitude in meters

% Calculate GSD [m/pixel]
GSD = (pixelSize * satAlt) / focalLength;

% Detect SURF features
points = detectSURFFeatures(grayImage);
featurePixelLocations = points.selectStrongest(n_f);

% Get pixel coordinates
xy = featurePixelLocations.Location;

% Compute offsets from image center (in pixels)
dx = xy(:,1) - imageWidth/2;
dy = imageHeight/2 - xy(:,2); % y is inverted

% Convert pixel offsets to meters
dx_m = dx * GSD;  % x is image right
dy_m = dy * GSD;  % y is image up

% Rotate offsets according to heading
eul_body_to_enu = quat2eul(q.', 'XYZ');
theta = eul_body_to_enu(1);
east_offsets  = dx_m * cos(theta) - dy_m * sin(theta); % East direction
north_offsets = dx_m * sin(theta) + dy_m * cos(theta); % North direction

% Convert satellite ECEF to geodetic lat/lon/alt
refLat = lla(1);
refLon = lla(2);
refAlt = lla(3); % Altitude in meters

% Assume flat ground at same altitude as nadir point for ENU conversion
up_offsets = zeros(size(east_offsets)); % flat ground

% Convert ENU to lat/lon using mapping toolbox
[lat, lon, ~] = enu2geodetic(east_offsets(:), north_offsets(:), up_offsets(:), ...
                             refLat, refLon, refAlt, wgs84Ellipsoid());


featureGeoLocations = [lat, lon].';
end
