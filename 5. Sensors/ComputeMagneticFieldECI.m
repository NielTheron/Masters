function mag_field_ECI = ComputeMagneticFieldECI(r_ECI, mag_angle, t, we)
%% Step 1: Convert ECI position to geographic coordinates
% Convert from ECI to ECEF (Earth-Centered Earth-Fixed)
theta_earth = we * t;  % Earth rotation angle
R_ECI2ECEF = [cos(theta_earth), sin(theta_earth), 0;
             -sin(theta_earth), cos(theta_earth), 0;
              0,                0,               1];

r_ECEF = R_ECI2ECEF * r_ECI * 1000;  % Convert km to meters

% Convert ECEF to geodetic coordinates
[lat_deg, lon_deg, alt_m] = ecef2geodetic(wgs84Ellipsoid("meter"),r_ECEF(1), r_ECEF(2), r_ECEF(3));
lat_rad = deg2rad(lat_deg);
lon_rad = deg2rad(lon_deg);

%% Step 2: Calculate direction from satellite to North Pole in ECEF
% North Pole position in ECEF coordinates
north_pole_ECEF = [0; 0; 6378137];  % Approximately on Z-axis in meters

% Vector from satellite to North Pole in ECEF
to_north_pole_ECEF = north_pole_ECEF - r_ECEF;
to_north_pole_ECEF = to_north_pole_ECEF / norm(to_north_pole_ECEF);  % Normalize

%% Step 3: Make the field tangent to Earth's surface
% Calculate the local "up" vector (radially outward from Earth center)
up_ECEF = r_ECEF / norm(r_ECEF);

% Project the north pole direction onto the plane tangent to Earth's surface
% This removes the radial component, making it tangent to the surface
mag_field_tangent_ECEF = to_north_pole_ECEF - dot(to_north_pole_ECEF, up_ECEF) * up_ECEF;
mag_field_tangent_ECEF = mag_field_tangent_ECEF / norm(mag_field_tangent_ECEF);

%% Step 4: Apply magnetic inclination (dip angle)
% Inclination rotates the field away from horizontal
% Positive inclination tips the field downward (toward Earth center)
if mag_angle ~= 0
    dip_rad = deg2rad(mag_angle);
    
    % The dip rotation is around the east direction (perpendicular to both up and north)
    east_ECEF = cross(up_ECEF, mag_field_tangent_ECEF);
    east_ECEF = east_ECEF / norm(east_ECEF);
    
    % Rodrigues' rotation formula to rotate around east axis
    cos_dip = cos(dip_rad);
    sin_dip = sin(dip_rad);
    
    % Rotate the tangent field toward the up direction by inclination angle
    mag_field_ECEF = cos_dip * mag_field_tangent_ECEF + sin_dip * cross(east_ECEF, mag_field_tangent_ECEF);
else
    mag_field_ECEF = mag_field_tangent_ECEF;
end

%% Step 5: Convert ECEF to ECI frame
R_ECEF2ECI = R_ECI2ECEF';  % Transpose to get inverse rotation
mag_field_ECI = R_ECEF2ECI * mag_field_ECEF;

end