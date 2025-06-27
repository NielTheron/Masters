function Mag_B_est = Magnetometer(r_ECI, attitude, noise_std_deg, mag_angle, t, we)
% Simulate a magnetometer that points toward geographic North Pole,
% tangent to Earth's surface, with optional magnetic dip and noise.
%
% Inputs:
%   r_ECI          - Satellite position vector in ECI frame [3x1] (km)
%   attitude       - Spacecraft attitude quaternion [4x1] (scalar-first: [qs qx qy qz])
%   inclination_deg- Magnetic dip angle (degrees) - positive points downward
%   noise_std_deg  - Standard deviation of noise in degrees
%   t              - Time since epoch (seconds)
%   we             - Earth rotation rate (rad/s)
%
% Output:
%   Mag_B_est      - Normalized magnetometer vector in body frame [3x1]
%
% Magnetic field behavior:
%   - At equator (lat=0°): points toward North Pole (parallel to Z_ECI)
%   - At lat=45°: points 45° toward North Pole, tangent to surface
%   - At North Pole: points straight down into Earth
%==================================================================

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

%% Step 6: Convert ECI to body frame using attitude quaternion
% Normalize quaternion to ensure unit quaternion
attitude = attitude(:) / norm(attitude(:));

% Convert quaternion to rotation matrix (ECI to body)
% Assuming attitude is body-to-inertial quaternion [qs qx qy qz]
qs = attitude(1); qx = attitude(2); qy = attitude(3); qz = attitude(4);

R_ECI2Body = [1-2*(qy^2+qz^2),  2*(qx*qy-qs*qz),  2*(qx*qz+qs*qy);
              2*(qx*qy+qs*qz),  1-2*(qx^2+qz^2),  2*(qy*qz-qs*qx);
              2*(qx*qz-qs*qy),  2*(qy*qz+qs*qx),  1-2*(qx^2+qy^2)].';

mag_field_body = R_ECI2Body * mag_field_ECI;

%% Step 7: Add measurement noise
if noise_std_deg > 0
    % Generate random rotation axis
    noise_axis = randn(3,1);
    noise_axis = noise_axis / norm(noise_axis);
    
    % Generate noise angle
    noise_angle = deg2rad(noise_std_deg) * randn();
    
    % Create noise quaternion
    q_noise = [cos(noise_angle/2); noise_axis * sin(noise_angle/2)];
    
    % Apply noise rotation to magnetic field vector
    mag_field_body = quatrotate(q_noise', mag_field_body')';
end

%% Step 8: Normalize and return result
Mag_B_est = mag_field_body / norm(mag_field_body);

end