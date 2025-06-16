function [r_eci, v_eci] = InitialiseOrbitWithOrientation(lat_deg, lon_deg, alt_km, i_deg, RAAN_deg, argPerigee_deg)
%==========================================================================
% Initialize circular orbit with specified position and orientation
% Position defined by lat/lon/alt, velocity computed for circular orbit
%==========================================================================

% Constants
mu = 398600.4418;       % km^3/s^2
Re = 6378.137;          % km

% Convert degrees to radians
lat = deg2rad(lat_deg);
lon = deg2rad(lon_deg);
i = deg2rad(i_deg);
RAAN = deg2rad(RAAN_deg);
w = deg2rad(argPerigee_deg);

% Step 1: Position from lat/lon/alt (assuming t=0, so ECI ≈ ECEF)
r_eci = [(Re + alt_km)*cos(lat)*cos(lon);
         (Re + alt_km)*cos(lat)*sin(lon);
         (Re + alt_km)*sin(lat)];

% Step 2: For circular orbit, velocity magnitude
r_mag = norm(r_eci);
v_mag = sqrt(mu / r_mag);

% Step 3: Build rotation matrix from perifocal to ECI
% R = R3(RAAN) * R1(i) * R3(w)
R_pqw_to_eci = [cos(RAAN)*cos(w)-sin(RAAN)*sin(w)*cos(i), -cos(RAAN)*sin(w)-sin(RAAN)*cos(w)*cos(i),  sin(RAAN)*sin(i);
                sin(RAAN)*cos(w)+cos(RAAN)*sin(w)*cos(i), -sin(RAAN)*sin(w)+cos(RAAN)*cos(w)*cos(i), -cos(RAAN)*sin(i);
                sin(w)*sin(i),                               cos(w)*sin(i),                               cos(i)];

% Step 4: Find true anomaly from current position
% Project position into perifocal frame
r_pqw = R_pqw_to_eci' * r_eci;
nu = atan2(r_pqw(2), r_pqw(1));

% Step 5: Velocity in perifocal frame for circular orbit
v_pqw = v_mag * [-sin(nu); cos(nu); 0];

% Step 6: Transform velocity to ECI
v_eci = R_pqw_to_eci * v_pqw;

% Format output as row vectors
r_eci = r_eci.';
v_eci = v_eci.';

end