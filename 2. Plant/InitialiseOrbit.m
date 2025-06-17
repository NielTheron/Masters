function [r_eci, v_eci] = InitialiseOrbit(lat_deg, lon_deg, alt_km)
%==========================================================================
% Generates a circular orbit at a given latitude, longitude, and altitude,
% where the velocity is purely eastward (tangent to local parallel).
%==========================================================================
% INPUT:
% lat_deg    : Latitude [deg]
% lon_deg    : Longitude [deg]
% alt_km     : Altitude above sea level [km]
% OUTPUT:
% r_eci      : Position vector in ECI [km] - COLUMN VECTOR [3x1]
% v_eci      : Velocity vector in ECI [km/s] - COLUMN VECTOR [3x1]
%==========================================================================

% Constants
mu_earth = 398600.4418;     % [km^3/s^2]
%---

% Convert lat/lon to radians
lon = deg2rad(lon_deg);
%---

% Compute ECEF position (assuming spherical Earth)
[x,y,z] = geodetic2ecef(wgs84Ellipsoid('km'),lat_deg,lon_deg,alt_km);
%---

r_ecef=[x;y;z];

% Convert to ECI (ECI = ECEF at time = 0) - ENSURE COLUMN VECTOR
r_eci = r_ecef(:);  % Force column vector
%---

% Local East Unit Vector in ECEF (and ECI at time 0)
east_ecef = [-sin(lon);
              cos(lon);
              0];
east_unit = east_ecef / norm(east_ecef);
%---

% Orbital speed for circular orbit
r_mag = norm(r_eci);
v_mag = sqrt(mu_earth / r_mag);
%---

% Velocity vector: due east - ENSURE COLUMN VECTOR
v_eci = v_mag * east_unit;  % Already a column vector, no transpose needed
%---

end