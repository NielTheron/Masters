%==========================================================================
% Niel Theron
% 05-06-2025
%==========================================================================
% The purpose of this function is to take a lat long coordinate and make a
% proper circular orbit initial condition based of the height.
%==========================================================================
% INPUT:
% lat          :Lattitude (deg)
% lon          :Longitude (deg)
% alt          :Altitude (km)
%
% OUPUT:
% r            :Position vector (km)
% v            :Velocity vector (km/s)
%
% VARAIBELS:
% ref_dir      :Reference direction [x,y,z] (ECI)
% r_mag        :Magnitude of position vector
% v_mag        :Magnitude of velocity vector
% v_dir        :Direction of the velocity vector
%
% CONSTANTS:
% Mu_earth     :Gravitational parameter (km3/s2)
%==========================================================================

function [r, v] = InitialiseOrbit(lat, lon, alt)

% Refrence Direction
ref_dir = [0 0 1];
%---

% Work out initial conditions
r = lla2ecef([lat,lon,alt*1000],"WGS84")/1000;
Mu_earth = 398600.4418;
r_mag = norm(r);
v_mag = sqrt(Mu_earth / r_mag);
%---

% Cross product gives orthogonal direction
v_dir = cross(ref_dir,r);
v_dir = v_dir / norm(v_dir);  % normalize
%---

% Scale to orbital speed
v = v_mag * v_dir;
%---

end