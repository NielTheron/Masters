function [r_p, v_p, q_eci_to_bod, w_B_I] = InitialiseOrbitAligned(lat_p, lon_p, alt_p, rollRate_BO_p, pitchRate_BO_p, yawRate_BO_p)
%==========================================================================
% InitialiseOrbitAligned
%==========================================================================
% Purpose: Initialize satellite orbit with body frame aligned to orbital frame
%
% Inputs:
%   lat_p, lon_p, alt_p - Initial orbital position
%   rollRate_BO_p, pitchRate_BO_p, yawRate_BO_p - Body rates relative to orbital frame (deg/s)
%
% Outputs:
%   r_p            - Position vector (km) [3x1]
%   v_p            - Velocity vector (km/s) [3x1] 
%   q_bod_to_eci   - Body-to-ECI quaternion [1x4] [qw, qx, qy, qz]
%   w_B_I          - Body angular velocity relative to inertial frame (rad/s) [3x1]
%
% Note: Input rates are B/O (body relative to orbital), output is B/I (body relative to inertial)
%==========================================================================

% Calculate orbital position and velocity (eastward velocity)
[r_int, v_int] = InitialiseOrbit(lat_p, lon_p, alt_p);

% Ensure vectors are column vectors
r_int = r_int(:);
v_int = v_int(:);
%---

% Define orbital (LVLH) frame vectors in ECI coordinates
z_orbital_eci = -r_int / norm(r_int);               % Nadir (toward Earth)
r_unit = r_int / norm(r_int);
v_perp = v_int - dot(v_int, r_unit) * r_unit;
x_orbital_eci = v_perp / norm(v_perp);              % Along-track (velocity direction)
y_orbital_eci = cross(z_orbital_eci, x_orbital_eci);% Cross-track
%---

% Body frame = Orbital frame (perfect alignment)
R_eci_to_bod = [x_orbital_eci, y_orbital_eci, z_orbital_eci];
q_eci_to_bod = rotm2quat(R_eci_to_bod);
%---

% Calculate orbital angular velocity (O/I expressed in orbital frame)
% For circular orbit: ω_O/I = [0; 0; -n] where n is mean motion
n_orbit = sqrt(398600 / norm(r_int)^3); % Mean motion (rad/s)
w_O_I_in_O = [0; 0; -n_orbit]; % Orbital frame rotates about -Z axis

% Convert input body rates from B/O to B/I
% ω_B/I = ω_B/O + ω_O/I (both expressed in body frame)
w_B_O = deg2rad([rollRate_BO_p; pitchRate_BO_p; yawRate_BO_p]); % Body relative to orbital
w_O_I_in_B = w_O_I_in_O; % Since body = orbital frame initially

% Total angular velocity: Body relative to inertial
w_B_I = w_B_O + w_O_I_in_B;

% Set outputs (ensure column vectors)
r_p = r_int(:);
v_p = v_int(:);

end

%==========================================================================