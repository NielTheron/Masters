function h = HFunction(xm, c_eci)
%==========================================================================
% Niel Theron - Complete Transformation Chain
%==========================================================================
% Measurement model: ECI → LVLH (Orbital) → Body transformation
% 
% Inputs:
% xm    - State vector [r_ECI(3), v_ECI(3), q_body_to_inertial(4), w_body(3)]
% c_eci - Feature position in ECI frame [3x1] (km)
%
% Output:
% h     - Expected measurement in body/camera frame [3x1] (km)
%
% Transformation Chain: ECI → LVLH → BOD
%==========================================================================

%==========================================================================
% Step 1: Transform from ECI to LVLH (Orbital Frame)
%==========================================================================

% Extract satellite position and velocity in ECI
r_sat_eci = [xm(1) xm(2) xm(3)].';
v_sat_eci = [xm(4) xm(5) xm(6)].';

% Calculate LVLH frame vectors
z_lvlh = -r_sat_eci / norm(r_sat_eci);           % Nadir pointing (toward Earth center)
y_lvlh = cross(z_lvlh, v_sat_eci);               % Cross-track (opposite to angular momentum)
y_lvlh = y_lvlh / norm(y_lvlh);
x_lvlh = cross(y_lvlh, z_lvlh);                  % Along-track (velocity direction)
x_lvlh = x_lvlh / norm(x_lvlh);

% Create rotation matrix from ECI to LVLH
R_eci_to_lvlh = [x_lvlh.';
                 y_lvlh.';
                 z_lvlh.'];

% Transform feature from ECI to LVLH frame
f_eci_relative = c_eci - r_sat_eci;              % Feature relative to satellite in ECI
f_lvlh = R_eci_to_lvlh * f_eci_relative;         % Feature in LVLH frame

%==========================================================================
% Step 2: Transform from LVLH to Body Frame
%==========================================================================

% Extract quaternion (Body to Inertial)
q = quatnormalize(xm(7:10).');
qs = q(1); qx = q(2); qy = q(3); qz = q(4);

% Create rotation matrix from Body to Inertial (ECI)
R_body_to_eci = [1 - 2*(qy^2 + qz^2),  2*(qx*qy - qs*qz),  2*(qx*qz + qs*qy);
                 2*(qx*qy + qs*qz),  1 - 2*(qx^2 + qz^2),  2*(qy*qz - qs*qx);
                 2*(qx*qz - qs*qy),  2*(qy*qz + qs*qx),  1 - 2*(qx^2 + qy^2)];

% Calculate rotation matrix from Body to LVLH
% R_body_to_lvlh = R_eci_to_lvlh * R_body_to_eci
R_body_to_lvlh = R_eci_to_lvlh * R_body_to_eci;

% Invert to get LVLH to Body transform
R_lvlh_to_body = R_body_to_lvlh';

% Transform from LVLH to Body frame
h = R_lvlh_to_body * f_lvlh;

%==========================================================================
% Output is in Body/Camera Frame
%==========================================================================
% X-axis: Along-track
% Y-axis: Cross-track  
% Z-axis: Camera lens direction (nadir pointing)
% Units: km (same as input ECI coordinates)

end