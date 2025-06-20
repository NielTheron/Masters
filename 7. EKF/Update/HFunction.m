function h = HFunction(xm, c_eci)
%==========================================================================
% HFunction - EKF Measurement Model for EarthTracker
%==========================================================================
% Purpose: Predicts the expected EarthTracker measurement given the current
%          state estimate and known feature position in ECI coordinates
%
% Author: D.P. Theron
% Date: June 18, 2025
%
% Inputs:
%   xm    - State vector [13x1]:
%           [1:3]   - Satellite position in ECI frame (km)
%           [4:6]   - Satellite velocity in ECI frame (km/s)  
%           [7:10]  - Attitude quaternion inertial-to-body q_B/I [qs, qx, qy, qz]
%           [11:13] - Angular velocity body frame (rad/s)
%   c_eci - Feature position in ECI frame [3x1] (km)
%
% Output:
%   h     - Predicted measurement: feature vector in body frame [3x1] (km)
%
% Transformation: ECI → Body Frame
% Steps: 1) Translate feature relative to satellite
%        2) Rotate from inertial frame to body frame using quaternion
%==========================================================================


% Extract Satellite Position from State Vector
r_sat_eci = xm(1:3);  % Position vector [x; y; z] in ECI frame (km)
%---

% Translate Feature Position to Relative Coordinates
f_eci_relative = c_eci - r_sat_eci;  % Relative position vector (km)
%---


% Extract and Normalize Attitude Quaternion
% Extract quaternion representing inertial-to-body rotation
% Convention: q_B/I = [qs, qx, qy, qz] where qs is scalar part
q = xm(7:10);
q = q(:);  % Ensure column vector

% Normalize quaternion to maintain unit constraint
% This is critical for numerical stability
q = quatnormalize(q.');  % quatnormalize expects row vector
q = q(:);  % Convert back to column vector

% Extract individual quaternion components for clarity
qs = q(1);  % Scalar component
qx = q(2);  % X vector component  
qy = q(3);  % Y vector component
qz = q(4);  % Z vector component
%---

% Construct Rotation Matrix from ECI to Body Frame
R_I2B = [
    1 - 2*(qy^2 + qz^2),  2*(qx*qy - qs*qz),  2*(qx*qz + qs*qy);
    2*(qx*qy + qs*qz),   1 - 2*(qx^2 + qz^2),  2*(qy*qz - qs*qx);
    2*(qx*qz - qs*qy),   2*(qy*qz + qs*qx),   1 - 2*(qx^2 + qy^2)
];
%---

%Transform Feature Vector to Body Frame
h = R_I2B.' * f_eci_relative;
%---

end

