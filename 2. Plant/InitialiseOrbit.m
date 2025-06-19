function [r_I, v_I, q_I2B, w_I2B] ...
 = InitialiseOrbit( ...
 lat, lon, alt, ...
 rol, yaw, pit,...
 wx, wy, wz, ...
 Mu, we, t)
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
[r_I, v_I] = LLA2RV([lat; lon; alt],Mu,we,t);
%---

% Calculate the quaternion from Inertial to Body
[~,q_I2O] = ECI2ORB([0;0;0],r_I,v_I);
[~,q_O2B] = ORB2BOD([0;0;0],[rol, yaw, pit]);
q_I2B = quatmultiply(q_O2B,q_I2O).';
%---

% Convert input body rates from B/O to B/I
w_O2B = deg2rad([wx; wy; wz]); % Body relative to orbital
w_I2B = w_O2B;
%---

end

%==========================================================================