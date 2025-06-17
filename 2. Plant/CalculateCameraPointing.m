function q_bod_to_orb = CalculateCameraPointing(yaw_offset_deg, pitch_offset_deg, roll_offset_deg)
%==========================================================================
% CalculateCameraPointing
%==========================================================================
% Purpose: Calculate body-to-orbital quaternion for camera pointing control
%
% Inputs:
%   yaw_offset_deg   - Yaw rotation from orbital alignment (deg)
%   pitch_offset_deg - Pitch rotation from orbital alignment (deg) 
%   roll_offset_deg  - Roll rotation from orbital alignment (deg)
%
% Outputs:
%   q_bod_to_orb     - Body-to-orbital quaternion [1x4] [qw, qx, qy, qz]
%
% Note: When all offsets = 0, body frame = orbital frame (identity quaternion)
%       Use this for camera pointing commands relative to orbital frame
%==========================================================================

% Convert Euler angles to quaternion (YZX sequence to match your existing code)
q_bod_to_orb = eul2quat(deg2rad([roll_offset_deg, yaw_offset_deg, pitch_offset_deg]), "YZX");

end
