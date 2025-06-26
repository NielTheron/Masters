%==========================================================================
% Niel Theron
% 19-06-2025
%==========================================================================
% The purpose of this function is to transform from the orbital reference
% frame to the body reference frame
%==========================================================================
% Input:
%   v_O : Vector in the orbital refrence frame [km]
%   q_O2B   : The quaternion that repreasent the rotatio from orbital to
%   body [4x1]
%==========================================================================


function [v_B, q_O2B] = ORB2BOD(v_O,rot_YPR)

    q_O2B = eul2quat(deg2rad(rot_YPR),"YZX");
    R_O2B = quat2rotm(q_O2B);
    v_B = R_O2B*v_O;

end