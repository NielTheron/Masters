%==========================================================================
% Niel Theorn
% 19-06-2025
%==========================================================================
% This function is used to convert from the ECR reference frame to the ECI
% reference frame
%=========================================================================
% Input:
%   v_E : ECR Vector [km] [3x1]
%   t   : Time [s]
%   we  : Angular veloctiy if the Earth [rad/s]
% Output:
%   v_I : ECI Vector [km] [3x1]
% Variables:
%   theta : Rotation angle [rad]
%==========================================================================
function [v_I,q_E2I] = ECR2ECI(v_E,t,we)

    % Make Rotation matrix
    theta = -we*t;
    R_E2I =  [cos(theta) -sin(theta) 0;
             sin(theta) cos(theta) 0;
             0           0          1];
    q_E2I = rotm2quat(R_E2I);
    %---

    % Multiply
    v_I = R_E2I*v_E;
    %---

end

