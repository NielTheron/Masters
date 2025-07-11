%==========================================================================
% Niel Theron
% 19-06-2025
%==========================================================================
% The purpose of this function is to convert from the inertial frame to the
% body orbital frame
%=========================================================================
% Input:
%   v_I : Inertial vector [3x1] [km]
%   r_I : Position vector [3x1] [km]
%   ve_I : Velocity vector [3x1] [km] 
% Output:
%   v_O : Orbital feature vector
%=========================================================================

function [v_O,q_I2O] = ECI2ORB(v_I,r_I,ve_I)

    % Define orbital (LVLH) frame vectors in ECI coordinates
    z_O = -r_I / norm(r_I);                 % Nadir (toward Earth)
    r_unit = r_I / norm(r_I);
    ve_perp = ve_I - dot(ve_I, r_unit) * r_unit;
    x_O = ve_perp / norm(ve_perp);          % Along-track (velocity direction)
    y_O = cross(z_O, x_O);                  % Cross-track
    y_O = y_O / norm(y_O);
    %---

    % Translation
    v_ORI = (v_I - r_I);

    % Create Rotation Matrix
    R_I2O = [x_O, y_O, z_O];
    q_I2O = rotm2quat(R_I2O);
    %---

    % Create Vector
    v_O = R_I2O*v_ORI;
    %

end

