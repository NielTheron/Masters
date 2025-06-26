%==========================================================================
% Niel Theron
% 19-06-2025
%==========================================================================
% The purpose of this function is to get the ECI position en velocity
% vectors given the LLA vector
%==========================================================================
% Inout:
%   v_L : LLA vector [3x1] [deg,deg,km]
% 
% Output:
%   r_I : Position vector
%   v_I : Velocity vector
%==========================================================================

function [r_I, v_I] = LLA2RV(v_L,Mu,we,t)

    % Convert lat/lon to radians
    lon = deg2rad(v_L(2,1));
    %---

    % Get inertial positoin vector
    r_I = ECR2ECI(LLA2ECR(v_L),we,t);
    %----

    % Local East Unit Vector in ECEF
    east_ecef = [-sin(lon);
                  cos(lon);
                  0];
    east_unit = east_ecef / norm(east_ecef);
    %---

    % Orbital speed for circular orbit
    r_mag = norm(r_I);
    v_mag = sqrt(Mu / r_mag);
    %---

    % Velocity vector: due east - ENSURE COLUMN VECTOR
    v_I = v_mag * east_unit;  % Already a column vector, no transpose needed
    %---
end

