function q_bod_to_eci = UpdateBodyToECI(q_bod_to_orb, r_eci, v_eci)
%==========================================================================
% UpdateBodyToECI  
%==========================================================================
% Purpose: Convert body-to-orbital quaternion to body-to-ECI quaternion
%
% Inputs:
%   q_bod_to_orb - Body-to-orbital quaternion [1x4]
%   r_eci        - Current position in ECI (km) [3x1]
%   v_eci        - Current velocity in ECI (km/s) [3x1]
%
% Outputs:
%   q_bod_to_eci - Body-to-ECI quaternion [1x4]
%
% Note: Use this to convert camera pointing commands to ECI frame
%       for your EKF state vector
%==========================================================================

% Calculate current orbital frame orientation
z_orbital_eci = -r_eci / norm(r_eci);                   % Nadir
r_unit = r_eci / norm(r_eci);
v_perp = v_eci - dot(v_eci, r_unit) * r_unit;
x_orbital_eci = v_perp / norm(v_perp);                  % Along-track
y_orbital_eci = cross(z_orbital_eci, x_orbital_eci);    % Cross-track
%---

% Orbital-to-ECI rotation matrix
R_orb_to_eci = [x_orbital_eci, y_orbital_eci, z_orbital_eci];
q_orb_to_eci = rotm2quat(R_orb_to_eci);
%---

% Combine: q_bod_to_eci = q_orb_to_eci * q_bod_to_orb
q_bod_to_eci = quatmultiply(q_orb_to_eci, q_bod_to_orb);

end