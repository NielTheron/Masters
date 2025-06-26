
function [q_est, R_est] = TRIAD(sun_body, mag_body, r_ECI, t, we)
%==========================================================================
% TRIAD - Two-Vector Attitude Determination Algorithm
%==========================================================================
% Purpose: Estimates spacecraft attitude using sun vector and magnetometer
%          measurements via the TRIAD (Tri-Axial Attitude Determination) method
%
% Author: D.P. Theron
% Date: December 2024
%
% Inputs:
%   sun_body - Sun vector in body frame [3x1] (from CSS)
%   mag_body - Magnetic field vector in body frame [3x1] (from magnetometer)
%   r_ECI    - Satellite position in ECI frame [3x1] (km)
%   t        - Time since epoch (seconds)
%   we       - Earth rotation rate (rad/s)
%
% Outputs:
%   q_est    - Estimated attitude quaternion [4x1] [qs qx qy qz]
%   R_est    - Estimated rotation matrix from inertial to body frame [3x3]
%   validity - Quality indicator [0-1], 1 = excellent, 0 = poor
%
% Algorithm: TRIAD uses two non-collinear reference vectors to construct
%            orthogonal triads in both reference and body frames, then
%            determines the rotation matrix between them.
%==========================================================================

%% Input Validation and Preprocessing
% Ensure inputs are column vectors
sun_body = sun_body(:) / norm(sun_body(:));  % Normalize sun vector
mag_body = mag_body(:) / norm(mag_body(:));  % Normalize mag vector
r_ECI = r_ECI(:);


%% Step 1: Construct Reference Vectors in Inertial Frame
%==========================================================================

% 1.1: Sun vector in inertial frame (simplified model)
% Assuming sun is along +X direction in ECI frame
sun_inertial = [1; 0; 0];  % Simplified sun model

% 1.2: Magnetic field vector in inertial frame
mag_inertial = ComputeMagneticFieldECI(r_ECI, t, we);

% Normalize reference vectors
sun_inertial = sun_inertial / norm(sun_inertial);
mag_inertial = mag_inertial / norm(mag_inertial);

%% Step 3: Construct Orthogonal Triads
%==========================================================================

% 3.1: Reference triad in inertial frame
v1_ref = sun_inertial;                                   % First vector (sun)
v2_ref = cross(sun_inertial, mag_inertial);              % Cross product
v2_ref = v2_ref / norm(v2_ref);                          % Normalize
v3_ref = cross(v1_ref, v2_ref);                          % Complete right-handed triad

% 3.2: Body triad in body frame
v1_body = sun_body;                                      % First vector (sun)
v2_body = cross(sun_body, mag_body);                     % Cross product
v2_body = v2_body / norm(v2_body);                       % Normalize
v3_body = cross(v1_body, v2_body);                       % Complete right-handed triad

%% Step 4: Construct Rotation Matrix
%==========================================================================
% Build triad matrices
T_ref = [v1_ref, v2_ref, v3_ref];      % Reference triad matrix [3x3]
T_body = [v1_body, v2_body, v3_body];  % Body triad matrix [3x3]

% Rotation matrix from inertial to body frame
% R_est transforms vectors from inertial frame to body frame
R_est = T_body * T_ref.';


%% Step 5: Convert to Quaternion
%==========================================================================
% Get initial quaternion
q_est = rotm2quat(R_est).';  % [qs qx qy qz]

% CORRECTION MATRIX: Apply the fixes you identified
% Based on your observation: q1 flipped, q2 and q3 swapped
q_corrected = zeros(4,1);
q_corrected(1) = q_est(1);   % q0 (qs) is correct
q_corrected(2) = -q_est(2);  % q1 (qx) needs sign flip
q_corrected(3) = q_est(4);   % q2 (qy) gets q3's value
q_corrected(4) = q_est(3);   % q3 (qz) gets q2's value

% Ensure unit quaternion
q_est = q_corrected / norm(q_corrected);

% Recompute rotation matrix from corrected quaternion
R_est = quat2rotm(q_est.');

end