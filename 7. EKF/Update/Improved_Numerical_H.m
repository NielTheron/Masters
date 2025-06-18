function H1 = Improved_Numerical_H(xm, c_eci)
%==========================================================================
% Improved_Numerical_H - Numerical Jacobian for EKF Measurement Model
%==========================================================================
% Purpose: Computes numerical Jacobian matrix for the HFunction measurement
%          model using central difference method with adaptive step sizes
%
% Author: D.P. Theron  
% Date: June 18, 2025
%
% Inputs:
%   xm    - State vector [13x1]:
%           [1:3]   - Satellite position in ECI frame (km)
%           [4:6]   - Satellite velocity in ECI frame (km/s)
%           [7:10]  - Attitude quaternion body-to-inertial [qs, qx, qy, qz]
%           [11:13] - Angular velocity body frame (rad/s)
%   c_eci - Feature position in ECI frame [3x1] (km)
%
% Output:
%   H1    - Jacobian matrix [3x13] relating measurement to state changes
%
% Method: Central difference with adaptive step sizes optimized for
%         position, velocity, quaternion, and angular velocity components
%==========================================================================

%==========================================================================
% Initialize Jacobian Computation
%==========================================================================
% Get dimensions and baseline measurement
n = length(xm);                    % Number of state variables (13)
h0 = compute_h_simplified(xm, c_eci);  % Baseline measurement [3x1]
m = length(h0);                    % Number of measurements (3)
H1 = zeros(m, n);                  % Initialize Jacobian matrix [3x13]

%==========================================================================
% Compute Partial Derivatives Using Central Difference
%==========================================================================
for i = 1:n
    %======================================================================
    % Adaptive Step Size Selection
    %======================================================================
    if i >= 1 && i <= 3
        % Position components (km) - larger step size
        epsilon = 1e-6 * max(1, abs(xm(i)));
        
    elseif i >= 4 && i <= 6
        % Velocity components (km/s) - moderate step size
        epsilon = 1e-8 * max(1, abs(xm(i)));
        
    elseif i >= 7 && i <= 10
        % Quaternion components (unitless) - small step size for stability
        epsilon = 1e-7;
        
    elseif i >= 11 && i <= 13
        % Angular velocity components (rad/s) - moderate step size
        epsilon = 1e-8 * max(1, abs(xm(i)));
        
    else
        % Default case (should not occur with 13-state vector)
        epsilon = 1e-8 * max(1, abs(xm(i)));
    end
    
    %======================================================================
    % Create Perturbed States
    %======================================================================
    % Initialize perturbation vector
    dx = zeros(n, 1);
    dx(i) = epsilon;
    
    % Create forward and backward perturbed states
    x_plus = xm + dx;
    x_minus = xm - dx;
    
    %======================================================================
    % Handle Quaternion Normalization for Perturbed States
    %======================================================================
    % Ensure quaternion components remain normalized after perturbation
    if i >= 7 && i <= 10
        % Normalize quaternions for both perturbed states
        q_plus_norm = quatnormalize(x_plus(7:10).');
        x_plus(7:10) = q_plus_norm(:);  % Convert back to column vector
        
        q_minus_norm = quatnormalize(x_minus(7:10).');  
        x_minus(7:10) = q_minus_norm(:);  % Convert back to column vector
    end
    
    %======================================================================
    % Compute Perturbed Measurements
    %======================================================================
    % Evaluate measurement function at perturbed states
    h_plus = compute_h_simplified(x_plus, c_eci);
    h_minus = compute_h_simplified(x_minus, c_eci);
    
    %======================================================================
    % Central Difference Approximation
    %======================================================================
    % Compute partial derivative: ∂h/∂x_i ≈ (h(x+ε) - h(x-ε))/(2ε)
    H1(:, i) = (h_plus - h_minus) / (2 * epsilon);
end

%==========================================================================
% Apply Precision Rounding for Numerical Stability
%==========================================================================
% Round to 4 significant figures to match analytical precision
H1 = round(H1, 4, "significant");

end

%==========================================================================
% Supporting Function: Simplified Measurement Model
%==========================================================================
function h = compute_h_simplified(xm, c_eci)
% Simplified measurement computation matching HFunction.m logic
% Direct ECI → Body frame transformation

%==========================================================================
% Step 1: Extract Satellite Position
%==========================================================================
r_sat_eci = xm(1:3);  % Satellite position in ECI frame (km)
r_sat_eci = r_sat_eci(:);  % Ensure column vector

%==========================================================================
% Step 2: Compute Relative Feature Vector in ECI Frame
%==========================================================================
f_eci_relative = c_eci - r_sat_eci;  % Feature relative to satellite (km)

%==========================================================================
% Step 3: Extract and Normalize Quaternion
%==========================================================================
q = xm(7:10);  % Extract quaternion [qs, qx, qy, qz]
q = q(:);      % Ensure column vector

% Normalize quaternion for numerical stability
q = quatnormalize(q.');  % quatnormalize expects row vector
q = q(:);               % Convert back to column vector

% Extract quaternion components
qs = q(1);  % Scalar component
qx = q(2);  % X vector component
qy = q(3);  % Y vector component  
qz = q(4);  % Z vector component

% Step 4: Construct Rotation Matrix ECI → Body Frame
%==========================================================================
% Quaternion-to-DCM conversion for ECI to Body transformation
% Note: Input quaternion q represents body-to-inertial rotation
% This matrix is the transpose (inverse) of the quaternion's natural DCM
R_eci_to_body = [
    1 - 2*(qy^2 + qz^2),  2*(qx*qy - qs*qz),  2*(qx*qz + qs*qy);
    2*(qx*qy + qs*qz),   1 - 2*(qx^2 + qz^2),  2*(qy*qz - qs*qx);
    2*(qx*qz - qs*qy),   2*(qy*qz + qs*qx),   1 - 2*(qx^2 + qy^2)
];

%==========================================================================
% Step 5: Transform to Body Frame
%==========================================================================
% Apply rotation to get feature vector in body frame coordinates
h = R_eci_to_body * f_eci_relative;

end

%==========================================================================
% Notes:
% - Jacobian dimensions: [3x13] for 3 measurements, 13 state variables
% - Step sizes optimized for different state variable types
% - Quaternion normalization preserves unit constraint during perturbation
% - Central difference method provides better accuracy than forward difference
% - Rounding prevents numerical artifacts from propagating through EKF
% - Function matches HFunction.m logic exactly for consistency
%==========================================================================