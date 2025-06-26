%==========================================================================
% Niel Theron
% 21-05-2025
%==========================================================================
% The Purpose of this function is to abstract a coarse sun sensor
%=========================================================================
function Sun_B_est = CoarseSunSensor(attitude, CSS_noise_deg)

%=========================
% Setup: True sun vector
%=========================
Sun_I = [150e6, 0, 0].';               % Sun vector in inertial
Sun_I = Sun_I / norm(Sun_I);        % Normalize

%=========================
% Rotate to body frame
%=========================
R_B2I = quat2rotm(attitude.');      % Rotation from body to inertial
R_I2B = R_B2I';                     % Inverse rotation
Sun_B_true = R_I2B * Sun_I;         % True sun vector in body frame

%=========================
% CSS Sensor directions
%=========================
css_dirs = [ 1  0  0;   % +X
            -1  0  0;   % -X
             0  1  0;   % +Y
             0 -1  0;   % -Y
             0  0  1;   % +Z
             0  0 -1];  % -Z

%=========================
% Simulate CSS readings (cosine response + max(0))
%=========================
css_readings = max(0, css_dirs * Sun_B_true);  % 6x1 vector of readings

%=========================
% Add noise to each CSS reading
%=========================
sigma = deg2rad(CSS_noise_deg);     % Convert noise to radians
css_readings = css_readings + sigma * randn(size(css_readings));
css_readings = max(css_readings, 0); % Clamp negatives

%=========================
% Reconstruct sun vector estimate from readings
%=========================
% Weighted sum of sensor directions
Sun_B_est = css_dirs' * css_readings;    % 3x1
Sun_B_est = Sun_B_est / norm(Sun_B_est); % Normalize to unit vector

end
