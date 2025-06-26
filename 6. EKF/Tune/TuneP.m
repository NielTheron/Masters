function P = TuneP()
    sigma_pos = 1000;     % Reduced from 500 to 10 meters
    sigma_vel = 1;      % Reduced from 10 to 1 m/s  
    sigma_q = 0.01;    % Reduced quaternion uncertainty
    sigma_w = 0.01;    % Reduced angular velocity uncertainty
    P = diag([
        sigma_pos^2 * ones(1,3), ...
        sigma_vel^2 * ones(1,3), ...
        sigma_q^2 * ones(1,4), ...
        sigma_w^2 * ones(1,3)
    ]);
end