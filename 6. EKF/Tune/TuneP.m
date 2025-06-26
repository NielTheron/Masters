function P = TuneP()
    sigma_pos = 5;     % km
    sigma_vel = 10;    % km/s  
    sigma_q = 0.8;     % Reduced quaternion uncertainty
    sigma_w = 0.2;     % Reduced angular velocity uncertainty
    P = diag([
        sigma_pos^2 * ones(1,3), ...
        sigma_vel^2 * ones(1,3), ...
        sigma_q^2 * ones(1,4), ...
        sigma_w^2 * ones(1,3)
    ]);
end