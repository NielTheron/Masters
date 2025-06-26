function Q = TuneQ()
sigma_pos = 2;  % km
sigma_vel = 3; % km/s
sigma_q = 1e-4;
sigma_w = 1e-4;  % rad/s

Q = diag([
    sigma_pos^2 * ones(1,3), ...
    sigma_vel^2 * ones(1,3), ...
    sigma_q^2 * ones(1,4), ...
    sigma_w^2 * ones(1,3)
    ]);

end