function P = TuneP()
sigma_pos = 1e3;  % m
sigma_vel = 1e3; % m/s
sigma_q = 1e2;
sigma_w = 1e2;  % rad/s

P = diag([
    sigma_pos^2 * ones(1,3), ...
    sigma_vel^2 * ones(1,3), ...
    sigma_q^2 * ones(1,4), ...
    sigma_w^2 * ones(1,3)
    ]);

end