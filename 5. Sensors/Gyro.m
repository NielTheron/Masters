function [GYR_measurement, GYR_drift_new] = Gyro(angV, GYR_noise, GYR_drift_prev, drift_rate)

    GYR_drift_new = GYR_drift_prev + deg2rad(drift_rate) * randn(3,1);
    sigma_gyr = deg2rad(GYR_noise) * randn(3,1);
    GYR_measurement = angV + sigma_gyr + GYR_drift_new;

end