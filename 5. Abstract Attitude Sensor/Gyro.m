function [GYR_measurement, GYR_drift_new] = Gyro(position, GYR_noise, GYR_drift_prev, drift_rate)

    GYR_drift_new = GYR_drift_prev + drift_rate * randn(3,1);
    sigma_gyr = GYR_noise * randn(3,1);
    GYR_measurement = position + sigma_gyr + GYR_drift_new;

end