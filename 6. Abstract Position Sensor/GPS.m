%==========================================================================
% Niel Theron
% 21-05-2025
%==========================================================================
% The purpose of this function is to abstract the GPS, the GPS also has
% drift
%==========================================================================

function [GPS_measurement, GPS_drift_new] = GPS(position, GPS_noise, GPS_drift_prev, drift_rate)

    GPS_drift_new = GPS_drift_prev + drift_rate * randn(3,1);
    sigma_gps = GPS_noise * randn(3,1);
    GPS_measurement = position + sigma_gps + GPS_drift_new;

end
