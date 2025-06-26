function [z_GPS, z_GYR, z_ST, z_CSS, z_MAG, z_TRIAD] = ...
    SampleSensors(x_true, t, we_p, ...
    dt_GPS, noise_GPS, drift_GPS, driftRate_GPS, ...
    dt_GYR, noise_GYR, drift_GYR, driftRate_GYR, ...
    dt_ST, noise_ST, ...
    dt_CSS, noise_CSS, ...
    dt_MAG, noise_MAG,angle_MAG)

% GPS
j = mod(t,dt_GPS);
if j == 0
    z_GPS(:,r) = GPS(x_true(1:3,r), noise_GPS, drift_GPS, driftRate_GPS);
else
    z_GPS(:,r) = [0 0 0].';
end
%---

% GYR
j = mod(t,dt_GYR);
if j == 0
    z_GYR(:,r) = Gyro(x_true(11:13,r), noise_GYR, drift_GYR, driftRate_GYR);
else
    z_GYR(:,r) = [0 0 0].';
end
%---

% ST
j = mod(t,dt_ST);
if j == 0
    z_ST(:,r) = StarTracker(x_true(7:10,r), noise_ST);
else
    z_ST(:,r) = [0 0 0 0].';
end
%---


% CSS
j = mod(t,dt_CSS);
if j == 0
    z_CSS(:,r) = CoarseSunSensor(x_true(7:10,r), noise_CSS);
else
    z_CSS(:,r) = [0 0 0].';
end
%---

% MAG
j = mod(t,dt_MAG);
if j == 0
    z_MAG(:,r) = Magnetometer(x_true(1:3,r),x_true(7:10,r),noise_MAG,angle_MAG,t,we_p);
else
    z_MAG(:,r) = [0 0 0].';
end
%---


z_TRIAD(:,r) = TRIAD(z_CSS(:,r),z_MAG(:,r),x_true(1:3,r),t,we_p);

end

