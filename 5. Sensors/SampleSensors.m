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
    z_GPS = GPS(x_true(1:3), noise_GPS, drift_GPS, driftRate_GPS);
else
    z_GPS= [0 0 0].';
end
%---

% GYR
j = mod(t,dt_GYR);
if j == 0
    z_GYR = Gyro(x_true(11:13), noise_GYR, drift_GYR, driftRate_GYR);
else
    z_GYR = [0 0 0].';
end
%---

% ST
j = mod(t,dt_ST);
if j == 0
    z_ST = StarTracker(x_true(7:10), noise_ST);
else
    z_ST = [0 0 0 0].';
end
%---


% CSS
j = mod(t,dt_CSS);
if j == 0
    z_CSS = CoarseSunSensor(x_true(7:10), noise_CSS);
else
    z_CSS = [0 0 0].';
end
%---

% MAG
j = mod(t,dt_MAG);
if j == 0
    z_MAG = Magnetometer(x_true(1:3),x_true(7:10),noise_MAG,angle_MAG,t,we_p);
else
    z_MAG = [0 0 0].';
end
%---

z_TRIAD = TRIAD(z_CSS,z_MAG,angle_MAG,x_true(1:3),t,we_p);

end

