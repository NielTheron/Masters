function PlotGPS(x_true,dt,GPS_measurement)
figure('Name','GPS Meaasurement')
hold on
n = (0:size(x_true,2)-1)*dt;
plot(n,x_true(1:3,:))
plot(n,GPS_measurement)
title("Abstract Position Measurement Sensors");
xlabel("Time (s)");
ylabel("Position (km)");
grid on
end

