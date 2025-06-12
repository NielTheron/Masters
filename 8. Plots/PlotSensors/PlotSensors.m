function PlotSensors(x_true,dt,ST_measurement,MAG_measurement,CSS_measurement)
figure('Name',"Sensor Comparison")
hold on
n = (0:size(x_true,2)-1)*dt;
plot(n,x_true(7:10,:))
plot(n,ST_measurement)
plot(n,MAG_measurement)
plot(n,CSS_measurement)
title("Abstract Attitude Sensor Measurements")
% legend("AutoUpdate","on");
xlabel("Time (s)");
ylabel("Magnitude");
grid on
end

