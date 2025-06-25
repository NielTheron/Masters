function PlotGyro(x_true,dt,GYR_measurement)
figure('Name',"GyroScope")
hold on
n = (0:size(x_true,2)-1)*dt;
plot(n,x_true(11:13,:))
plot(n,GYR_measurement)
% legend("AutoUpdate","on");
title("Abstract Gyroscope Measurement")
xlabel("Time (s)");
ylabel("Angular Velocity (rad/s)")
grid on
end


