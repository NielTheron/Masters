function PlotEstimatedMeasurement(y_est,n_f,dt)
figure('Name','Estimated Measured')
hold on
n = (0:size(y_est,3)-1)*dt;
for i = 1:n_f
    plot(n,squeeze(y_est(1,i,:)));
    plot(n,squeeze(y_est(2,i,:)));
    plot(n,squeeze(y_est(3,i,:)));
end
title("Estimated measurements")
xlabel("Time(s)")
ylabel("Distance (km) in Body Frame")
grid on
hold off

end