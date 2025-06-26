function PlotMeasurementError(y_true,y_est,n_f,dt)

figure('Name','Output Error')
hold on
n = (0:size(y_true,3)-1)*dt;
for i = 1:n_f
    plot(n,squeeze(y_true(1,i,:))-squeeze(y_est(1,i,:)));
    plot(n,squeeze(y_true(2,i,:))-squeeze(y_est(2,i,:)));
    plot(n,squeeze(y_true(3,i,:))-squeeze(y_est(3,i,:)));
end
title("Measurement Error")
xlabel("Time (s)")
ylabel("Distance (km)")
grid on
hold off
end
