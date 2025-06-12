function PlotMeasurement(y_true,n_f,dt)

figure('Name','Measured Output')
hold on
n = (0:size(y_true,3)-1)*dt;
for i = 1:n_f
    plot(n, squeeze(y_true(1,i,:)));
    plot(n, squeeze(y_true(2,i,:)));
    plot(n, squeeze(y_true(3,i,:)));
end
title("Measurements")
xlabel("Time(s)")
ylabel("Distance (km) in Body Frame")
grid on
hold off

end