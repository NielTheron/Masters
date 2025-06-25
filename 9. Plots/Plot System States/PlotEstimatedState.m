function PlotEstimatedState(x_est,dt)
figure('Name','Estimated States')
n = (0:size(x_est,2)-1)*dt;


tiledlayout(2,2)
nexttile
hold on
stairs(n,x_est(1,:))
stairs(n,x_est(2,:))
stairs(n,x_est(3,:))
title('Estimated position')
xlabel('Time (s)')
ylabel("Position (ECI) (km)")
ylim([-8000 8000])
grid on
hold off

nexttile
hold on
stairs(n,x_est(4,:))
stairs(n,x_est(5,:))
stairs(n,x_est(6,:))
title('Estimated velocity')
xlabel('Time (s)')
ylabel("Velocity (ECI) (km/s)")
ylim([-8 8])
grid on
hold off

nexttile
hold on
stairs(n,x_est(7,:))
stairs(n,x_est(8,:))
stairs(n,x_est(9,:))
stairs(n,x_est(10,:))
title('Estimated attitude')
xlabel('Time (s)')
ylabel("Attitude (BOD2ECI)")
ylim([-2 2])
grid on
hold off

nexttile
hold on
stairs(n,x_est(11,:))
stairs(n,x_est(12,:))
stairs(n,x_est(13,:))
title('Estimated angular velocity')
xlabel('Time (s)')
ylabel("Angular velocity (BOD) (rad/s)")
ylim([-1 1])
grid on
hold off
end