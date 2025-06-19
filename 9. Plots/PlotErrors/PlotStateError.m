function PlotStateError(xA,x_est,dt)
figure('Name','Position Error')
n = (0:size(xA,2)-1)*dt;

tiledlayout(2,2)
nexttile
hold on
stairs(n,xA(1,:)-x_est(1,:))
stairs(n,xA(2,:)-x_est(2,:))
stairs(n,xA(3,:)-x_est(3,:))
title('Position Error')
xlabel('Time (s)')
ylabel("Position (ECI) (km)")
grid on
hold off

nexttile
hold on
stairs(n,xA(4,:)-x_est(4,:))
stairs(n,xA(5,:)-x_est(5,:))
stairs(n,xA(6,:)-x_est(6,:))
title('Velocity Error')
xlabel('Time (s)')
ylabel("Velocity (ECI) (km\s)")
grid on
hold off

nexttile
hold on
stairs(n,xA(7,:)-x_est(7,:))
stairs(n,xA(8,:)-x_est(8,:))
stairs(n,xA(9,:)-x_est(9,:))
stairs(n,xA(10,:)-x_est(10,:))
title('Attitude Error')
xlabel('Time (s)')
ylabel("Attitude (BOD2ECI)")
grid on
hold off

nexttile
hold on
stairs(n,xA(11,:)-x_est(11,:))
stairs(n,xA(12,:)-x_est(12,:))
stairs(n,xA(13,:)-x_est(13,:))
title('Angular Velocity Error')
xlabel('Time (s)')
ylabel("Angular Velocity (BOD) (rad/s)")
grid on
hold off
end