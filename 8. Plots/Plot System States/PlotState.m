function PlotState(xA,dt)
figure('Name','Real States')

n = (0:size(xA,2)-1)*dt;
tiledlayout(2,2)

nexttile
hold on
stairs(n,xA(1,:))
stairs(n,xA(2,:))
stairs(n,xA(3,:))
title('Real position')
xlabel('Time (s)')
ylabel("Position (ECI) (km)")
ylim([-8000 8000])
grid on
hold off

nexttile
hold on
stairs(n,xA(4,:))
stairs(n,xA(5,:))
stairs(n,xA(6,:))
title('Real velocity')
xlabel('Time (s)')
ylabel("Velocity (ECI) (km/s)")
ylim([-8 8])
grid on
hold off

nexttile
hold on
stairs(n,xA(7,:))
stairs(n,xA(8,:))
stairs(n,xA(9,:))
stairs(n,xA(10,:))
title('Real attitude')
xlabel('Time (s)')
ylabel("Attitude (BOD2ECI)")
ylim([-2 2])
grid on
hold off

nexttile
hold on
stairs(n,xA(11,:))
stairs(n,xA(12,:))
stairs(n,xA(13,:))
title('Real angular velocity')
xlabel('Time (s)')
ylabel("Angular velocity (BOD) (rad/s)")
ylim([-1 1])
grid on
hold off
end