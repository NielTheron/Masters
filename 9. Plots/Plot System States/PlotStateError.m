function PlotStateError(xA,x_est,dt)
figure('Name','Position Error')
n = (0:size(xA,2)-1)*dt;

tiledlayout(2,2)
nexttile
hold on
stairs(n,(xA(1,:)-x_est(1,:))*1000)
stairs(n,(xA(2,:)-x_est(2,:))*1000)
stairs(n,(xA(3,:)-x_est(3,:))*1000)
title('Position Error')
xlabel('Time (s)')
ylabel("Position (ECI) (m)")
grid on
hold off

nexttile
hold on
stairs(n,(xA(4,:)-x_est(4,:))*1000)
stairs(n,(xA(5,:)-x_est(5,:))*1000)
stairs(n,(xA(6,:)-x_est(6,:))*1000)
title('Velocity Error')
xlabel('Time (s)')
ylabel("Velocity (ECI) (m/s)")
grid on
hold off

nexttile
hold on
% Vectorized version
dot_products = sum(xA(7:10,:) .* x_est(7:10,:), 1); % Dot product along rows
dot_products = max(-1, min(1, abs(dot_products)));   % Clamp
angular_error = 2 * acos(dot_products);
stairs(n, rad2deg(angular_error));
title('Attitude Error (deg)')
xlabel('Time (s)')
ylabel("Attitude (BOD2ECI)")
grid on
hold off

nexttile
hold on
stairs(n,rad2deg(xA(11,:)-x_est(11,:)))
stairs(n,rad2deg(xA(12,:)-x_est(12,:)))
stairs(n,rad2deg(xA(13,:)-x_est(13,:)))
title('Angular Velocity Error')
xlabel('Time (s)')
ylabel("Angular Velocity (BOD) (deg/s)")
grid on
hold off
end