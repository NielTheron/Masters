function Plot3D(xA)

hold on
plot3(xA(1,:),xA(2,:),xA(3,:),LineWidth=2)
% plot3(x_est(1,:),x_est(2,:),x_est(3,:),LineWidth=2)
% plot3(yA(1,:),yA(2,:),yA(3,:),LineWidth=2)
% plot3(y_est(1,:),y_est(2,:),y_est(3,:),LineWidth=2)
% scatter3(c(1,:),c(2,:),c(3,:),"filled",'MarkerFaceColor','red')

R = 6371;
[X,Y,Z] = ellipsoid(0,0,0,R,R,R,80);
Earth = surf(X,Y,-Z,'FaceColor','none','EdgeColor','none');
CData_image = imread("Earth.jpg") ;
set(Earth,'FaceColor','texturemap','CData',CData_image)


title('3D Plot')
xlabel('x-Position (km)')
ylabel('y-Position (km)')
zlabel('z-Position (km)')


% xlim([4636 9378])
% ylim([-869 2822])
% zlim([-2070 2565])
% view([22 32])
axis equal
hold off

end