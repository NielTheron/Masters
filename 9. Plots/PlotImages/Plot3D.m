function Plot3D(x_true)

figure;
set(gcf, 'Color', 'k'); % Black background
hold on
plot3(x_true(1,:),x_true(2,:),x_true(3,:),LineWidth=2,Color='r')

R = 6371;
[X,Y,Z] = ellipsoid(0,0,0,R,R,R,80);
Earth = surf(X,Y,-Z,'FaceColor','none','EdgeColor','none');
CData_image = imread("Earth.jpg") ;
set(Earth,'FaceColor','texturemap','CData',CData_image)

view(75,30);
title('3D Plot', 'Color', 'w')
xlabel('x-Position (km)', 'Color', 'w')
ylabel('y-Position (km)', 'Color', 'w')
zlabel('z-Position (km)', 'Color', 'w')
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
axis equal
grid on
hold off

end