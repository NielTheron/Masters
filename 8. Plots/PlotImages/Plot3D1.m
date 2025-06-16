function Plot3D1(x_true)
figure;
hold on

% Plot Earth (stationary)
R = 6371;
[X,Y,Z] = ellipsoid(0,0,0,R,R,R,80);
Earth = surf(X,Y,-Z,'FaceColor','none','EdgeColor','none');
CData_image = imread("Earth.jpg") ;
set(Earth,'FaceColor','texturemap','CData',CData_image)

% Initialize empty line for orbit
orbit_line = plot3(NaN, NaN, NaN, 'LineWidth', 2, 'Color', 'r');

title('3D Plot')
xlabel('x-Position (km)')
ylabel('y-Position (km)')
zlabel('z-Position (km)')
axis equal

% Animation loop
n_points = size(x_true, 2);
for i = 1:n_points
    % Update orbit line with points up to current position
    set(orbit_line, 'XData', x_true(1, 1:i), ...
                    'YData', x_true(2, 1:i), ...
                    'ZData', x_true(3, 1:i));
    
    drawnow;
    pause(0.0001); % Small delay for animation
end

hold off
end