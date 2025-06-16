function Plot3D2(x_true)
figure;
set(gcf, 'Color', 'k'); % Black background
hold on

% Add stars
max_range = max(max(abs(x_true(1:3,:)))) * 1.5;
n_stars = 500;
star_x = (rand(n_stars,1) - 0.5) * 2 * max_range;
star_y = (rand(n_stars,1) - 0.5) * 2 * max_range;
star_z = (rand(n_stars,1) - 0.5) * 2 * max_range;
plot3(star_x, star_y, star_z, '.', 'Color', 'w', 'MarkerSize', 1);

% Plot Earth (stationary)
R = 6371;
[X,Y,Z] = ellipsoid(0,0,0,R,R,R,80);
Earth = surf(X,Y,-Z,'FaceColor','none','EdgeColor','none');
CData_image = imread("Earth.jpg") ;
set(Earth,'FaceColor','texturemap','CData',CData_image)

% Initialize empty line for orbit
orbit_line = plot3(NaN, NaN, NaN, 'LineWidth', 2, 'Color', 'r');

title('3D Plot', 'Color', 'w')
xlabel('x-Position (km)', 'Color', 'w')
ylabel('y-Position (km)', 'Color', 'w')
zlabel('z-Position (km)', 'Color', 'w')
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
axis equal

% Animation loop
n_points = size(x_true, 2);
for i = 1:n_points
    % Update orbit line with points up to current position
    set(orbit_line, 'XData', x_true(1, 1:i), ...
                    'YData', x_true(2, 1:i), ...
                    'ZData', x_true(3, 1:i));
    
    drawnow;
    pause(0.00000000000001); % Small delay for animation
end

hold off
end