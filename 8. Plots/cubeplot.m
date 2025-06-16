function cubeplot(origin, length)
    figure;

    % Define cube vertices centered at origin
    [X, Y, Z] = ndgrid([-0.5 0.5]);
    X = X(:); Y = Y(:); Z = Z(:);
    vertices = [X Y Z] * length + origin;

    % Define faces
    faces = [
        1 2 4 3;  % Bottom
        5 6 8 7;  % Top
        1 2 6 5;  % Front
        2 4 8 6;  % Right
        4 3 7 8;  % Back
        3 1 5 7   % Left
    ];

    % Draw the cube
    patch('Vertices', vertices, 'Faces', faces, ...
        'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'k', 'FaceAlpha', 1);

    % Setup view
    axis equal;
    view(3);
    grid on;
    set(gca, 'Color', 'k');
    set(gcf, 'Color', 'k');
    ax = gca;
    ax.XColor = 'w'; ax.YColor = 'w'; ax.ZColor = 'w';

    % Limits
    margin = 0.6;
    xlim(origin(1) + [-margin, margin]*length);
    ylim(origin(2) + [-margin, margin]*length);
    zlim(origin(3) + [-margin, margin]*length);

    hold on;

    % ===== Add Solar Panels (now slightly offset outwards) =====
    panelColor = [0 0.6 1]; % Deep blue
    panelSize = 0.8 * length;
    offset = 0.01 * length; % Offset outside the cube surface

    ps = panelSize / 2;

    % === Front face (y = +half + offset) ===
    x = origin(1) + [-ps ps ps -ps];
    y = origin(2) + length/2 + offset;
    z = origin(3) + [-ps -ps ps ps];
    fill3(x, y*ones(1,4), z, panelColor, 'EdgeColor', 'none');

    % === Right face (x = +half + offset) ===
    x = origin(1) + length/2 + offset;
    y = origin(2) + [-ps ps ps -ps];
    z = origin(3) + [-ps -ps ps ps];
    fill3(x*ones(1,4), y, z, panelColor, 'EdgeColor', 'none');

    % === Top face (z = +half + offset) ===
    x = origin(1) + [-ps ps ps -ps];
    y = origin(2) + [-ps -ps ps ps];
    z = origin(3) + length/2 + offset;
    fill3(x, y, z*ones(1,4), panelColor, 'EdgeColor', 'none');

    % === ESL LOGO on front face ===
    text(origin(1), origin(2) + length/2 + 2*offset, origin(3), ...
        'ESL', ...
        'Color', 'white', 'FontSize', 16, ...
        'HorizontalAlignment', 'center', ...
        'FontWeight', 'bold', ...
        'Rotation', 0);

    hold off;
end
