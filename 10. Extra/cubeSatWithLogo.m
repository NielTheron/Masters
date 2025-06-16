function cubeSatWithLogo(origin, length, logoPath)
    figure;

    % Define cube vertices centered at origin
    [X, Y, Z] = ndgrid([-0.5 0.5]);
    X = X(:); Y = Y(:); Z = Z(:);
    vertices = [X Y Z] * length + origin;

    faces = [
        1 2 4 3;  % Bottom (Z-)
        5 6 8 7;  % Top (Z+)
        1 2 6 5;  % Front (Y+)
        2 4 8 6;  % Right (X+)
        4 3 7 8;  % Back (Y-)
        3 1 5 7   % Left (X-)
    ];

    % Cube body
    patch('Vertices', vertices, 'Faces', faces, ...
        'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'k');

    axis equal;
    view(3); grid on;
    set(gca, 'Color', 'k');
    set(gcf, 'Color', 'k');
    ax = gca;
    ax.XColor = 'w'; ax.YColor = 'w'; ax.ZColor = 'w';
    hold on;

    margin = 0.6;
    xlim(origin(1) + [-margin, margin]*length);
    ylim(origin(2) + [-margin, margin]*length);
    zlim(origin(3) + [-margin, margin]*length);

    panelColor = [0 0.6 1]; % Blue
    panelSize = 0.8 * length;
    offset = 0.01 * length;
    ps = panelSize / 2;

    % === FRONT (Y+) ===
    x = origin(1) + [-ps ps ps -ps];
    y = origin(2) + length/2 + offset;
    z = origin(3) + [-ps -ps ps ps];
    fill3(x, y*ones(1,4), z, panelColor, 'EdgeColor', 'none');

    % === BACK (Y-) ===
    y = origin(2) - length/2 - offset;
    fill3(x, y*ones(1,4), z, panelColor, 'EdgeColor', 'none');

    % === RIGHT (X+) ===
    x = origin(1) + length/2 + offset;
    y = origin(2) + [-ps ps ps -ps];
    fill3(x*ones(1,4), y, z, panelColor, 'EdgeColor', 'none');

    % === BOTTOM (Z−) ===
    z = origin(3) - length/2 - offset;
    x = origin(1) + [-ps ps ps -ps];
    y = origin(2) + [-ps -ps ps ps];
    fill3(x, y, z*ones(1,4), panelColor, 'EdgeColor', 'none');

    % === Circle on LEFT face (X−) ===
    r = 0.1 * length;
    theta = linspace(0, 2*pi, 50);
    x = origin(1) - length/2 - offset;
    y = origin(2) + r * cos(theta);
    z = origin(3) + r * sin(theta);
    fill3(x*ones(1,50), y, z, [0.8 0.8 0.8], 'EdgeColor', 'none');

    % === ESL Logo on TOP (Z+) ===
    if exist(logoPath, 'file')
        [logoImg, ~, alpha] = imread(logoPath);
        logoImg = im2double(logoImg);

        % Coordinates for top face (Z+)
        x = origin(1) + [-0.5 0.5; -0.5 0.5] * length;
        y = origin(2) + [-0.5 -0.5; 0.5 0.5] * length;
        z = ones(2,2) * (origin(3) + length/2 + 0.001);

        surface(-x, y, z, ...
            'CData', logoImg, ...
            'FaceColor', 'texturemap', ...
            'EdgeColor', 'none', ...
            'AlphaData', double(alpha)/255, ...
            'FaceAlpha', 'texturemap');
    else
        warning('Logo image not found at: %s', logoPath);
    end

    hold off;
end
