function detailedCubeSat(origin, length, logoPath)
 figure;
% Cube vertices centered at origin
 [X, Y, Z] = ndgrid([-0.5 0.5]);
 X = X(:); Y = Y(:); Z = Z(:);
 vertices = [X Y Z] * length + origin;
 faces = [
 1 2 4 3; % Bottom (Z-)
 5 6 8 7; % Top (Z+)
 1 2 6 5; % Front (Y+)
 2 4 8 6; % Right (X+)
 4 3 7 8; % Back (Y-)
 3 1 5 7 % Left (X-)
 ];
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
% === Solar panel settings ===
 panelColor = [0 0.6 1];
 nSplits = 2;
 panelWidth = 0.8 * length;
 panelHeight = panelWidth / nSplits;
 offset = 0.01 * length;
% Define grid-patch helper
function addSplitPanels(faceNormal, up, right, center)
for i = 1:nSplits
 localOffset = (i - 0.5 - nSplits/2) * panelHeight;
 c = center + up * localOffset;
 verts = [
 c - right*panelWidth/2 - up*panelHeight/2;
 c + right*panelWidth/2 - up*panelHeight/2;
 c + right*panelWidth/2 + up*panelHeight/2;
 c - right*panelWidth/2 + up*panelHeight/2
 ];
 patch('Faces', [1 2 3 4], 'Vertices', verts, ...
'FaceColor', panelColor, 'EdgeColor', 'k'); % black gridlines
end
end
% === Add Panels ===
% Front (Y+)
 addSplitPanels([0 1 0], [0 0 1], [1 0 0], origin + [0 length/2 + offset 0]);
% Back (Y−)
 addSplitPanels([0 -1 0], [0 0 1], [1 0 0], origin + [0 -length/2 - offset 0]);
% Right (X+)
 addSplitPanels([1 0 0], [0 0 1], [0 1 0], origin + [length/2 + offset 0 0]);
% Bottom (Z−)
 addSplitPanels([0 0 -1], [0 1 0], [1 0 0], origin + [0 0 -length/2 - offset]);
% Top (Z+)
 addSplitPanels([0 0 1], [0 1 0], [1 0 0], origin + [0 0 length/2 + offset]);
% === Circle on LEFT face ===
 r_outer = 0.1 * length;
 r_inner = 0.05 * length;
 theta = linspace(0, 2*pi, 100);
 x_left = origin(1) - length/2 - offset;
 y_circle = origin(2) + r_outer * cos(theta);
 z_circle = origin(3) + r_outer * sin(theta);
 fill3(x_left*ones(1,100), y_circle, z_circle, [1 1 1], 'EdgeColor', 'none');
% Inner black circle
 y_inner = origin(2) + r_inner * cos(theta);
 z_inner = origin(3) + r_inner * sin(theta);
 fill3(x_left*ones(1,100), y_inner, z_inner, [0 0 0], 'EdgeColor', 'none');
% === ESL LOGO on LEFT FACE (rotated and centered) ===
logoW = 0.3 * length; logoH = 0.15 * length;
cx = origin(1) - length/2 - 0.002; % Small offset from left face
cy = origin(2); % Centered in Y direction
cz = origin(3) + 0.25 * length; % Above the circle

if exist(logoPath, 'file')
    [logoImg, ~, alpha] = imread(logoPath);
    logoImg = im2double(logoImg);
    
    % Rotate the image 90 degrees
    logoImg = rot90(logoImg);
    
    % Create the surface for the logo
    x = cx * ones(2, 2);
    y = [cy-logoW/2, cy+logoW/2; cy-logoW/2, cy+logoW/2];
    z = [cz-logoH/2, cz-logoH/2; cz+logoH/2, cz+logoH/2];
    
    surf(x, y, z, logoImg, ...
        'FaceColor', 'texturemap', ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 1);
else
    warning('Logo image not found at: %s', logoPath);
    % Add text as fallback
    text(cx, cy, cz, 'ESL', 'Color', 'white', 'FontSize', 12, ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
end


title(ax,"ESL CUBESAT","Color",'w')
xlabel(ax,"x-position (cm)")
ylabel(ax,"y-position (cm)")
zlabel(ax,"z-position (cm)")



 hold off;
end