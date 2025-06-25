figure;
hold on

X = zeros(1,size(z_ET,2));
Y = zeros(1,size(z_ET,2));
Z = zeros(1,size(z_ET,2));

% Plot the 3D quivers with smaller arrowheads
quiver3(X,Y,Z, z_ET(1,:,66), z_ET(2,:,66), 0.01*z_ET(3,:,66),'off','LineWidth',2);

% Example: Make the image 4 units wide and 3 units tall
xRange = 10.8;  % Width of the image in 3D space
yRange = 10.8;  % Height of the image in 3D space
zLevel = 5; % Where to place the image along the Z-axis

img = imread('feature_image_066.png');
img = im2double(img);

[xImg, yImg] = meshgrid(linspace(-xRange/2, xRange/2, size(img,2)), ...
                        linspace(-yRange/2, yRange/2, size(img,1)));

zImg = zLevel * ones(size(xImg));

theta = deg2rad(45);  % Rotation angle in radians

% Rotation around Z axis
Ry = [cos(theta), 0, sin(theta);
      0, 1, 0;
     -sin(theta), 0, cos(theta)];

thetaz = deg2rad(0);  % Rotation angle in radians

% Rotation around Z axis
Rz = [cos(thetaz), -sin(thetaz), 0;
      sin(thetaz),  cos(thetaz), 0;
           0    ,     0      , 1];


pts = [xImg(:), yImg(:), zImg(:)]';  % 3 x N
rotPts = Rz * Ry * pts;                  % Rotate

% Reshape back to original grid shape
xRot = reshape(rotPts(1, :), size(xImg));
yRot = reshape(rotPts(2, :), size(yImg));
zRot = reshape(rotPts(3, :), size(zImg));

surface(xRot, yRot, zRot, flipud(img), ...
    'FaceColor', 'texturemap', ...
    'EdgeColor', 'none');

camX = 0;
camY = 0;
camZ = 0.01 * z_ET(3,1,66);  % Use first feature
plotCamera('Location', [camX, camY, camZ], ...
           'Orientation', [1 0 0; 0 -1 0; 0 0 -1], ...
           'Size', 0.5);
axis equal;
xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km/100)');
view(3);
grid on;