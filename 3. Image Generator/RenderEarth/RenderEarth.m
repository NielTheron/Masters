%==========================================================================
% Niel Theron
% 05-06-2025
%==========================================================================
% The purpose of this function is to render an Earth with a high resolution
% image patch on the surface
%==========================================================================
% OUTPUT:
% ax                : Axes object    
%
% VARAIBLES:
% imgPatch          : Georeferenced image
% R                 : Spatial reference
% imgBase           : Base Earth image
% [xs, ys, zs]      : Sphere surface
% [n_cols, n_rows]  : The number of columns and rolls in the image
% fig               : Figure object
% lat_vec           : The points of lattitude on the image
% lon_vec           : The points of longitude on the image
% [lat_patch, lon_patch]        : The meshgrid on lattitude and longitude points
% [x_patch, y_patch, z_patch]   : The project patch on the WGS84 ellipsoid
%   
% CONSTANTS:
% a                 : Eqautorial radius for the WGS84 ellipsoid (km)
% b                 : Polar radius for the WGS84 ellipsoid (km)
% n                 : Number of points on the sphere
%==========================================================================

function ax = RenderEarth()

    % Load base Earth texture and patch
    [imgPatch, R] = readgeoraster('ParisStrip.tif');
    if size(imgPatch,3) == 4
        imgPatch = imgPatch(:,:,1:3);
    end
    imgPatch = im2double(imgPatch);

    imgBase = imread('Earth.jpg');
    if size(imgBase,3) == 4
        imgBase = imgBase(:,:,1:3);
    end
    imgBase = im2double(imgBase);
    %---

    % WGS84 ellipsoid constants
    a = 6378.137;    % Equatorial radius [km]
    b = 6356.752;    % Polar radius [km]

    n = 200;
    [xs, ys, zs] = sphere(n);

    fig = figure('Color', 'k', 'Visible', 'off');
    ax = axes('Parent', fig);
    hold(ax, 'on');
    axis(ax, 'equal');
    axis(ax, 'off');

    surface(ax, xs*a, ys*a, -zs*b, imgBase, ...
        'FaceColor', 'texturemap', 'EdgeColor', 'none');
    %---

    % Patch
    [n_rows, n_cols, ~] = size(imgPatch);
    lon_vec = linspace(R.LongitudeLimits(1), R.LongitudeLimits(2), n_cols);
    lat_vec = linspace(R.LatitudeLimits(2), R.LatitudeLimits(1), n_rows);
    [lon_patch, lat_patch] = meshgrid(lon_vec, lat_vec);
    [x_patch, y_patch, z_patch] = geodetic2ecef(wgs84Ellipsoid('km'), lat_patch, lon_patch, 0);

    surface(ax, x_patch, y_patch, z_patch, imgPatch, ...
        'FaceColor', 'texturemap', 'EdgeColor', 'none');
    %---

    % Visual parameters
    lighting(ax, 'gouraud');
    material(ax, 'dull');
    camlight(ax, 'headlight');
    %---
    
end
