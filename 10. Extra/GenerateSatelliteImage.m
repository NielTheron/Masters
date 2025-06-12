%==========================================================================
% Niel Theron
% 05-06-2025
%==========================================================================
% The purpose of this function is given the camera parameters and the
% satellite position to create a satellite image
%==========================================================================
% INPUT:
% ax            : Axes object to draw the image
% r_sat         : Position vector (km) (3x1)
% imgWidth      : The along track image size (pixels)
% imgHeight     : The cross-track image size (pixels)
% focalLength   : The focal length of the camera (m)
% pixelSize     : The size of a pixel (m)
% 
% OUTPUT:
% Image         : The Satellite Image
%
% VARIABLES:
% sat_lla       : Satellite LLA position vector (deg,deg,km) (3x1)
% nadir_lla     : Point directly underneath Satellite (deg,deg,km) (3x1)
% r_nadir       : Targat position vector (km) (3x1)
% viewDirection : The direction the camera is viewing
% upVector      : Vector that indicates the up direction of the satellite
% sensorWidth   : The width of the image capturing sensor (m)
% sensorHeight  : The height of the image capuring sensor (m)
% fov_v         : The vertical FOV of the camera (deg)
% frame         : The image in another form
%==========================================================================

function Image = GenerateSatelliteImage(ax, r_sat, imgWidth, imgHeight, focalLength, pixelSize)

    % Set Up Figure
    fig = ancestor(ax, 'figure');
    set(fig, 'Visible', 'off');  % Offscreen rendering
    fig.Units = 'pixels';
    fig.Position = [100, 100, imgWidth, imgHeight];
    ax.Units = 'normalized';
    ax.Position = [0, 0, 1, 1];
    set(ax, 'LooseInset', [0 0 0 0]);
    axis(ax, 'equal');
    axis(ax, 'off');
    pbaspect(ax, [imgWidth imgHeight 1]);  % Maintain consistent aspect
    daspect(ax, [1 1 1]);
    %---

    % Get nardir position
    sat_lla = ecef2lla(r_sat.' * 1000).';
    nadir_lla = [sat_lla(1) sat_lla(2) 0].';
    r_nadir = (lla2ecef(nadir_lla.', "WGS84") / 1000).';
    %---

    % Set satellite camera
    campos(ax, r_sat);
    camtarget(ax, r_nadir);
    camproj(ax, 'perspective');
    %---
    
    % Compute camera up vector
    viewDirection = r_nadir - r_sat;
    viewDirection = viewDirection / norm(viewDirection);
    z_global = [0; 0; 1];
    if abs(dot(viewDirection, z_global)) > 0.99
        z_global = [0; 1; 0];  % Avoid near-parallel
    end
    right = cross(viewDirection, z_global);
    upVector = cross(right, viewDirection);
    upVector = upVector / norm(upVector);
    camup(ax, upVector.');
    %---
    
    % Calculate field of view
    sensorWidth = imgWidth * pixelSize;   % meters
    sensorHeight = imgHeight * pixelSize; % meters
    set(ax, 'CameraViewAngleMode', 'manual');
    fov_v = 2 * atand((sensorHeight / 2) / focalLength);  % Vertical FOV in degrees
    camva(ax, fov_v);
    %---

    % Get Image
    drawnow;
    frame = getframe(ax);
    Image = frame.cdata;
    Image = imresize(Image, [imgHeight, imgWidth]);
    %---
end