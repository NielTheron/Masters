%==========================================================================
% Niel Theron
% 05-06-2025
%==========================================================================
% The purpose of this function is given the camera parameters and the
% satellite position to create a satellite image
%==========================================================================
% INPUT:
% ax            : Axes object to draw the image
% r_sat         : Position vector (km) (3x1) - Satellite position in ECEF
% v_sat         : Velocity vector (km/s) (3x1) - Satellite velocity in ECEF (for robust LVLH)
%                 If not provided or zero, a simplified LVLH is used.
% q_orbital_to_body : Quaternion representing the rotation from the orbital
%                 (LVLH) frame to the body frame. (4x1, scalar first [qw; vx; vy; vz])
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
% r_nadir       : Target position vector (km) (3x1)
% viewDirection : The direction the camera is viewing
% upVector      : Vector that indicates the up direction of the satellite
% sensorWidth   : The width of the image capturing sensor (m)
% sensorHeight  : The height of the image capuring sensor (m)
% fov_v         : The vertical FOV of the camera (deg)
% frame         : The image in another form
%==========================================================================

function Image = GenerateSatelliteImage(ax, r_sat, v_sat, q_orbital_to_body, imgWidth, imgHeight, focalLength, pixelSize)
%==========================================================================
% Simplified Satellite Image Generator
% Niel Theron - 12-06-2025
%==========================================================================

    % Set Up Figure
    fig = ancestor(ax, 'figure');
    set(fig, 'Visible', 'off');
    fig.Units = 'pixels';
    fig.Position = [100, 100, imgWidth, imgHeight];
    ax.Units = 'normalized';
    ax.Position = [0, 0, 1, 1];
    set(ax, 'LooseInset', [0 0 0 0]);
    axis(ax, 'equal');
    axis(ax, 'off');
    pbaspect(ax, [imgWidth imgHeight 1]);
    daspect(ax, [1 1 1]);

    % Define LVLH Frame (Z=nadir, X=along-track, Y=cross-track)
    e_z_LVLH = -r_sat / norm(r_sat);                    % Nadir
    r_unit = r_sat / norm(r_sat);
    v_perp = v_sat - dot(v_sat, r_unit) * r_unit;
    e_x_LVLH = v_perp / norm(v_perp);                   % Along-track
    e_y_LVLH = cross(e_z_LVLH, e_x_LVLH);               % Cross-track
    
    % LVLH to ECEF rotation matrix
    R_LVLH_to_ECEF = [e_x_LVLH, e_y_LVLH, e_z_LVLH];

    % Camera directions in body frame
    camera_view_direction_body = [0; 0; 1];             % Look along +Z (nadir)
    camera_up_direction_body = [0; -1; 0];              % Up is -Y

    % Apply body rotation
    R_body_to_LVLH = quat2rotm(q_orbital_to_body');
    viewDirection_LVLH = R_body_to_LVLH * camera_view_direction_body;
    upVector_LVLH = R_body_to_LVLH * camera_up_direction_body;

    % Transform to ECEF
    viewDirection_ecef = R_LVLH_to_ECEF * viewDirection_LVLH;
    upVector_ecef = R_LVLH_to_ECEF * upVector_LVLH;

    % Set MATLAB camera
    campos(ax, r_sat');
    camtarget(ax, (r_sat + viewDirection_ecef * 1000)');
    camproj(ax, 'perspective');
    camup(ax, upVector_ecef');

    % Set FOV
    fov_v = 2 * atand((imgHeight * pixelSize / 2) / focalLength);
    set(ax, 'CameraViewAngleMode', 'manual');
    camva(ax, fov_v);

    % Capture image
    drawnow;
    frame = getframe(ax);
    Image = imresize(frame.cdata, [imgHeight, imgWidth]);
end