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

    % --- Define LVLH (Orbital) Reference Frame (Z-nadir, X-along-track, Y-cross-track) ---
    e_z_LVLH = -r_sat / norm(r_sat); % Z-axis: Nadir pointing

    if nargin < 3 || isempty(v_sat) || all(v_sat == 0) % Check if v_sat is provided and non-zero
        warning('Velocity vector (v_sat) not provided or is zero. Using simplified LVLH X and Y axes.'); %
        % Simplified X and Y if velocity is not available
        % This assumes an orbit roughly aligned with the equatorial plane for X,Y approximation
        % It's NOT robust for all orbits.
        z_global = [0; 0; 1]; % Earth's Z-axis
        e_y_LVLH = cross(e_z_LVLH, z_global); % Approximating cross-track, aiming for perpendicular to Earth's Z
        if norm(e_y_LVLH) < 1e-6 % If e_z_LVLH is parallel to z_global (e.g., satellite at poles)
            z_global = [0; 1; 0]; % Use a different reference vector
            e_y_LVLH = cross(e_z_LVLH, z_global); %
        end
        e_y_LVLH = e_y_LVLH / norm(e_y_LVLH); %
        e_x_LVLH = cross(e_y_LVLH, e_z_LVLH); % X-axis: completes right-handed system (approx along-track)
        e_x_LVLH = e_x_LVLH / norm(e_x_LVLH); %
    else
        % Robust LVLH Frame (requires velocity vector)
        % X-axis: Along-track, in direction of velocity component perpendicular to r_sat
        % First, project velocity onto the plane perpendicular to r_sat
        r_unit = r_sat / norm(r_sat); %
        v_perp = v_sat - dot(v_sat, r_unit) * r_unit; %
        e_x_LVLH = v_perp / norm(v_perp); % X-axis: Along-track

        % Y-axis: Cross-track, perpendicular to orbital plane
        e_y_LVLH = cross(e_z_LVLH, e_x_LVLH); % Y-axis: Cross-track
        e_y_LVLH = e_y_LVLH / norm(e_y_LVLH); % Ensure it's a unit vector
    end

    % Form the rotation matrix from LVLH (orbital) frame to ECEF frame
    R_LVLH_to_ECEF = [e_x_LVLH, e_y_LVLH, e_z_LVLH]; %
    %---

    % --- Camera Orientation ---
    % Define the camera's fixed orientation within the satellite's BODY frame.
    % Assuming a standard camera setup where:
    % Camera +X (forward) is typically the viewing direction.
    % Camera +Y (left/right) is typically the cross-track direction relative to the view.
    % Camera +Z (up/down) is typically the up/down direction relative to the view.

    % If your camera looks along the *negative Z-axis* of its own sensor, and its 'up' is +Y, then:
    camera_view_direction_body = [0; 0; -1]; % Camera looks along negative Z in its body frame
    % Corrected for 180-degree roll issue:
    camera_up_direction_body = [0; -1; 0];    % Camera 'up' is along NEGATIVE Y in its body frame

    % Convert the orbital-to-body quaternion to a rotation matrix
    % No transpose needed for scalar-first quaternion input to quat2rotm
    R_body_to_LVLH = quat2rotm(q_orbital_to_body); %

    % Transform the camera's body-frame view and up vectors to the LVLH frame
    viewDirection_LVLH = R_body_to_LVLH * camera_view_direction_body; %
    upVector_LVLH = R_body_to_LVLH * camera_up_direction_body; %

    % Transform these vectors from the LVLH frame to the ECEF frame
    viewDirection_ecef = R_LVLH_to_ECEF * viewDirection_LVLH; %
    upVector_ecef = R_LVLH_to_ECEF * upVector_LVLH; %

    % Normalize vectors
    viewDirection_ecef = viewDirection_ecef / norm(viewDirection_ecef); %
    upVector_ecef = upVector_ecef / norm(upVector_ecef); %
    %---

    % --- Set MATLAB Camera Parameters ---
    campos(ax, r_sat); % Camera position is the satellite position

    % Calculate the camera target based on the view direction
    % We need a point that is 'far enough' along the view direction for visualization.
    % A large distance, e.g., 1000 km, should suffice.
    r_target_ecef = r_sat + viewDirection_ecef * 1000; % Target 1000 km along the view direction
    camtarget(ax, r_target_ecef); % Use the calculated target

    camproj(ax, 'perspective'); % Always use perspective projection for realism
    camup(ax, upVector_ecef.'); % Use the calculated up vector (MATLAB expects row vector)
    %---

    % --- Calculate Field of View ---
    sensorWidth = imgWidth * pixelSize;   % meters
    sensorHeight = imgHeight * pixelSize; % meters
    set(ax, 'CameraViewAngleMode', 'manual'); %
    fov_v = 2 * atand((sensorHeight / 2) / focalLength);  % Vertical FOV in degrees
    camva(ax, fov_v); %
    %---

    % --- Get Image ---
    drawnow; %
    frame = getframe(ax); %
    Image = frame.cdata; %
    Image = imresize(Image, [imgHeight, imgWidth]); %
    %---
end