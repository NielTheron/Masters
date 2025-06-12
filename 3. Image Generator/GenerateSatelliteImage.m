function Image = GenerateSatelliteImage(ax, r_sat, v_sat, q_body_to_eci, imgWidth, imgHeight, focalLength, pixelSize)
%==========================================================================
% Satellite Image Generator for New Body Frame Convention
% Body Frame: +X=along-track, +Y=anti-velocity, +Z=nadir
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

    %======================================================================
    % STEP 1: Define LVLH (Orbital) Frame
    %======================================================================
    e_z_LVLH = -r_sat / norm(r_sat);                    % Nadir (toward Earth)
    r_unit = r_sat / norm(r_sat);
    v_perp = v_sat - dot(v_sat, r_unit) * r_unit;
    e_x_LVLH = v_perp / norm(v_perp);                   % Along-track
    e_y_LVLH = cross(e_z_LVLH, e_x_LVLH);               % Cross-track
    
    R_LVLH_to_ECI = [e_x_LVLH, e_y_LVLH, e_z_LVLH];

    %======================================================================
    % STEP 2: Convert Quaternions  
    %======================================================================
    q_norm = q_body_to_eci / norm(q_body_to_eci);
    qs = q_norm(1); qx = q_norm(2); qy = q_norm(3); qz = q_norm(4);
    
    R_body_to_eci = [1-2*(qy^2+qz^2), 2*(qx*qy-qs*qz), 2*(qx*qz+qs*qy);
                     2*(qx*qy+qs*qz), 1-2*(qx^2+qz^2), 2*(qy*qz-qs*qx);
                     2*(qx*qz-qs*qy), 2*(qy*qz+qs*qx), 1-2*(qx^2+qy^2)];
    
    R_ECI_to_LVLH = R_LVLH_to_ECI';
    R_orbital_to_body = R_body_to_eci * R_ECI_to_LVLH;

    %======================================================================
    % STEP 3: Camera Orientation - Body Frame Aligned with Orbital Frame
    %======================================================================
    % Now body frame = orbital frame, so:
    % Body +X = along-track, Body +Y = cross-track, Body +Z = nadir (toward Earth)
    
    camera_view_direction_body = [0; 0; 1];    % Look along +Z (toward Earth) ✅
    camera_up_direction_body = [0; -1; 0];     % Up is -Y (anti-cross-track)

    % Apply body rotation
    R_body_to_LVLH = R_orbital_to_body';
    viewDirection_LVLH = R_body_to_LVLH * camera_view_direction_body;
    upVector_LVLH = R_body_to_LVLH * camera_up_direction_body;

    % Transform to ECI
    viewDirection_eci = R_LVLH_to_ECI * viewDirection_LVLH;
    upVector_eci = R_LVLH_to_ECI * upVector_LVLH;

    %======================================================================
    % STEP 4: Set Camera
    %======================================================================
    campos(ax, r_sat');
    target_distance = 1000;
    camtarget(ax, (r_sat + viewDirection_eci * target_distance)');
    camproj(ax, 'perspective');
    camup(ax, upVector_eci');

    % Set FOV
    fov_v = 2 * atand((imgHeight * pixelSize / 2) / focalLength);
    set(ax, 'CameraViewAngleMode', 'manual');
    camva(ax, fov_v);

    %======================================================================
    % STEP 5: Capture Image
    %======================================================================
    drawnow;
    frame = getframe(ax);
    Image = imresize(frame.cdata, [imgHeight, imgWidth]);
    
    %======================================================================
    % Debug output (optional)
    %======================================================================
    % fprintf('Body frame: +X=along-track, +Y=anti-velocity, +Z=nadir\n');
    % fprintf('Camera pointing along body +Z toward Earth\n');
    
end