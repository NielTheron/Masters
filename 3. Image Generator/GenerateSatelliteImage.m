%==========================================================================
% Niel Theron
% 19-06-2025
%==========================================================================

function Image = GenerateSatelliteImage(ax, r_I, v_I, q_I2B, imgWidth, imgHeight, focalLength, pixelSize)

%% Set Up Figure ==========================================================
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
%---

%% Get rotation matrix ====================================================
% Get inertial to orbital rotation matr
[~,q_I2O] = ECI2ORB([0;0;0],r_I,v_I);
R_I2O = quat2rotm(q_I2O);
%---

% Get the inertial to body rotation matrix
R_I2B = quat2rotm(q_I2B.');
%---

R_O2B = R_I2B*R_I2O.';

%% Apply camera rotation ==================================================

camera_view_direction_body = [0; 0; 1];    % Look along +Z (toward Earth) ✅
camera_up_direction_body   = [0; -1; 0];     % Up is -Y (anti-cross-track)

% Apply body rotation
R_B2O = R_O2B';
viewDirection_O = R_B2O * camera_view_direction_body;
upVector_O= R_B2O * camera_up_direction_body;

% Transform to ECI
viewDirection_eci = R_I2O* viewDirection_O;
upVector_eci = R_I2O * upVector_O;

%---


%% Set Camera =============================================================

campos(ax, r_I.');
target_distance = 500;
camtarget(ax, (r_I + viewDirection_eci * target_distance).');
camproj(ax, 'perspective');
camup(ax, upVector_eci');

% Set FOV
fov_v = 2 * atand((imgHeight * pixelSize / 2) / focalLength);
set(ax, 'CameraViewAngleMode', 'manual');
camva(ax, fov_v);

%---

%% Capture Image ==========================================================

drawnow;
frame = getframe(ax);
Image = imresize(frame.cdata, [imgHeight, imgWidth]);

end