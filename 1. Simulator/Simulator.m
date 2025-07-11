%==========================================================================
% Simulator
% Niel Theron
% 12-06-2025
%==========================================================================

%% Load Functions =========================================================

% This adds all the functions to the compile path
warning('off');
LoadPath;
CleanFolders;
clearvars;
%---

%==========================================================================
%% Simulation Parameters ==================================================

% Simulation Parameters
st      = 15;                           % Simulation time (s)
dt_p    = 0.1;                          % Sample rate (s)
n_s     = round(st/dt_p);               % Number of samples
n_f     = 10;                           % Number of features
%---

%==========================================================================
%% Initialise Plant =======================================================

% Plant Variables
n_x     = 13;                           % Number of states
x_true  = zeros(n_x,n_s);               % Initialise plant states 
%---

% Plant Constants
Mu_p = 3.986e5;                         % Gravitational parameter (km3/s2)
Re_p = 6.378e3;                         % Radius of Earth (km)
J2_p = 1.082e-3;                        % J2 parameter
I_p  = [1 1 1].';                       % Moment of inertia (kg*m2)
we_p = 0; %7.292e-5;                    % Rotational speed of earth (rad/s)
%---

% Initial States
lat_p   = 48.858;                       % Lattitude (deg)
lon_p   = 1.66;                         % Longitude (deg)
alt_p   = 500;                          % Altitude (km)
rol_p   = 0;                            % Camera roll  Offset B/O (deg)
yaw_p   = 0;                            % Camera Yaw   Offset B/O (deg)
pit_p   = 0;                            % Camera Pitch Offset B/O (deg)
wx_p    = 0;                            % Roll  rate B/O (deg/s)
wy_p    = 10;                            % Yaw   rate B/O (deg/s)
wz_p    = 10;                           % Pitch rate B/O (deg/s) 
t_p     = 0;                            % Initial time (s)
%---

% Initialize orbit
[r_p, v_p, q_I2B_p, w_I2B_p] = ...
InitialiseOrbit( ...
lat_p, lon_p, alt_p, ...
rol_p, yaw_p, pit_p, ...
wx_p, wz_p, wy_p, ...
Mu_p, we_p, t_p);
%---

% Set initial true state
x_true(:,1) = [r_p; v_p; q_I2B_p; w_I2B_p];
%---

%==========================================================================
%% Initialise Camera ======================================================

% Render Environement
ax = RenderEarth();
%---

% Initialise Camera Varaibles
catalogue_geo   = zeros(3,n_f,n_s);     % Catalogue in lla
catalogue_eci   = zeros(3,n_f,n_s);     % Catalogue in ECI
%---

% Camera Variables
imgWidth_cam    = 720;                  % Along track image width (pixels)
imgHeight_cam   = 720;                  % Cross-track image height (pixels)
focalLength_cam = 0.58;                 % Focal length (m)
pixelSize_cam   = 17.4e-6;              % Pixel size (m)
%---

%==========================================================================
%% Initialise EKF =========================================================

% EKF Variables
x_EKF = zeros(n_x,n_s);             % Initialise filter states
P_EKF = zeros(n_x,n_x,n_s);         % Initialise filter covariance matrix
%---

% EKF Constants
Mu_f = 3.986e5;                     % Gravitational parameter (km3/s2)
Re_f = 6.378e3;                     % Radius of Earth (km)
J2_f = 1.082e-3;                    % J2 parameter
I_f = [1 1 1].';                    % Moment of inertia (kg*m2)
we_f = 0; % 7.292e-5;               % Rotational speed of earth (rad/s)
Q_f = TuneQ();                      % Process noise covariance matrix
%---

% Initial States 
lat_f   = 48.858;                   % Latitude  (deg)
lon_f   = 1.66;                     % Longitude (deg)
alt_f   = 500;                      % Altitude  (km)
rol_f   = 0;                        % Camera Roll  offset B/O (deg)
yaw_f   = 0;                        % Camere Yaw   offset B/O (deg)
pit_f   = 0;                        % Camera Pitch offset B/O (deg)
wx_f    = 0;                        % Roll  rate B/O (deg/s)
wy_f    = 10;                        % Yaw   rate B/O (deg/s)
wz_f    = 10;                        % Pitch rate B/O (deg/s)
t_f     = 0;                        % Initial Time (s)
%---

% Initialize EKF orbit
[r_f, v_f, q_I2B_f, w_I2B_f] = ...
InitialiseOrbit( ...
lat_f, lon_f, alt_f, ...
rol_f, yaw_f, pit_f, ...
wx_f, wy_f, wz_f, ...
Mu_f, we_f, t_f);
%---

% Initialize Filter
x_EKF(:,1) = [r_f; v_f; q_I2B_f; w_I2B_f];    % Estimated state
P_EKF(:,:,1) = TuneP();                       % Covariance matrix
%---

%==========================================================================
%% Initialise Earth Tracker ===============================================

% Initialsie Earth Tracker Varaibles
n_ET     = 3;                           % Number of measurements
dt_ET    = 0.2;                           % Earth tracker sample rate (s)
noise_ET = 0;
R_ET     = noise_ET*eye(3);             % Measurement Noise Covariance Matrix

z_ET     = zeros(n_ET,n_f,n_s);         % Earth tracker measurement (km)
y_ET     = zeros(n_ET,n_f,n_s);         % Estimated Earth tracker measurement (km)
K_ET     = zeros(n_x,n_ET,n_f,n_s);     % Earth tracker Kalman Gain (km)
    
f_m = zeros(2,n_f,n_s);                 % Feature Pixel Locations (pixels)
%---

% Earth Tracker Constants
GSD_ET          = 15;                   % Ground sampling distance (m/pixel) 
imgWidth_ET     = 720;                  % Along track image width  (pixels)
imgHeight_ET    = 720;                  % Cross-track image height (pixels)
focalLength_ET  = 0.58;                 % Focal length (m)
pixelSize_ET    = 1.74e-5;              % Pixel size   (m)
%---

%==========================================================================
%% Initialise Direct Sensors ==============================================

% Star Tracker
n_ST        = 4;                        % Number of measurements
dt_ST       = 0.1;                      % Star tracker sampling rate (s)
noise_ST    = 1;                        % Star tracker noise (deg)
R_ST        = deg2rad(noise_ST)^2*eye(n_ST);    % Star tracker noise matrix (rad)

z_ST        = zeros(n_ST,n_s);          % Star tracker measurements
y_ST        = zeros(n_ST,n_s);          % Star tracker estimated measurements
K_ST        = zeros(n_x,n_ST,n_s);      % Star tracker Kalman Gain
%---

% Gyroscope
n_GYR       = 3;                        % Number of measurements
dt_GYR      = 0.1;                      % Gyroscope sampling samping rate (s)
noise_GYR   = 1;                     % Gyroscope sensor noise (deg)
R_GYR       = deg2rad(noise_GYR)^2*eye(3);  % Gyroscope sensor noise matrix
driftRate_GYR = deg2rad(1);          % Gyroscope drift rate (deg/s)

drift_GYR   = zeros(n_GYR,1);           % Gyroscope drift buffer
z_GYR       = zeros(n_GYR,n_s);         % Gyroscope measurement
y_GYR       = zeros(n_GYR,n_s);         % Gyroscope estimated measurement
K_GYR       = zeros(n_x,n_GYR,n_s);     % Gyroscope Kalman Gain
%---

% GPS
n_GPS       = 3;                        % Number of measurements
dt_GPS      = 0.1;                      % GPS sampling samping rate (s)
noise_GPS   = 1;                     % GPS sensor noise (km)
R_GPS       = noise_GPS^2*eye(3);       % GPS noise matrix
driftRate_GPS   = 1;                 % GPS drift rate (km)

drift_GPS   = zeros(n_GPS,1);           % GPS drift buffer
z_GPS       = zeros(n_GPS,n_s);         % GPS measurement
y_GPS       = zeros(n_GPS,n_s);         % GPS estimated measurement
K_GPS       = zeros(n_x,n_GPS,n_s);     % GPS Kalman Gain
%----

%==========================================================================
%% Initialise TRIAD =======================================================

% Magnetometer
n_MAG       = 3;                        % Number of measurements
dt_MAG      = 0.1;                        % Magnetometer sampling rate (s)
noise_MAG   = 0;                        % Magnetometer noise (deg)
angle_MAG   = 0;                        % Declination angle (deg)
z_MAG       = zeros(n_MAG,n_s);         % Magnetometer measurements
%---

% Coarse Sun Sensor
n_CSS       = 3;                        % Number of measurements
dt_CSS      = 0.1;                        % Coarse sun sensor sampling rate (s)
noise_CSS   = 0;                     % Coarse sun sensor noise (deg)
z_CSS       = zeros(n_CSS,n_s);         % Coarse sun sensor measurement
%---

% TRIAD
n_TRIAD       = 4;
% dt_TRIAD      = 0.1;
noise_TRIAD   = 1;
R_TRIAD       = deg2rad(noise_TRIAD*eye(4));    % TRIAD noise matix

z_TRIAD       = zeros(n_TRIAD,n_s);
y_TRIAD       = zeros(n_TRIAD,n_s);     % TRIAD estimated measurement
K_TRIAD       = zeros(n_x,n_TRIAD,n_s); % TRIAD Kalman Gain
%---

%==========================================================================
%% Initialise Simulation ==================================================

% Progress bar
fig = uifigure('Name','Simulation Progress');
d = uiprogressdlg(fig, 'Title','Running Simulation', ...
    'Message','Initializing...', 'Indeterminate','off');
startTime = tic;
%---

%==========================================================================
%% Run Simulation =========================================================
for r = 1:n_s-1

    % Variables -----------------------------------------------------------
    t = r*dt_p;
    %----------------------------------------------------------------------
    
    % Plant ---------------------------------------------------------------
    x_true(:,r+1) = Plant(x_true(:,r), dt_p, I_p, Mu_p, Re_p, J2_p);
    %----------------------------------------------------------------------
    
    % Earth Tracker Measurement -------------------------------------------
    if mod(t,dt_ET) == 0
        % Image generator -----------------------------------------------------
        satelliteImage = GenerateSatelliteImage(ax, x_true(1:3,r), x_true(4:6,r), x_true(7:10,r), imgWidth_cam, imgHeight_cam, focalLength_cam, pixelSize_cam);
        % SaveSatelliteImages(satelliteImage,r);
        % ----------------------------------------------------------------------

        % Feature Detection ---------------------------------------------------
        [f_m(:,:,r), grayImage] = FeaturePixelDetection(satelliteImage, n_f);
        % SaveFeatureImages(grayImage, f_m(:,:,r),r);
        %----------------------------------------------------------------------

        % % Catalogue Creation --------------------------------------------------
        catalogue_geo(:,:,r) = FeatureGeolocation(f_m(:,:,r), satelliteImage, x_true(1:3,r) , x_true(7:10,r), focalLength_cam, pixelSize_cam);
        for i = 1:n_f
            catalogue_eci(:,i,r) = ECR2ECI(LLA2ECR(catalogue_geo(:,i,r)),t,we_p);
        end
        % %----------------------------------------------------------------------

        z_ET(:,:,r) = EarthTracker(f_m(:,:,r),imgWidth_ET,imgHeight_ET,focalLength_ET, pixelSize_ET, GSD_ET);
    else
        z_ET(:,:,r) = zeros(n_ET,n_f);
    end
    %----------------------------------------------------------------------

    % Sensors -----------------------------------------------------------
    [z_GPS(:,r), z_GYR(:,r), z_ST(:,r), z_CSS(:,r), z_MAG(:,r), z_TRIAD(:,r)] = ...
        SampleSensors(x_true(:,r), t, we_p, ...
        dt_GPS, noise_GPS, drift_GPS, driftRate_GPS, ...
        dt_GYR, noise_GYR, drift_GYR, driftRate_GYR, ...
        dt_ST, noise_ST, ...
        dt_CSS, noise_CSS, ...
        dt_MAG, noise_MAG,angle_MAG);
    % --------------------------------------------------------------------

    % EKF ---------------------------------------------------------------
    [y_ET(:,:,r), x_EKF(:,r+1), P_EKF(:,:,r+1), K_ET(:,:,:,r)] = EKF( ...
        catalogue_eci(:,:,r),x_EKF(:,r),P_EKF(:,:,r),I_f,Q_f,dt_p,Mu_f,Re_f,J2_f, ...
        z_ET(:,:,r), z_TRIAD(:,r), z_ST(:,r), z_GPS(:,r), z_GYR(:,r), ...
        R_ET, R_TRIAD, R_ST, R_GPS, R_GYR);
    %----------------------------------------------------------------------

    % Progress bar --------------------------------------------------------
    elapsedTime = toc(startTime);
    progress = r / (n_s-1);
    estTotalTime = elapsedTime / progress;
    estRemaining = estTotalTime - elapsedTime;
    d.Value = progress;
    d.Message = sprintf('Elapsed: %.2fs | %d%% | Est. remaining: %.2fs | Sample: %d | Footage Time: %.2fs | Time per sample: %.2fs', ...
        elapsedTime, round(progress*100), estRemaining,r+1,t,elapsedTime/r);
    %----------------------------------------------------------------------
end

%==========================================================================
%% Clean Processes ========================================================
delete('temp_image.png')
%==========================================================================