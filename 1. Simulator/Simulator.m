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
simulationTime  = 0.2;                   % Simulation time (s)
dt_p            = 0.1;                  % Sample rate (s)
n_s = simulationTime/dt_p;              % Number of samples
n_f             = 20;                   % Number of features
%---

%==========================================================================
%% Initialise Plant =======================================================
% Plant Variables
n_x = 13;                       % Number of states
x_true = zeros(n_x,n_s);        % Initialise plant states
%---

% Plant Constants
Mu_p = 398600;                  % Gravitational parameter (km3/s2)
Re_p = 6378;                    % Radius of Earth (km)
J2_p = 1.08263e-3;              % J2 parameter
I_p = [1 1 1].';                % Moment of inertia (kg*m2)
we_p = 0; %7.2921159e-5;            % Rotational speed of earth (rad/s)
%---

% Initial States
lat_p = 48.858;                 % Lattitude (deg)
lon_p = 1.66;                   % Longitude (deg)
alt_p = 500;                    % Altitude (km)
yawRate_BO_p = 0.0;             % Yaw rate B/O (deg/s)
pitchRate_BO_p = 0.0;           % Pitch rate B/O (deg/s)
rollRate_BO_p = 0.0;            % Roll rate B/O (deg/s)
%---

% Initialize orbit (converts B/O rates to B/I rates automatically)
[r_p, v_p, q_bod_to_eci_init, w_B_I] = InitialiseOrbitAligned(lat_p, lon_p, alt_p, ...
                                         rollRate_BO_p, pitchRate_BO_p, yawRate_BO_p);
%---

% Set camera pointing (relative to orbital frame)
yaw_p = 0;                     % Camera Yaw Offset B/O (deg)
pit_p = 0;                      % Camera Pitch Offset B/O(deg)
rol_p = 0;                      % Camera roll Offset B/O (deg)
q_bod_to_orb = CalculateCameraPointing(yaw_p, pit_p, rol_p);
%---

% Update ECI quaternion for initial state
q_bod_to_eci_current = UpdateBodyToECI(q_bod_to_orb, r_p, v_p);
%---

% Set initial true state (with B/I angular velocity)
x_true(:,1) = [r_p; v_p; q_bod_to_eci_current(:); w_B_I];
%---

%==========================================================================
%% Initialise Camera ======================================================

% Render Environement
ax = RenderEarth();
%---

% Initialise Camera Varaibles
catalogue_geo   = zeros(2,n_f,n_s);     % Catalogue in lla
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
Mu_f = 398600;                      % Gravitational parameter (km3/s2)
Re_f = 6378;                        % Radius of Earth (km)
J2_f = 1.08263e-3;                  % J2 parameter
I_f = [1 1 1].';                    % Moment of inertia (kg*m2)
we_f = 7.2921159e-5;                % Rotational speed of earth (rad/s)
Q_f = TuneQ(dt_p);                  % Process noise covariance matrix
%---

% Initial States 
lat_f = 48.858715;                  % Latitude (deg)
lon_f = 1.00;                       % Longitude (deg) - offset from truth
alt_f = 500;                        % Altitude (km)
yawRate_BO_f = 0.05;                % Yaw rate B/O (deg/s) - different from truth
pitchRate_BO_f = 0.0;               % Pitch rate B/O (deg/s)
rollRate_BO_f = 0.0;                % Roll rate B/O (deg/s)
%---

% Initialize EKF orbit
[r_f, v_f, q_bod_to_eci_init_f, w_B_I_f] = InitialiseOrbitAligned(lat_f, lon_f, alt_f, ...
                                            rollRate_BO_f, pitchRate_BO_f, yawRate_BO_f);
%---

% Set initial EKF camera pointing (could be different from truth)
yaw_f = 30;                         % Camere yaw offset deg
pit_f = 0;                          % Camera pitch offset deg 
rol_f = 0;                          % Camera roll offset deg
q_bod_to_orb_f = CalculateCameraPointing(yaw_f, pit_f, rol_f);
%---

% Update ECI quaternion for initial EKF state
q_bod_to_eci_f = UpdateBodyToECI(q_bod_to_orb_f, r_f, v_f);
%---

% Initialize Filter
x_EKF(:,1) = [r_f; v_f; q_bod_to_eci_f(:); w_B_I_f];    % Estimated state
P_EKF(:,:,1) = TuneP();                                 % Covariance matrix
%---

%==========================================================================
%% Initialise Earth Tracker ===============================================

% Initialsie Earth Tracker Varaibles
n_ET     = 3;                               % Number of measurements
dt_ET    = 1;                               % Earth tracker sample rate (s)
noise_ET = 0;
R_ET     = noise_ET*eye(3);                 % Measurement Noise Covariance Matrix

z_ET     = zeros(n_ET,n_f,n_s);              % Earth tracker measurement
y_ET     = zeros(n_ET,n_f,n_s);             % Estimated Earth tracker measurement
K_ET     = zeros(n_x,n_ET,n_f,n_s);         % Earth tracker Kalman Gain
    
f_m = zeros(2,n_f,n_s);                     % Feature Pixel Locations (Pixels)
%---

% Earth Tracker Constants
GSD_ET          = 15;                       % Ground sampling distance (m/pixel) 
imgWidth_ET     = 720;                      % Along track image width (pixels)
imgHeight_ET    = 720;                      % Cross-track image height (pixels)
focalLength_ET  = 0.58;                     % Focal length (m)
pixelSize_ET    = 17.4e-6;                  % Pixel size (m)
%---

%==========================================================================
%% Initialise Sensors =====================================================

% Star Tracker
n_ST        = 4;                        % Number of measurements
dt_ST       = 1;                        % Star tracker sampling rate (s)
noise_ST    = 0.1;                      % Star tracker noise (deg)
R_ST        = noise_ST*eye(n_ST);       % Star tracker noise matrix

z_ST        = zeros(n_ST,n_s);          % Star tracker measurements
y_ST        = zeros(n_ST,n_s);          % Star tracker estimated measurements
K_ST        = zeros(n_x,n_ST,n_s);      % Star tracker Kalman Gain
%---

% Magnetometer
n_MAG       = 4;                        % Number of measurements
dt_MAG      = 1;                        % Magnetometer sampling rate (s)
noise_MAG   = 3;                        % Magnetometer noise (deg)
R_MAG       = noise_MAG*eye(4);         % Magnetometer noise matrix

z_MAG       = zeros(n_MAG,n_s);         % Magnetometer measurements
y_MAG       = zeros(n_MAG,n_s);         % Magnetometer estimated measurements
K_MAG       = zeros(n_x,n_MAG,n_s);     % Magnetometer Kalman Gain
%---

% Coarse Sun Sensor
n_CSS       = 4;                        % Number of measurements
dt_CSS      = 1;                        % Coarse sun sensor sampling rate (s)
noise_CSS   = 5;                        % Coarse sun sensor noise (deg)
R_CSS       = noise_CSS*eye(4);         % Coarse sun sensor noise matix

z_CSS       = zeros(n_CSS,n_s);         % Coarse sun sensor measurement
y_CSS       = zeros(n_CSS,n_s);         % Coarse sun sensor estimated measurement
K_CSS       = zeros(n_x,n_CSS,n_s);     % Coarse sun sensor Kalman Gain
%---

% Gyroscope
n_GYR       = 3;                        % Number of measurements
dt_GYR      = 1;                        % Gyroscope sampling samping rate (s)
noise_GYR   = 0.1;                      % Gyroscope sensor noise (deg)
R_GYR       = deg2rad(noise_GYR*eye(3));% Gyroscope sensor noise matrix
driftRate_GYR = 1;                      % Gyroscope drift rate (deg/s)

drift_GYR   = zeros(n_GYR,1);           % Gyroscope drift buffer
z_GYR       = zeros(n_GYR,n_s);         % Gyroscope measurement
y_GYR       = zeros(n_GYR,n_s);         % Gyroscope estimated measurement
K_GYR       = zeros(n_x,n_GYR,n_s);     % Gyroscope Kalman Gain
%---

% GPS
n_GPS       = 3;                        % Number of measurements
dt_GPS      = 1;                        % GPS sampling samping rate (s)
noise_GPS   = 0.1;                      % GPS sensor noise (km)
R_GPS       = noise_GPS*eye(3);         % GPS noise matrix
driftRate_GPS   = 0.01;                 % GPS drift rate (km)

drift_GPS   = zeros(n_GPS,1);           % GPS drift buffer
z_GPS       = zeros(n_GPS,n_s);         % GPS measurement
y_GPS       = zeros(n_GPS,n_s);         % GPS estimated measurement
K_GPS       = zeros(n_x,n_GPS,n_s);     % GPS Kalman Gain
%----

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
    % Variables
    t = r*dt_p;
    %---
    
    % Plant (no changes needed - already uses proper B/I dynamics)
    x_true(:,r+1) = Plant(x_true(:,r), dt_p, I_p, Mu_p, Re_p, J2_p);
    %---
    
    % Image generator (uses current attitude from state vector)
    satelliteImage = GenerateSatelliteImage(ax, x_true(1:3,r), x_true(4:6,r), x_true(7:10,r), imgWidth_cam, imgHeight_cam, focalLength_cam, pixelSize_cam);
    % SaveSatelliteImages(satelliteImage,r);
    %---
    
    % Feature geo-detection
    catalogue_geo(:,:,r) = FeatureGeoDetection(satelliteImage, x_true(1:3,r), x_true(7:10,r), focalLength_cam, pixelSize_cam, n_f);
    catalogue_eci(:,:,r) = Geo2ECI(catalogue_geo(:,:,r),we_p, t);
    %---
    
    % Earth Tracker Measurement
    [f_m(:,:,r), grayImage, feature_pixels] = FeaturePixelDetection(satelliteImage, n_f);
    % SaveFeatureImages(grayImage, feature_pixels, r);
    z_ET(:,:,r) = EarthTracker(f_m(:,:,r),imgWidth_ET,imgHeight_ET,focalLength_ET, pixelSize_ET, GSD_ET);
    %---

    % EKF:
    [y_ET(:,:,r), x_EKF(:,r+1), P_EKF(:,:,r+1), K_ET(:,:,:,r)] = EKF( ...
        catalogue_eci(:,:,r),x_EKF(:,r),P_EKF(:,:,r),I_f,Q_f,dt_p,Mu_f, ...
        z_ET(:,:,r), z_CSS(:,r), z_MAG(:,r), z_ST(:,r), z_GPS(:,r), z_GYR(:,r), ...
        R_ET, R_CSS, R_MAG, R_ST, R_GPS, R_GYR);
    % ---

    % Progress bar
    elapsedTime = toc(startTime);
    progress = r / (n_s-1);
    estTotalTime = elapsedTime / progress;
    estRemaining = estTotalTime - elapsedTime;
    d.Value = progress;
    d.Message = sprintf('Elapsed: %.2fs | %d%% | Est. remaining: %.2fs | Sample: %d | Footage Time: %.2fs', ...
        elapsedTime, round(progress*100), estRemaining,r+1,t);
    %---
end
%--

%==========================================================================
%% Clean Processes ========================================================
delete('temp_image.png')
%==========================================================================