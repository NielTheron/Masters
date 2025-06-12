%==========================================================================
% Niel Theron
% 26-03-2025
%==========================================================================
% The purpose of this script is to plot all the graphs
%==========================================================================

%% Load path ==============================================================

% Load plots into path
addpath("8. Plots\PlotErrors")
addpath("8. Plots\PlotImages")
addpath("8. Plots\PlotKalmanFilter")
addpath("8. Plots\PlotPlant")
addpath("8. Plots\PlotSensors")
addpath("8. Plots\Videos")
%---

%==========================================================================
%% === Plots ==============================================================

% Plot varaibles
% PlotState(x_true,dt_p)
% PlotEstimatedState(x_EKF,dt_p)
% PlotStateError(x_true,x_EKF,dt_p)
PlotMeasurement(z_ET,n_f,dt_p)
PlotEstimatedMeasurement(y_ET,n_f,dt_p)
PlotMeasurementError(z_ET,y_ET,n_f,dt)
% Plot3D(x_true)
% PlotSensors(x_true,dt,ST_measurement,MAG_measurement,CSS_measurement)
% PlotGyro(x_true,dt,GYR_measurement)
% PlotGPS(x_true,dt,GPS_measurement)
% PlotSatelliteImage(satellite_image)
% PlotFeatureDetectedImage(grayImage,feature_pixels)
% MakeFeatureVideo(dt)
% MakeVideo(dt_p)
% PlotCatalogueGeo(catalogue_geo,10)
% PlotCatalogueECI(catalogue_eci,120)
%---

%==========================================================================
