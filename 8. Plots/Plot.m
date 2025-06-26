%==========================================================================
% Niel Theron
% 26-03-2025
%==========================================================================
% The purpose of this script is to plot all the graphs
%==========================================================================

%% === Plots ==============================================================

% Plot System States
%---------------------
PlotState(x_true,dt_p)
% PlotEstimatedState(x_EKF,dt_p)
% PlotStateError(x_true,x_EKF,dt_p)
%---------------------

% Plot Estimator
%---------------------
% PlotCovariance(x_true,x_EKF,P_EKF,n_s)

%---------------------



% Plot Earth Tracker
%-------------------
% PlotMeasurement(z_ET,n_f,dt_p)
% PlotEstimatedMeasurement(y_ET,n_f,dt_p)
% PlotMeasurementError(z_ET,y_ET,n_f,dt_p)
%-------------------

% Plot Sensors
%-------------------
% PlotGyro(x_true,dt_p,z_GYR)
% PlotGPS(x_true,dt_p,z_GPS)
PlotST(x_true,dt_p,z_ST)
PlotCSS(x_true,dt_p,z_CSS)
PlotMag(x_true,dt_p,z_MAG)
PlotTRIAD(x_true,dt_p,z_TRIAD)
%-------------------

% Plot Images
%------------------
% PlotSatelliteImage(satellite_image)
% PlotFeatureDetectedImage(grayImage,feature_pixels)
% MakeFeatureVideo(dt)
% MakeVideo(dt_p)
%------------------


% Plot Catalogues
%----------------
% PlotCatalogueGeo(catalogue_geo,10)
% PlotCatalogueECI(catalogue_eci,1)
%----------------


% Visual Plots
%--------------------
% Plot3D(x_true,x_EKF)

%---

%==========================================================================


