%==========================================================================
% Niel Theron
% 10-06-2025
%==========================================================================
% The purpose of this function is to load all the other functions
%==========================================================================

function LoadPath()
addpath("1. Simulator\")
addpath("1. Simulator\UnitDelay\")
addpath("2. Plant\")
addpath("3. Image Generator\")
addpath("3. Image Generator\RenderEarth")
addpath("3. Image Generator\SatelliteImages\")
addpath("3. Image Generator\FeatureImages\")
addpath("4. Earth Tracker\")
addpath("5. Abstract Attitude Sensor\")
addpath("6. Abstract Position Sensor\")
addpath("7. EKF\")
addpath("7. EKF\Prediction\")
addpath("7. EKF\Tune\")
addpath("7. EKF\Update\")
addpath("8. Transformations\")
addpath("9. Plots\")
addpath("9. Plots\PlotErrors")
addpath("9. Plots\PlotImages")
addpath("9. Plots\PlotKalmanFilter")
addpath("9. Plots\PlotPlant")
addpath("9. Plots\PlotSensors")
addpath("9. Plots\Videos")
end

%==========================================================================