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

addpath("5. Sensors\")

addpath("6. EKF\")
addpath("6. EKF\Prediction\")
addpath("6. EKF\Tune\")
addpath("6. EKF\Update\")

addpath("7. Transformations\")

addpath("8. Plots\")
addpath("8. Plots\Plot Catalogues\")
addpath("8. Plots\Plot Earth Tracker\")
addpath("8. Plots\Plot Estimator\")
addpath("8. Plots\Plot Sensors\")
addpath("8. Plots\Plot Images\")
addpath("8. Plots\Plot System States\")
addpath("8. Plots\Plot Visual\")
addpath("8. Plots\Videos")
end

%==========================================================================