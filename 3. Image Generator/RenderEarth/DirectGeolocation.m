%==========================================================================
% Niel Theron
% 19-06-2025
%==========================================================================
% Direct Geolocation using Georeferenced Paris Strip Image with Pre-Detected Features
% This function takes pre-detected feature pixel coordinates and maps them
% to geographic coordinates using the ParisStrip.tif georeferenced raster
%==========================================================================
% INPUT:
% featurePixelCoords    : Pre-detected feature pixel coordinates [2×n] = [x; y]
% satelliteImage        : The captured satellite image (RGB) - for visualization
% referenceRaster       : Path to georeferenced image ('ParisStrip.tif')
% matchingMethod        : 'template' or 'descriptor' (default: 'template')
% 
% OUTPUT:
% featureGeoLocations   : Geographic coordinates [lat; lon] in degrees (2×n)
% matchedFeatureIdx     : Indices of successfully matched features
% matchQuality          : Quality metrics for each match
% referenceImage        : The loaded reference image for visualization
%
% VARIABLES:
% R                     : Spatial reference object from geotiff
% refImg                : Reference georeferenced image
% satGray              : Grayscale satellite image
% refGray              : Grayscale reference image
% featurePatches       : Image patches around each feature
% bestMatches          : Best match locations in reference image
% correlationScores    : Correlation scores for template matching
%==========================================================================

function [featureGeoLocations, matchedFeatureIdx, matchQuality, referenceImage] = ...
    DirectGeolocation(featurePixelCoords, satelliteImage, referenceRaster, matchingMethod)

    % Default parameters
    if nargin < 4
        matchingMethod = 'template';
    end
    if nargin < 3
        referenceRaster = 'ParisStrip.tif';
    end

    %======================================================================
    % STEP 1: Load and prepare the georeferenced reference image
    %======================================================================
    try
        [refImg, R] = readgeoraster(referenceRaster);
        fprintf('Successfully loaded georeferenced image: %s\n', referenceRaster);
        fprintf('Geographic bounds: Lat [%.4f, %.4f], Lon [%.4f, %.4f]\n', ...
                R.LatitudeLimits(1), R.LatitudeLimits(2), ...
                R.LongitudeLimits(1), R.LongitudeLimits(2));
    catch ME
        error('Failed to load georeferenced image %s: %s', referenceRaster, ME.message);
    end
    
    % Handle 4-channel images (remove alpha)
    if size(refImg, 3) == 4
        refImg = refImg(:,:,1:3);
    end
    refImg = im2double(refImg);
    referenceImage = refImg;
    
    % Convert to grayscale for feature matching
    satGray = rgb2gray(satelliteImage);
    refGray = rgb2gray(refImg);
    
    % Get image dimensions
    [satHeight, satWidth] = size(satGray);
    [refRows, refCols] = size(refGray);
    
    %======================================================================
    % STEP 2: Validate input feature coordinates
    %======================================================================
    if size(featurePixelCoords, 1) ~= 2
        error('featurePixelCoords must be [2×n] matrix with [x; y] coordinates');
    end
    
    numFeatures = size(featurePixelCoords, 2);
    fprintf('Processing %d pre-detected features\n', numFeatures);
    
    % Check if features are within image bounds
    validFeatures = featurePixelCoords(1,:) >= 1 & featurePixelCoords(1,:) <= satWidth & ...
                   featurePixelCoords(2,:) >= 1 & featurePixelCoords(2,:) <= satHeight;
    
    if sum(validFeatures) < numFeatures
        warning('%d features are outside image bounds and will be skipped', ...
                numFeatures - sum(validFeatures));
    end
    
    %======================================================================
    % STEP 3: Extract feature patches and match with reference image
    %======================================================================
    patchSize = 31;  % Size of patch around each feature (must be odd)
    halfPatch = floor(patchSize/2);
    
    % Initialize output arrays
    featureGeoLocations = [];
    matchedFeatureIdx = [];
    matchQuality = [];
    
    fprintf('Matching features using %s method...\n', matchingMethod);
    
    for i = 1:numFeatures
        if ~validFeatures(i)
            continue;
        end
        
        x = round(featurePixelCoords(1, i));
        y = round(featurePixelCoords(2, i));
        
        % Check if patch fits within satellite image
        if x-halfPatch < 1 || x+halfPatch > satWidth || ...
           y-halfPatch < 1 || y+halfPatch > satHeight
            continue;
        end
        
        % Extract feature patch from satellite image
        featurePatch = satGray(y-halfPatch:y+halfPatch, x-halfPatch:x+halfPatch);
        
        %==================================================================
        % STEP 4: Find best match in reference image
        %==================================================================
        if strcmp(matchingMethod, 'template')
            % Template matching using normalized cross-correlation
            correlation = normxcorr2(featurePatch, refGray);
            
            % Find peak correlation
            [maxCorr, maxIdx] = max(correlation(:));
            [peakY, peakX] = ind2sub(size(correlation), maxIdx);
            
            % Convert correlation coordinates to image coordinates
            refX = peakX - halfPatch;
            refY = peakY - halfPatch;
            
            % Quality metric is the correlation coefficient
            quality = maxCorr;
            
        elseif strcmp(matchingMethod, 'descriptor')
            % SURF descriptor matching (more robust but slower)
            try
                % Detect SURF features in both patches
                satPoints = detectSURFFeatures(featurePatch);
                if satPoints.Count < 1
                    continue;
                end
                
                % Extract descriptors
                [satDesc, ~] = extractFeatures(featurePatch, satPoints);
                
                % Match with reference image features
                refPoints = detectSURFFeatures(refGray);
                [refDesc, refValidPoints] = extractFeatures(refGray, refPoints);
                
                % Find matches
                indexPairs = matchFeatures(satDesc, refDesc, 'MaxRatio', 0.7);
                
                if size(indexPairs, 1) < 1
                    continue;
                end
                
                % Use best match
                bestMatchIdx = indexPairs(1, 2);
                matchedRefPoint = refValidPoints(bestMatchIdx);
                
                refX = matchedRefPoint.Location(1);
                refY = matchedRefPoint.Location(2);
                quality = matchedRefPoint.Metric;
                
            catch
                continue;
            end
        end
        
        %==================================================================
        % STEP 5: Convert reference image coordinates to geographic coordinates
        %==================================================================
        % Apply quality threshold
        if quality < 0.3  % Adjust threshold as needed
            continue;
        end
        
        % Check if match is within reference image bounds
        if refX < 1 || refX > refCols || refY < 1 || refY > refRows
            continue;
        end
        
        % Convert pixel coordinates to geographic coordinates using spatial reference
        lon = R.LongitudeLimits(1) + (refX - 1) * ...
              (R.LongitudeLimits(2) - R.LongitudeLimits(1)) / (refCols - 1);
        
        lat = R.LatitudeLimits(2) - (refY - 1) * ...
              (R.LatitudeLimits(2) - R.LatitudeLimits(1)) / (refRows - 1);
        
        % Store results
        featureGeoLocations = [featureGeoLocations, [lat; lon]];
        matchedFeatureIdx = [matchedFeatureIdx, i];
        matchQuality = [matchQuality, quality];
    end
    
    %======================================================================
    % STEP 6: Display results summary
    %======================================================================
    numMatched = length(matchedFeatureIdx);
    fprintf('\n=== Direct Geolocation Results ===\n');
    fprintf('Input features: %d\n', numFeatures);
    fprintf('Successfully matched: %d (%.1f%%)\n', numMatched, 100*numMatched/numFeatures);
    
    if numMatched > 0
        fprintf('Latitude range: [%.6f, %.6f] degrees\n', ...
                min(featureGeoLocations(1,:)), max(featureGeoLocations(1,:)));
        fprintf('Longitude range: [%.6f, %.6f] degrees\n', ...
                min(featureGeoLocations(2,:)), max(featureGeoLocations(2,:)));
        fprintf('Average match quality: %.3f\n', mean(matchQuality));
    else
        warning('No features were successfully matched!');
    end
    
    %======================================================================
    % STEP 7: Optional visualization
    %======================================================================
    if nargout == 0 || true  % Always show visualization for debugging
        visualizeDirectMatches(satelliteImage, refImg, featurePixelCoords, ...
                              featureGeoLocations, matchedFeatureIdx, matchQuality);
    end
end

%==========================================================================
% Helper function for visualization
%==========================================================================
function visualizeDirectMatches(satImg, refImg, allFeatures, geoLocs, matchedIdx, quality)
    figure('Name', 'Direct Geolocation with Pre-detected Features', ...
           'Position', [100 100 1400 900]);
    
    % Plot 1: Satellite image with all detected features
    subplot(2,3,1);
    imshow(satImg);
    hold on;
    
    % Plot all features in blue
    plot(allFeatures(1,:), allFeatures(2,:), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
    
    % Highlight successfully matched features in red
    if ~isempty(matchedIdx)
        matchedFeatures = allFeatures(:, matchedIdx);
        plot(matchedFeatures(1,:), matchedFeatures(2,:), 'r+', 'MarkerSize', 10, 'LineWidth', 3);
    end
    
    title(sprintf('Satellite Image\n%d Total, %d Matched Features', ...
                  size(allFeatures,2), length(matchedIdx)));
    legend({'All Features', 'Matched Features'}, 'Location', 'best');
    
    % Plot 2: Reference image
    subplot(2,3,2);
    imshow(refImg);
    title('Reference Image (Paris Strip)');
    
    % Plot 3: Geographic distribution of matched features
    if ~isempty(geoLocs)
        subplot(2,3,3);
        scatter(geoLocs(2,:), geoLocs(1,:), 60, quality, 'filled');
        colorbar;
        colormap(gca, 'jet');
        xlabel('Longitude (degrees)');
        ylabel('Latitude (degrees)');
        title('Geographic Distribution\n(Color = Match Quality)');
        grid on;
        
        % Add feature numbers
        for i = 1:size(geoLocs, 2)
            text(geoLocs(2,i), geoLocs(1,i), sprintf('%d', matchedIdx(i)), ...
                 'FontSize', 8, 'Color', 'white', 'FontWeight', 'bold');
        end
    end
    
    % Plot 4: Match quality histogram
    if ~isempty(quality)
        subplot(2,3,4);
        histogram(quality, 10);
        xlabel('Match Quality');
        ylabel('Number of Features');
        title('Match Quality Distribution');
        grid on;
    end
    
    % Plot 5: Feature matching success rate
    subplot(2,3,5);
    successRate = length(matchedIdx) / size(allFeatures, 2) * 100;
    pie([length(matchedIdx), size(allFeatures,2) - length(matchedIdx)], ...
        {sprintf('Matched (%.1f%%)', successRate), ...
         sprintf('Unmatched (%.1f%%)', 100-successRate)});
    title('Feature Matching Success Rate');
    
    % Plot 6: Coordinate statistics
    if ~isempty(geoLocs)
        subplot(2,3,6);
        
        % Create text summary
        axis off;
        text(0, 0.9, 'Geolocation Statistics:', 'FontSize', 12, 'FontWeight', 'bold');
        text(0, 0.8, sprintf('Total Features: %d', size(allFeatures,2)), 'FontSize', 10);
        text(0, 0.7, sprintf('Matched Features: %d', length(matchedIdx)), 'FontSize', 10);
        text(0, 0.6, sprintf('Success Rate: %.1f%%', successRate), 'FontSize', 10);
        text(0, 0.5, sprintf('Lat Range: %.6f°', range(geoLocs(1,:))), 'FontSize', 10);
        text(0, 0.4, sprintf('Lon Range: %.6f°', range(geoLocs(2,:))), 'FontSize', 10);
        text(0, 0.3, sprintf('Avg Quality: %.3f', mean(quality)), 'FontSize', 10);
        text(0, 0.2, sprintf('Min Quality: %.3f', min(quality)), 'FontSize', 10);
        text(0, 0.1, sprintf('Max Quality: %.3f', max(quality)), 'FontSize', 10);
        
        xlim([0 1]);
        ylim([0 1]);
    end
end