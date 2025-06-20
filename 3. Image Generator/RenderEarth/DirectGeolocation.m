function featureGeoLocations = DirectGeolocation(featurePixelCoords, satelliteImage, referenceRaster)
%========================= STEP 1: Load Reference Image =========================
[refImg, R] = readgeoraster(referenceRaster);
% Remove alpha channel if present
if size(refImg, 3) == 4
    refImg = refImg(:, :, 1:3);
end
refImg = im2double(refImg);
refGray = rgb2gray(refImg); % Convert to grayscale
[refRows, refCols] = size(refGray); % Get dimensions

%========================= STEP 2: Prepare Satellite Image ==================
% Convert satellite image to grayscale for feature matching
satGray = rgb2gray(satelliteImage);
[satRows, satCols] = size(satGray);

fprintf('Reference image size: %dx%d\n', refRows, refCols);
fprintf('Satellite image size: %dx%d\n', satRows, satCols);

%========================= STEP 3: Validate Features ========================
if size(featurePixelCoords, 1) ~= 2
    error('featurePixelCoords must be [2×n] matrix with [x; y] coordinates');
end
numFeatures = size(featurePixelCoords, 2);

%========================= STEP 4: Detect Reference Features =================
fprintf('Detecting SURF features in reference image...\n');
refPoints = detectSURFFeatures(refGray, 'MetricThreshold', 300); % Lower threshold for more features
[refDesc, refValidPoints] = extractFeatures(refGray, refPoints);
fprintf('Reference image: %d SURF features detected\n', refValidPoints.Count);

if refValidPoints.Count < 10
    warning('Very few features in reference image. Consider adjusting SURF parameters.');
end

%========================= STEP 5: Match Each Satellite Feature Patch ========
patchSize = 51; % Larger patch for better matching
halfPatch = floor(patchSize / 2);
featureGeoLocations = NaN(3, numFeatures); % Initialize with NaN for all features

% Matching parameters to try (progressively more permissive)
maxRatios = [0.6, 0.7, 0.8];
metricThresholds = [300, 500, 800];

fprintf('Processing %d satellite features...\n', numFeatures);

for i = 1:numFeatures
    x = round(featurePixelCoords(1, i));
    y = round(featurePixelCoords(2, i));
    
    fprintf('Feature %d at (%d, %d): ', i, x, y);
    
    % Skip if patch is out of bounds in SATELLITE image
    if x - halfPatch < 1 || x + halfPatch > satCols || ...
       y - halfPatch < 1 || y + halfPatch > satRows
        fprintf('Out of bounds in satellite image\n');
        continue;
    end
    
    % Extract patch around feature from SATELLITE image
    satPatch = satGray(y - halfPatch:y + halfPatch, x - halfPatch:x + halfPatch);
    
    % Try multiple approaches for better matching success
    matched = false;
    
    %=== Approach 1: SURF Feature Matching ===
    for threshIdx = 1:length(metricThresholds)
        if matched, break; end
        
        % Detect SURF features in satellite patch with varying thresholds
        patchPoints = detectSURFFeatures(satPatch, 'MetricThreshold', metricThresholds(threshIdx));
        
        if patchPoints.Count < 1
            continue;
        end
        
        [patchDesc, patchValidPoints] = extractFeatures(satPatch, patchPoints);
        if isempty(patchDesc)
            continue;
        end
        
        % Try different matching thresholds
        for ratioIdx = 1:length(maxRatios)
            indexPairs = matchFeatures(patchDesc, refDesc, ...
                                     'MaxRatio', maxRatios(ratioIdx), ...
                                     'MatchThreshold', 50);
            
            if ~isempty(indexPairs)
                % Get the best match from reference image
                bestIdx = indexPairs(1, 2);
                matchedPoint = refValidPoints(bestIdx);
                
                % Convert reference image coordinates to geographic coordinates
                refX = matchedPoint.Location(1);
                refY = matchedPoint.Location(2);
                
                % Check if match is within reference image bounds
                if refX >= 1 && refX <= refCols && refY >= 1 && refY <= refRows
                    lon = R.LongitudeLimits(1) + (refX - 1) * ...
                          (R.LongitudeLimits(2) - R.LongitudeLimits(1)) / (refCols - 1);
                    lat = R.LatitudeLimits(2) - (refY - 1) * ...
                          (R.LatitudeLimits(2) - R.LatitudeLimits(1)) / (refRows - 1);
                    
                    % Store result
                    featureGeoLocations(:, i) = [lat; lon; 0];
                    fprintf('SURF matched (threshold=%.1f, ratio=%.1f)\n', metricThresholds(threshIdx), maxRatios(ratioIdx));
                    matched = true;
                    break;
                end
            end
        end
    end
    
    %=== Approach 2: Template Matching (if SURF fails) ===
    if ~matched
        % Use normalized cross-correlation as fallback
        try
            % Enhance contrast for better template matching
            satPatchEnhanced = adapthisteq(satPatch);
            refGrayEnhanced = adapthisteq(refGray);
            
            correlation = normxcorr2(satPatchEnhanced, refGrayEnhanced);
            
            % Find peak correlation
            [maxCorr, maxIdx] = max(correlation(:));
            [peakY, peakX] = ind2sub(size(correlation), maxIdx);
            
            % Convert correlation coordinates to image coordinates
            refX = peakX - halfPatch;
            refY = peakY - halfPatch;
            
            % Quality threshold for template matching
            if maxCorr > 0.4 && refX >= 1 && refX <= refCols && refY >= 1 && refY <= refRows
                lon = R.LongitudeLimits(1) + (refX - 1) * ...
                      (R.LongitudeLimits(2) - R.LongitudeLimits(1)) / (refCols - 1);
                lat = R.LatitudeLimits(2) - (refY - 1) * ...
                      (R.LatitudeLimits(2) - R.LatitudeLimits(1)) / (refRows - 1);
                
                featureGeoLocations(:, i) = [lat; lon; 0];
                fprintf('Template matched (corr=%.3f)\n', maxCorr);
                matched = true;
            end
        catch
            % Template matching failed
        end
    end
    
    if ~matched
        fprintf('No match found\n');
    end
end

numMatched = sum(~isnan(featureGeoLocations(1,:)));
successRate = numMatched / numFeatures * 100;

fprintf('\n=== MATCHING RESULTS ===\n');
fprintf('Successfully geolocated %d out of %d features (%.1f%%)\n', numMatched, numFeatures, successRate);

if numMatched == 0
    fprintf('\n=== TROUBLESHOOTING SUGGESTIONS ===\n');
    fprintf('1. Check if satellite and reference images overlap geographically\n');
    fprintf('2. Verify image quality and contrast\n');
    fprintf('3. Consider using different reference image or larger patches\n');
    fprintf('4. Check feature detection parameters\n');
end

% Optional: Save debug images
if numMatched < numFeatures * 0.3 % If success rate < 30%
    saveDebugImages(satelliteImage, refImg, featurePixelCoords, featureGeoLocations);
end

end

%==========================================================================
function saveDebugImages(satImg, refImg, pixelCoords, geoLocs)
    try
        figure('Visible', 'off');
        
        subplot(1,2,1);
        imshow(satImg);
        hold on;
        plot(pixelCoords(1,:), pixelCoords(2,:), 'r+', 'MarkerSize', 10, 'LineWidth', 2);
        title('Satellite Image Features');
        
        subplot(1,2,2);
        imshow(refImg);
        title('Reference Image');
        
        saveas(gcf, 'debug_feature_matching.png');
        close(gcf);
        
        fprintf('Debug image saved as: debug_feature_matching.png\n');
    catch
        % Ignore save errors
    end
end