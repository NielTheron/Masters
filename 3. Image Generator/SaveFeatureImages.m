function SaveFeatureImages(grayImage,locations,r)
    rgbImage = repmat(grayImage, [1, 1, 3]);
    radii = 10;  % Radius of filled circle
    locations = locations';  % Now it's 13 x 2
    numPoints = size(locations, 1);
    circles = [locations, repmat(radii, numPoints, 1)];
    feature_image = insertShape(rgbImage, 'FilledCircle', circles,'Color', 'red', 'Opacity', 1);
    filename = sprintf('feature_image_%03d.png', r);  % Zero-padded filename like sat_image_001.mat
    % outputFolder = 'C:\Users\Niel\OneDrive - Stellenbosch University\Desktop\Masters\3. Image Generator\FeatureImages';
    outputFolder = 'C:\Users\22619291\OneDrive - Stellenbosch University\Desktop\Masters\3. Image Generator\FeatureImages';
    fullpath = fullfile(outputFolder,filename);
    imwrite(feature_image,fullpath);
end

