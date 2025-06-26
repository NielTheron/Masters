function [feature_pixel_locations, grayImage] = FeaturePixelDetection(image,n_f)
grayImage = rgb2gray(image);
points = detectSIFTFeatures(grayImage);
feature_pixels = points.selectStrongest(n_f);
feature_pixel_locations = feature_pixels.Location.';

if size(feature_pixel_locations,2) == n_f
    feature_pixel_locations = feature_pixels.Location.';
else
    feature_pixel_locations = zeros(2,n_f);
end


end

