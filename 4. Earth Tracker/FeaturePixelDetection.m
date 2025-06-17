function [feature_pixel_locations, grayImage, feature_pixels] = FeaturePixelDetection(image,n_f)

grayImage = rgb2gray(image);
points = detectSURFFeatures(grayImage);
feature_pixels = points.selectStrongest(n_f);
feature_pixel_locations = feature_pixels.Location.';


end

