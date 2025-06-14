

image = imread("Paris.png");
n_f = 20;
sat_pos = [140, 14, 255];
sat_quat = [1,0,0,0];
focal_length_m = 0.58;
pixel_size_m = 17.4e-6;
pixel_width = 720;
pixel_height = 720;


[feature, grayImage, feature_pixels] = Feature_Pixel_Detection(image,n_f);
ET_measurement = EarthTracker(feature,sat_pos, sat_quat, focal_length_m, pixel_size_m, pixel_height, pixel_width);
PlotETResults(ET_measurement)