function PlotFeatureDetectedImage(grayImage,feature_pixels)
figure; 
imshow(grayImage); 
hold on; 
plot(feature_pixels);
end

