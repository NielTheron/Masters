function SaveSatelliteImages(satellite_image,r)
    filename = sprintf('sat_image_%03d.png', r);  % Zero-padded filename like sat_image_001.mat
    outputFolder = 'C:\Users\Niel\OneDrive - Stellenbosch University\Desktop\Masters\3. Image Generator\SatelliteImages';
    fullpath = fullfile(outputFolder,filename);
    imwrite(satellite_image,fullpath);
    % fprintf('Saving frame %d...\n', r);
end

