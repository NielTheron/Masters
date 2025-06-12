%==========================================================================
% 10-06-2025
% Niel Theron
%==========================================================================
% The purpose of this function is to clear all the files in the image
% folders to make sure all images used in the analysis is the newest data
%==========================================================================

function CleanFolders()
    % Clear satellite Images
    folderPath = 'C:\Users\Niel\OneDrive - Stellenbosch University\Desktop\Masters\3. Image Generator\SatelliteImages'; % **IMPORTANT: Replace with your actual folder path**
    if exist(folderPath, 'dir')
        rmdir(folderPath, 's'); % 's' option removes the directory and all its contents
    end
    mkdir(folderPath);
    addpath(folderPath)
    %---

    % Clear feature Images
    folderPath = 'C:\Users\Niel\OneDrive - Stellenbosch University\Desktop\Masters\3. Image Generator\FeatureImages'; % **IMPORTANT: Replace with your actual folder path**
    if exist(folderPath, 'dir')
    rmdir(folderPath, 's'); % 's' option removes the directory and all its contents
    end
    mkdir(folderPath);
    addpath(folderPath)
    %---
end

