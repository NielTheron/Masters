function f_metric = EarthTracker(pixel_coords, Ix, Iy, fl, ps, GSD)
%==========================================================================
% Earth Tracker: Feature Vector Processing Algorithm
% D.P. Theron - June 17, 2025
%==========================================================================
% PURPOSE: Converts detected feature pixel coordinates to metric direction
% vectors suitable for satellite pose estimation
%
% INPUTS:
% pixel_coords  - [N x 2] matrix of pixel coordinates [fMx, fMy] 
% camera_params - struct with fields:
%                 .width  (Ix) - image width in pixels
%                 .height (Iy) - image height in pixels  
%                 .focal_length (fl) - focal length in meters
%                 .pixel_size (ps) - pixel size in meters
% GSD           - Ground Sample Distance in meters/pixel
%
% OUTPUT:
% f_metric      - [N x 3] matrix of feature vectors in meters
%                 Each row represents one feature vector [x, y, z]
%
% ALGORITHM STEPS:
% Step 1: Extract pixel coordinates (already provided as input)
% Step 2: Translate to optical center  
% Step 3: Construct 3D ray vector
% Step 4: Transform ray direction
% Step 5: Convert to metric units
%==========================================================================


N_features = size(pixel_coords, 2);

% Step 1: Feature Pixel Coordinate Extraction
% Input: fM = [fMx, fMy] (already provided)
fMx = pixel_coords(1, :);
fMy = pixel_coords(2, :);

% Step 2: Coordinate Translation to Optical Center
% Transform to camera boresight coordinates
fM_S = zeros(2, N_features);
fM_S(1, :) = fMx - Ix/2;  % fMx - Ix/2
fM_S(2, :) = fMy - Iy/2;  % fMy - Iy/2

% Step 3: Three-Dimensional Ray Vector Construction
% Calculate focal length in pixel units
f_pixels = fl / ps;  % Convert focal length to pixels

% Construct 3D ray vectors
fM_F = zeros(3, N_features);
fM_F(1, :) = fM_S(1, :);     % fM/Sx
fM_F(2, :) = fM_S(2, :);     % fM/Sy  
fM_F(3, :) = -f_pixels;       % fl/ps (same for all features)

% Step 4: Ray Direction Transformation
% Invert direction to point from focal point to ground
fG_F = zeros(3, N_features);
fG_F(1, :) = -fM_F(1, :);    % -fM/Fx
fG_F(2, :) = -fM_F(2, :);    % -fM/Fy
fG_F(3, :) = -fM_F(3, :);    % -fM/Fz

% Step 5: Metric Scale Conversion
% Apply GSD scaling to convert from pixels to meters
f_metric = GSD * fG_F / 1000; %(km)

end