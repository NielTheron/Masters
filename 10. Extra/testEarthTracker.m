function testEarthTracker()
%==========================================================================
% Test function for EarthTracker to identify issues
%==========================================================================

fprintf('=== EarthTracker Test Suite ===\n\n');

%% Test 1: Basic nadir-pointing satellite
fprintf('Test 1: Nadir-pointing satellite at 500 km altitude\n');

% Satellite position (500 km altitude above equator)
Re = 6378;  % km
altitude = 500;  % km
sat_pos_eci = [Re + altitude; 0; 0];  % Simple position

% Quaternion for nadir-pointing (camera looking down)
% Identity quaternion (no rotation from body to ECI)
sat_quat = [1; 0; 0; 0];  % [w, x, y, z] - no rotation

% Camera parameters (matching your simulation)
focal_length_m = 0.058;     % 58mm lens
pixel_size_m = 17.4e-6;     % 17.4 micron pixels
pixel_height = 720;
pixel_width = 720;

% Test features at image center and corners
feature = [
    pixel_width/2,  pixel_height/2;    % Center
    pixel_width/4,  pixel_height/4;    % Upper left quadrant
    3*pixel_width/4, pixel_height/4;   % Upper right quadrant
    pixel_width/4,  3*pixel_height/4;  % Lower left quadrant
    3*pixel_width/4, 3*pixel_height/4; % Lower right quadrant
];

fprintf('Input parameters:\n');
fprintf('  Satellite position: [%.1f, %.1f, %.1f] km\n', sat_pos_eci);
fprintf('  Satellite altitude: %.1f km\n', norm(sat_pos_eci) - Re);
fprintf('  Quaternion: [%.3f, %.3f, %.3f, %.3f]\n', sat_quat);

% Call EarthTracker
ET_measurement = EarthTracker(feature, sat_pos_eci, sat_quat, ...
                             focal_length_m, pixel_size_m, pixel_height, pixel_width);

fprintf('\nResults:\n');
for i = 1:size(feature, 1)
    fprintf('  Feature %d [%.0f, %.0f] -> [%.2f, %.2f, %.2f] km\n', ...
            i, feature(i,:), ET_measurement(:,i));
end

%% Test 2: Check coordinate system consistency
fprintf('\n\nTest 2: Coordinate system validation\n');

% For nadir pointing, Z-component should be negative (pointing down to Earth)
% and magnitude should be approximately equal to altitude
expected_magnitude = altitude;
for i = 1:size(ET_measurement, 2)
    actual_magnitude = norm(ET_measurement(:, i));
    if actual_magnitude > 0
        fprintf('  Feature %d: Expected ~%.0f km, got %.1f km', i, expected_magnitude, actual_magnitude);
        if ET_measurement(3, i) < 0
            fprintf(' (Z < 0 ✓)\n');
        else
            fprintf(' (Z >= 0 ✗)\n');
        end
    end
end

%% Test 3: Different satellite orientations
fprintf('\n\nTest 3: Testing different satellite orientations\n');

% Test with 180-degree roll (upside down)
quat_roll_180 = [0; 1; 0; 0];  % 180-degree roll about X-axis
fprintf('180-degree roll:\n');
ET_measurement_roll = EarthTracker(feature(1,:), sat_pos_eci, quat_roll_180, ...
                                  focal_length_m, pixel_size_m, pixel_height, pixel_width);
fprintf('  Center pixel -> [%.2f, %.2f, %.2f] km\n', ET_measurement_roll);

%% Test 4: Validate your actual simulation parameters
fprintf('\n\nTest 4: Using your simulation parameters\n');

% Parameters from your simulator
lat_p = 48.858715;  % deg
lon_p = 1.66;       % deg
alt_p = 500;        % km
roll_p = 178;       % deg
yaw_p = 0;          % deg
pitch_p = 0;        % deg

% Convert to ECI position (simplified)
lat_rad = deg2rad(lat_p);
lon_rad = deg2rad(lon_p);
r_mag = Re + alt_p;

sat_pos_sim = r_mag * [cos(lat_rad) * cos(lon_rad);
                       cos(lat_rad) * sin(lon_rad);
                       sin(lat_rad)];

% Convert Euler angles to quaternion (your format: YZX order)
quat_sim = eul2quat(deg2rad([roll_p, yaw_p, pitch_p]), "YZX");

fprintf('Simulation parameters:\n');
fprintf('  Position: [%.1f, %.1f, %.1f] km\n', sat_pos_sim);
fprintf('  Quaternion: [%.4f, %.4f, %.4f, %.4f]\n', quat_sim);

% Test with simulation parameters
test_feature = [pixel_width/2, pixel_height/2];  % Center pixel
ET_sim = EarthTracker(test_feature, sat_pos_sim, quat_sim, ...
                     focal_length_m, pixel_size_m, pixel_height, pixel_width);

fprintf('Simulation test result:\n');
fprintf('  Center pixel -> [%.2f, %.2f, %.2f] km\n', ET_sim);

%% Test 5: Camera parameter validation
fprintf('\n\nTest 5: Camera parameter validation\n');

fov_x = 2 * atand((pixel_width * pixel_size_m / 2) / focal_length_m);
fov_y = 2 * atand((pixel_height * pixel_size_m / 2) / focal_length_m);
gsd_nadir = (altitude * 1000) * pixel_size_m / focal_length_m;  % Ground sampling distance

fprintf('Camera specifications:\n');
fprintf('  Focal length: %.1f mm\n', focal_length_m * 1000);
fprintf('  Pixel size: %.1f μm\n', pixel_size_m * 1e6);
fprintf('  Image size: %d x %d pixels\n', pixel_width, pixel_height);
fprintf('  Field of view: %.1f° x %.1f°\n', fov_x, fov_y);
fprintf('  Ground sampling distance (nadir): %.1f m/pixel\n', gsd_nadir);

fprintf('\n=== Test Complete ===\n');

end

% Uncomment to run the test
% testEarthTracker();