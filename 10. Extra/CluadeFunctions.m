    % Add this after your EKF call in the simulation loop
% (after the line: [y_ET(:,:,r), x_EKF(:,r+1), P_EKF(:,:,r+1), K_ET(:,:,:,r)] = EKF(...))

% Calculate expected measurements using true state for all features
y_ET_true_current = zeros(n_ET, n_f);
for i = 1:n_f
    if norm(catalogue_eci(:,i,r)) > 0
        y_ET_true_current(:,i) = HFunction(x_true(:,r), catalogue_eci(:,i,r));
    end
end

% Calculate measurement errors for current timestep
ET_errors_current = zeros(n_ET, n_f);
ET_error_norms_current = zeros(1, n_f);

for i = 1:n_f
    if norm(z_ET(:,i,r)) > 0 && norm(y_ET_true_current(:,i)) > 0
        ET_errors_current(:,i) = z_ET(:,i,r) - y_ET_true_current(:,i);
        ET_error_norms_current(i) = norm(ET_errors_current(:,i));
    end
end

% Store current errors in arrays (initialize these in your setup section)
if r == 1
    ET_error_norms_all = zeros(n_f, n_s);  % Store all error norms
    ET_rms_per_timestep = zeros(1, n_s);   % Store RMS per timestep
end

ET_error_norms_all(:, r) = ET_error_norms_current;

% Calculate RMS error for this timestep
valid_errors = ET_error_norms_current(ET_error_norms_current > 0);
if ~isempty(valid_errors)
    ET_rms_per_timestep(r) = sqrt(mean(valid_errors.^2));
    
    % Print diagnostics every 10 samples
    if mod(r, 10) == 0
        fprintf('Sample %d: ET RMS Error = %.4f km, Valid Features = %d/%d\n', ...
                r, ET_rms_per_timestep(r), length(valid_errors), n_f);
    end
end

%% Debug Earth Tracker Errors
% Add this right after your EKF call to diagnose the issues

if r == 1
    debug_data = struct();
    debug_data.z_ET_samples = [];
    debug_data.y_true_samples = [];
    debug_data.pixel_samples = [];
    debug_data.geo_samples = [];
end

% Sample first few features for detailed analysis
for i = 1:min(3, n_f)  % Analyze first 3 features
    if norm(z_ET(:,i,r)) > 0 && norm(catalogue_eci(:,i,r)) > 0
        
        % Store measurements for analysis
        debug_data.z_ET_samples = [debug_data.z_ET_samples, z_ET(:,i,r)];
        debug_data.y_true_samples = [debug_data.y_true_samples, y_ET_true_current(:,i)];
        debug_data.pixel_samples = [debug_data.pixel_samples, featurePixelLocations(:,i,r)];
        debug_data.geo_samples = [debug_data.geo_samples, catalogue_geo(:,i,r)];
        
        % Print detailed comparison for first timestep
        if r == 1
            fprintf('\n=== FEATURE %d ANALYSIS ===\n', i);
            fprintf('Pixel location: [%.1f, %.1f]\n', featurePixelLocations(1,i,r), featurePixelLocations(2,i,r));
            fprintf('Geo coordinates: [%.6f°, %.6f°]\n', catalogue_geo(1,i,r), catalogue_geo(2,i,r));
            fprintf('ECI coordinates: [%.3f, %.3f, %.3f] km\n', catalogue_eci(1,i,r), catalogue_eci(2,i,r), catalogue_eci(3,i,r));
            fprintf('z_ET measurement: [%.6f, %.6f, %.6f] km\n', z_ET(1,i,r), z_ET(2,i,r), z_ET(3,i,r));
            fprintf('Expected (HFunc): [%.6f, %.6f, %.6f] km\n', y_ET_true_current(1,i), y_ET_true_current(2,i), y_ET_true_current(3,i));
            fprintf('Error: [%.6f, %.6f, %.6f] km\n', z_ET(1,i,r)-y_ET_true_current(1,i), z_ET(2,i,r)-y_ET_true_current(2,i), z_ET(3,i,r)-y_ET_true_current(3,i));
            fprintf('Error magnitude: %.3f km = %.0f m\n', norm(z_ET(:,i,r)-y_ET_true_current(:,i)), norm(z_ET(:,i,r)-y_ET_true_current(:,i))*1000);
            
            % Check camera parameter consistency
            fprintf('\n=== CAMERA PARAMETER CHECK ===\n');
            fprintf('Image gen focal length: %.3f m\n', focalLength_cam);
            fprintf('EarthTracker focal length: %.3f m\n', focalLength_ET);
            % fprintf('Match? %s\n', isequal(focalLength_cam, focalLength_ET) ? 'YES' : 'NO - FIX THIS!');
            
            % Check expected measurement magnitude
            sat_to_earth_dist = norm(x_true(1:3,r));
            fprintf('Satellite altitude: %.1f km\n', sat_to_earth_dist - 6378);
            fprintf('Expected measurement magnitude: ~%.3f km\n', sat_to_earth_dist);
            fprintf('Actual z_ET magnitude: %.3f km\n', norm(z_ET(:,i,r)));
            fprintf('HFunction magnitude: %.3f km\n', norm(y_ET_true_current(:,i)));
        end
    end
end

% Check for systematic errors every 10 samples
if mod(r, 10) == 0 && size(debug_data.z_ET_samples, 2) > 5
    fprintf('\n=== SYSTEMATIC ERROR CHECK (Sample %d) ===\n', r);
    
    % Check for bias
    recent_errors = debug_data.z_ET_samples(:,end-4:end) - debug_data.y_true_samples(:,end-4:end);
    mean_bias = mean(recent_errors, 2);
    fprintf('Mean bias [x,y,z]: [%.6f, %.6f, %.6f] km\n', mean_bias(1), mean_bias(2), mean_bias(3));
    
    % Check magnitude ratios
    z_mags = sqrt(sum(debug_data.z_ET_samples.^2, 1));
    y_mags = sqrt(sum(debug_data.y_true_samples.^2, 1));
    mag_ratio = mean(z_mags ./ y_mags);
    fprintf('Average magnitude ratio (measured/expected): %.3f\n', mag_ratio);
    
    if abs(mag_ratio - 1) > 0.1
        fprintf('*** WARNING: Large magnitude difference suggests unit/scaling issue! ***\n');
    end
end

%% Earth Tracker Performance Analysis
if exist('ET_error_norms_all', 'var')
    
    % Extract all valid errors from the 3D structure
    valid_errors_all = ET_error_norms_all(ET_error_norms_all > 0);
    valid_rms_values = ET_rms_per_timestep(ET_rms_per_timestep > 0);
    
    if ~isempty(valid_errors_all)
        % Calculate statistics
        ET_mean_error = mean(valid_errors_all) * 1000;  % Convert to meters
        ET_std_error = std(valid_errors_all) * 1000;
        ET_max_error = max(valid_errors_all) * 1000;
        ET_rms_error = sqrt(mean(valid_errors_all.^2)) * 1000;
        ET_median_error = median(valid_errors_all) * 1000;
        
        fprintf('\n=== EARTH TRACKER PERFORMANCE ===\n');
        fprintf('Mean Error:     %.2f m\n', ET_mean_error);
        fprintf('Median Error:   %.2f m\n', ET_median_error);
        fprintf('Std Error:      %.2f m\n', ET_std_error);
        fprintf('RMS Error:      %.2f m\n', ET_rms_error);
        fprintf('Max Error:      %.2f m\n', ET_max_error);
        fprintf('Total Measurements: %d\n', length(valid_errors_all));
        fprintf('Success Rate: %.1f%% (valid features)\n', ...
                100 * length(valid_errors_all) / (n_f * length(valid_rms_values)));
        
        % Compare with other sensors
        fprintf('\n=== SENSOR ACCURACY COMPARISON ===\n');
        fprintf('Star Tracker:   ~30 m equivalent (0.1° at 500km)\n');
        fprintf('GPS:           ~100 m (0.1 km)\n');
        fprintf('Earth Tracker: ~%.0f m (your measurement)\n', ET_rms_error);
        fprintf('Magnetometer:  ~260 m equivalent (3° at 500km)\n');
        fprintf('Coarse Sun:    ~430 m equivalent (5° at 500km)\n');
        
        % Recommended update order based on accuracy (least to most accurate)
        if ET_rms_error < 50
            fprintf('\n*** RECOMMENDATION: Place Earth Tracker position 5 (high accuracy) ***\n');
            fprintf('Order: CSS → MAG → GYR → GPS → Earth Tracker → Star Tracker\n');
        elseif ET_rms_error < 150
            fprintf('\n*** RECOMMENDATION: Place Earth Tracker position 4 (medium-high accuracy) ***\n');
            fprintf('Order: CSS → MAG → Earth Tracker → GYR → GPS → Star Tracker\n');
        elseif ET_rms_error < 300
            fprintf('\n*** RECOMMENDATION: Place Earth Tracker position 3 (medium accuracy) ***\n');
            fprintf('Order: CSS → MAG → Earth Tracker → GYR → GPS → Star Tracker\n');
        else
            fprintf('\n*** RECOMMENDATION: Place Earth Tracker position 2 (lower accuracy) ***\n');
            fprintf('Order: CSS → Earth Tracker → MAG → GYR → GPS → Star Tracker\n');
        end
        
        % Create analysis plots
        figure('Name', 'Earth Tracker Performance Analysis');
        
        subplot(2,2,1);
        plot(valid_rms_values * 1000, 'b-', 'LineWidth', 1.5);
        xlabel('Time Step');
        ylabel('RMS Error (m)');
        title('Earth Tracker RMS Error Over Time');
        grid on;
        
        subplot(2,2,2);
        histogram(valid_errors_all * 1000, 30, 'FaceColor', 'blue');
        xlabel('Error (m)');
        ylabel('Frequency');
        title('Error Distribution (All Features)');
        grid on;
        
        subplot(2,2,3);
        % Feature success rate over time
        success_rate = sum(ET_error_norms_all > 0, 1) / n_f * 100;
        plot(success_rate, 'r-', 'LineWidth', 1.5);
        xlabel('Time Step');
        ylabel('Success Rate (%)');
        title('Feature Detection Success Rate');
        grid on;
        ylim([0 100]);
        
        subplot(2,2,4);
        % Average errors per feature
        feature_avg_errors = mean(ET_error_norms_all, 2) * 1000; % Average error per feature
        feature_avg_errors = feature_avg_errors(feature_avg_errors > 0); % Remove zero entries
        if ~isempty(feature_avg_errors)
            bar(feature_avg_errors);
            xlabel('Feature Index');
            ylabel('Average Error (m)');
            title('Average Error per Feature');
            grid on;
        end
        
    else
        fprintf('No valid Earth Tracker measurements found.\n');
    end
else
    fprintf('No Earth Tracker error data available for analysis.\n');
    fprintf('Make sure the diagnostic code ran during simulation.\n');
end