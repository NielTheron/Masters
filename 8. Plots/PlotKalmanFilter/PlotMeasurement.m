function PlotMeasurement(z_ET, n_f, dt)
%==========================================================================
% PlotMeasurement - Enhanced plotting function for Earth Tracker measurements
%==========================================================================
% INPUT:
% z_ET      : Earth tracker measurements (3 x n_f x n_samples)
% n_f       : Number of features
% dt        : Sample time interval (s)
%
% The function creates multiple visualization options for the 3D measurement data
%==========================================================================

    % Input validation
    if nargin < 3
        dt = 1; % Default sample time
    end
    
    if nargin < 2
        n_f = size(z_ET, 2); % Use all features
    end
    
    % Get dimensions
    [n_meas, n_features, n_samples] = size(z_ET);
    
    if n_meas ~= 3
        error('First dimension must be 3 (for 3D measurements)');
    end
    
    % Limit features if requested
    n_f = min(n_f, n_features);
    
    % Create time vector
    time_vec = (0:n_samples-1) * dt;
    
    % Create main figure with subplots
    fig = figure('Name', 'Earth Tracker Measurements (z_ET)', 'Position', [100 100 1200 800]);
    
    %% Plot 1: All measurements by component
    subplot(2,3,1)
    hold on
    colors = lines(n_f); % Generate distinct colors for each feature
    
    for i = 1:n_f
        plot(time_vec, squeeze(z_ET(1,i,:)), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Feature %d', i));
    end
    
    title('X-Component Measurements')
    xlabel('Time (s)')
    ylabel('Distance (km)')
    grid on
    if n_f <= 10  % Only show legend if not too many features
        legend('Location', 'best')
    end
    
    subplot(2,3,2)
    hold on
    for i = 1:n_f
        plot(time_vec, squeeze(z_ET(2,i,:)), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Feature %d', i));
    end
    
    title('Y-Component Measurements')
    xlabel('Time (s)')
    ylabel('Distance (km)')
    grid on
    if n_f <= 10
        legend('Location', 'best')
    end
    
    subplot(2,3,3)
    hold on
    for i = 1:n_f
        plot(time_vec, squeeze(z_ET(3,i,:)), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Feature %d', i));
    end
    
    title('Z-Component Measurements')
    xlabel('Time (s)')
    ylabel('Distance (km)')
    grid on
    if n_f <= 10
        legend('Location', 'best')
    end
    
    %% Plot 2: 3D trajectory for selected features
    subplot(2,3,4)
    hold on
    
    % Plot first few features in 3D
    n_plot_3d = min(5, n_f); % Plot max 5 features for clarity
    
    for i = 1:n_plot_3d
        x_data = squeeze(z_ET(1,i,:));
        y_data = squeeze(z_ET(2,i,:));
        z_data = squeeze(z_ET(3,i,:));
        
        plot3(x_data, y_data, z_data, 'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', sprintf('Feature %d', i));
        
        % Mark start and end points
        plot3(x_data(1), y_data(1), z_data(1), 'o', 'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
        plot3(x_data(end), y_data(end), z_data(end), 's', 'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
    end
    
    title('3D Measurement Trajectories')
    xlabel('X Distance (km)')
    ylabel('Y Distance (km)')
    zlabel('Z Distance (km)')
    grid on
    axis equal
    legend('Location', 'best')
    view(45, 30)
    
    %% Plot 3: Magnitude of measurements
    subplot(2,3,5)
    hold on
    
    for i = 1:n_f
        magnitude = sqrt(squeeze(z_ET(1,i,:)).^2 + squeeze(z_ET(2,i,:)).^2 + squeeze(z_ET(3,i,:)).^2);
        plot(time_vec, magnitude, 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Feature %d', i));
    end
    
    title('Measurement Magnitudes')
    xlabel('Time (s)')
    ylabel('Distance Magnitude (km)')
    grid on
    if n_f <= 10
        legend('Location', 'best')
    end
    
    %% Plot 4: Statistical summary
    subplot(2,3,6)
    
    % Calculate statistics for each feature
    feature_means = zeros(n_f, 3);
    feature_stds = zeros(n_f, 3);
    
    for i = 1:n_f
        feature_means(i, 1) = mean(squeeze(z_ET(1,i,:)));
        feature_means(i, 2) = mean(squeeze(z_ET(2,i,:)));
        feature_means(i, 3) = mean(squeeze(z_ET(3,i,:)));
        
        feature_stds(i, 1) = std(squeeze(z_ET(1,i,:)));
        feature_stds(i, 2) = std(squeeze(z_ET(2,i,:)));
        feature_stds(i, 3) = std(squeeze(z_ET(3,i,:)));
    end
    
    % Create bar plot of mean values
    bar_data = [feature_means(:,1), feature_means(:,2), feature_means(:,3)];
    bar(1:n_f, bar_data);
    
    title('Mean Measurements by Feature')
    xlabel('Feature Number')
    ylabel('Mean Distance (km)')
    legend('X-component', 'Y-component', 'Z-component', 'Location', 'best')
    grid on
    
    % Add overall title
    sgtitle(sprintf('Earth Tracker Measurements Analysis\n%d Features, %d Samples, dt=%.2fs', n_f, n_samples, dt))
    
end

