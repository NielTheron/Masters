function PlotGPS(x_true, dt, GPS_measurement)
    figure('Name', 'GPS Measurement')
    hold on
    n = (0:size(x_true,2)-1) * dt;

    % Colors and labels
    colors = {'b', 'g', 'r'};
    labels = {'x', 'y', 'z'};

    for i = 1:3
        % Filter out zero values
        valid_idx = GPS_measurement(i,:) ~= 0;
        n_valid = n(valid_idx);
        GPS_valid = GPS_measurement(i, valid_idx);

        % Scatter plot of GPS measurements
        scatter(n_valid, GPS_valid, 20, colors{i}, 'filled', ...
            'DisplayName', sprintf('GPS %s', labels{i}));
    end

    title("Abstract Position Measurement Sensors");
    xlabel("Time (s)");
    ylabel("Position (km)");
    legend('show');
    grid on
end
