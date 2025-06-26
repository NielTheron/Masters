function PlotGyro(x_true, dt, GYR_measurement)
    figure('Name', "Gyroscope")
    hold on
    n = (0:size(x_true,2)-1) * dt;

    % Colors and labels
    colors = {'b', 'g', 'r'};
    axisLabels = {'x', 'y', 'z'};

    GYR_measurement = rad2deg(GYR_measurement);

    for i = 1:3
        % Filter non-zero measurements
        valid_idx = GYR_measurement(i,:) ~= 0;
        n_valid = n(valid_idx);
        GYR_valid = GYR_measurement(i, valid_idx);

        % Scatter plot of gyroscope measurements
        scatter(n_valid, GYR_valid, 20, colors{i}, 'filled', ...
            'DisplayName', sprintf('ω_%s meas', axisLabels{i}));
    end

    title("Abstract Gyroscope Measurement")
    xlabel("Time (s)")
    ylabel("Angular Velocity (deg/s)")
    legend('show')
    grid on
end