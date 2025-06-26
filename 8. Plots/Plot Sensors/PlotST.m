function PlotST(x_true, dt, ST_measurement)
    figure('Name', "Sensor Comparison")
    hold on
    n = (0:size(x_true, 2)-1) * dt;

    % Colors for each quaternion component
    colors = {'b', 'g', 'r', 'm'};

    for i = 1:4
        % Filter non-zero measurements
        valid_idx = ST_measurement(i,:) ~= 0;
        n_valid = n(valid_idx);
        ST_valid = ST_measurement(i, valid_idx);

        % Scatter plot only (no lollipops)
        scatter(n_valid, ST_valid, 20, colors{i}, 'filled', ...
            'DisplayName', sprintf('ST q_%d meas', i-1));
    end

    title("Abstract Attitude Sensor Measurements")
    xlabel("Time (s)")
    ylabel("Magnitude")
    legend('show')
    grid on
end