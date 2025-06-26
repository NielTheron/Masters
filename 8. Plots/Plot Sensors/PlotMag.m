function PlotMag(x_true, dt, MAG_measurement)
    figure('Name', "Sensor Comparison")
    hold on
    n = (0:size(x_true, 2)-1) * dt;

    colors = {'r', 'g', 'b'};
    labels = {'X', 'Y', 'Z'};

    for i = 1:3
        % Get non-zero indices
        valid_idx = MAG_measurement(i,:) ~= 0;
        n_valid = n(valid_idx);
        MAG_valid = MAG_measurement(i, valid_idx);

        % Plot filled scatter points
        scatter(n_valid, MAG_valid, 20, colors{i}, 'filled', ...
            'DisplayName', sprintf('MAG %s', labels{i}));
    end

    title("Abstract Magnetometer Sensor Measurements")
    xlabel("Time (s)");
    ylabel("Magnitude");
    legend("show")
    grid on
end