function PlotCSS(x_true, dt, CSS_measurement)
    figure('Name', "Sensor Comparison")
    hold on
    n = (0:size(x_true, 2)-1) * dt;

    % Colors for each axis
    colors = {'b', 'g', 'r'};
    labels = {'x', 'y', 'z'};

    for i = 1:3
        % Get non-zero indices
        valid_idx = CSS_measurement(i,:) ~= 0;
        n_valid = n(valid_idx);
        CSS_valid = CSS_measurement(i, valid_idx);

        % Plot filled scatter points
        scatter(n_valid, CSS_valid, 20, colors{i}, 'filled', ...
            'DisplayName', sprintf('CSS %s', labels{i}));
    end

    title("Abstract Coarse Sun Sensor Measurements");
    xlabel("Time (s)");
    ylabel("Magnitude");
    legend;
    grid on
end