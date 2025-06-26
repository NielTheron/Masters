function PlotMeasurementError(y_true,y_est,n_f,dt)

    figure('Name', 'Measurement Error')
    hold on
    n = (0:size(y_est,3)-1) * dt;

    y_err = y_true - y_est;

    % Axis colors and labels
    colors = {'b', 'g', 'r'};
    labels = {'x', 'y', 'z'};

    for i = 1:n_f
        for k = 1:3  % x, y, z components
            y_vals = squeeze(y_err(k, i, :))';  % Get 1×time vector
            valid_idx = y_vals ~= 0;
            n_valid = n(valid_idx);
            y_valid = y_vals(valid_idx);

            % Scatter plot for non-zero measurements
            scatter(n_valid, y_valid, 20, colors{k}, 'filled', ...
                'DisplayName', sprintf('Feature %d %s', i, labels{k}));
        end
    end

    title("Measurement Error")
    xlabel("Time (s)")
    ylabel("Distance (km) in Body Frame")
    % legend('show')
    grid on
    hold off
end