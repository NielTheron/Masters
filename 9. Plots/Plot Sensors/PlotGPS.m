function PlotGPS(x_true, x_EKF, dt, GPS_measurement)
    figure('Name', 'GPS Measurement')
    hold on
    n = (0:size(x_true,2)-1)*dt;

    % % Plot ground truth position
    % plot(n, x_true(1,:), 'b', 'DisplayName', 'x true');
    % plot(n, x_true(2,:), 'g', 'DisplayName', 'y true');
    % plot(n, x_true(3,:), 'r', 'DisplayName', 'z true');

    % Plot estimates
    plot(n, x_EKF(1,:), 'b', 'DisplayName', 'x hat');
    plot(n, x_EKF(2,:), 'g', 'DisplayName', 'y hat');
    plot(n, x_EKF(3,:), 'r', 'DisplayName', 'z hat');

    % Colors for each axis
    colors = {'b', 'g', 'r'};
    labels = {'x', 'y', 'z'};

    for i = 1:3
        for j = 1:length(n)
            value = GPS_measurement(i,j);
            if value ~= 0
                % Only draw lollipop when value is not zero
                plot([n(j), n(j)], [0, value], 'Color', colors{i}, 'LineStyle', '-', 'HandleVisibility', 'off');
                scatter(n(j), value, 20, colors{i}, 'filled', 'HandleVisibility', 'off');
            end
        end
        % One entry per axis in the legend
        scatter(n(1), GPS_measurement(i,1), 20, colors{i}, 'filled', ...
            'DisplayName', sprintf('GPS %s', labels{i}), 'HandleVisibility', 'on');
    end

    title("Abstract Position Measurement Sensors")
    xlabel("Time (s)");
    ylabel("Position (km)")
    legend('show')
    grid on
end
