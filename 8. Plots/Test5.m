% Use the corrected version that accounts for orbital motion
options = struct();
options.show_orbital_drift = true;  % Show both ECI and orbital-relative plots
PlotBodyFrameCorrected(x_true(1:3,:), x_true(4:6,:), x_true(7:10,:), dt_p, options);