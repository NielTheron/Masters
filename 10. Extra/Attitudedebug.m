% Debug the roll behavior
options = struct();
options.debug_roll = true;  % Enable debug output
options.title_text = 'Body-to-ECI Quaternion Debug';

PlotBodyToECI(x_true(7:10,:), dt_p, options);