options = struct();
options.show_all_axes = true;     % Show X, Y, Z paths
options.show_cube = true;         % Show satellite cube
options.cube_size = 0.15;         % Larger cube
options.show_sphere = true;       % Unit sphere
options.line_width = 3;           % Thicker lines
options.title_text = 'Satellite Attitude Dynamics - Earth Pointing';

PlotBodyZAxisPath(x_true(7:10,:), dt_p, options);