% Example usage with moment of inertia
options.include_inertia = true;
options.I_body = [1.2; 0.8; 1.0];  % kg⋅m²
options.omega_body = [0.002; 0.001; 0.003];  % rad/s
options.show_inertia_effects = true;

PureRollTestWithInertia(, dt_p, options);