% Test with and without inertia coupling effects
N = 10000;
dt_test = 0.01;  % Smaller time step for better integration
w_initial = [0, 0.1, 0.1]; % Initial angular velocity: 0.1 rad/s about Z
I = [1.5; 0.8; 1.2]; % Asymmetric inertia tensor

%% WITHOUT coupling - constant angular velocity
q_no_coupling = zeros(4, N);
q_no_coupling(:,1) = [1, 0, 0, 0];
w_current = w_initial;

for i = 1:N-1
    q = q_no_coupling(:,i);
    % Quaternion derivative with constant w
    q_dot = 0.5 * [-w_current(1)*q(2) - w_current(2)*q(3) - w_current(3)*q(4);
                   w_current(1)*q(1) + w_current(3)*q(3) - w_current(2)*q(4);
                   w_current(2)*q(1) - w_current(3)*q(2) + w_current(1)*q(4);
                   w_current(3)*q(1) + w_current(2)*q(2) - w_current(1)*q(3)];
    q_no_coupling(:,i+1) = q + q_dot * dt_test;
    q_no_coupling(:,i+1) = q_no_coupling(:,i+1) / norm(q_no_coupling(:,i+1));
    % w_current stays the same (no coupling)
end

%% WITH coupling - integrate Euler's equations
q_with_coupling = zeros(4, N);
q_with_coupling(:,1) = [1, 0, 0, 0];
w_current = w_initial;

for i = 1:N-1
    q = q_with_coupling(:,i);
    
    % Calculate coupling torque: M = omega x (I*omega)
    I_omega = [I(1)*w_current(1); I(2)*w_current(2); I(3)*w_current(3)];
    M_coupling = [w_current(2)*I_omega(3) - w_current(3)*I_omega(2);
                  w_current(3)*I_omega(1) - w_current(1)*I_omega(3);
                  w_current(1)*I_omega(2) - w_current(2)*I_omega(1)];
    
    % Angular acceleration: omega_dot = -inv(I) * (omega x (I*omega))
    omega_dot = [-M_coupling(1)/I(1); -M_coupling(2)/I(2); -M_coupling(3)/I(3)];
    
    % Update angular velocity
    w_current = w_current + omega_dot' * dt_test;
    
    % Quaternion derivative with updated w
    q_dot = 0.5 * [-w_current(1)*q(2) - w_current(2)*q(3) - w_current(3)*q(4);
                   w_current(1)*q(1) + w_current(3)*q(3) - w_current(2)*q(4);
                   w_current(2)*q(1) - w_current(3)*q(2) + w_current(1)*q(4);
                   w_current(3)*q(1) + w_current(2)*q(2) - w_current(1)*q(3)];
    q_with_coupling(:,i+1) = q + q_dot * dt_test;
    q_with_coupling(:,i+1) = q_with_coupling(:,i+1) / norm(q_with_coupling(:,i+1));
end

%% Plot WITHOUT inertia coupling
PlotBodyToECI(q_no_coupling, dt_test, struct('title_text', 'No Coupling', 'include_inertia', false));
set(gcf, 'Color', 'black');
set(gca, 'Color', 'black', 'XColor', 'white', 'YColor', 'white', 'ZColor', 'white');
title('No Inertia Coupling', 'Color', 'white');
xlabel('X (ECI)', 'Color', 'white');
ylabel('Y (ECI)', 'Color', 'white');
zlabel('Z (ECI)', 'Color', 'white');

%% Plot WITH inertia coupling
options = struct();
options.title_text = 'With Coupling Effects';
options.include_inertia = true;
options.I_body = I;
options.omega_body = w_initial';
options.show_inertia_effects = true;

PlotBodyToECI(q_with_coupling, dt_test, options);
set(gcf, 'Color', 'black');
set(gca, 'Color', 'black', 'XColor', 'white', 'YColor', 'white', 'ZColor', 'white');
title('With Inertia Coupling Effects', 'Color', 'white');
xlabel('X (ECI)', 'Color', 'white');
ylabel('Y (ECI)', 'Color', 'white');
zlabel('Z (ECI)', 'Color', 'white');