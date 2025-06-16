function PureRollTestWithInertia(I_values, initial_w, dt_test, total_time, plot_title)
%==========================================================================
% Pure Roll Test with Moment of Inertia Effects
%==========================================================================
% Purpose: Test quaternion propagation with Euler's rigid body equations
%          to show the effects of asymmetric inertia on attitude dynamics
%
% Inputs:
% I_values   - [Ixx, Iyy, Izz] moment of inertia vector (kg*m^2)
% initial_w  - [wx, wy, wz] initial angular velocity (rad/s)
% dt_test    - time step (s)
% total_time - total simulation time (s)
% plot_title - title for the plot
%==========================================================================

if nargin < 5
    plot_title = 'Roll Test with Inertia Effects';
end

% Simulation parameters
N = round(total_time / dt_test);
dt = dt_test;

% Initialize arrays
q_test = zeros(4, N);
w_test = zeros(3, N);

% Initial conditions
q_test(:,1) = [1, 0, 0, 0];  % Start ECI-aligned
w_test(:,1) = initial_w;     % Initial angular velocity

% Moment of inertia
Ixx = I_values(1);
Iyy = I_values(2); 
Izz = I_values(3);

fprintf('=== INERTIA TEST PARAMETERS ===\n');
fprintf('Inertia: [%.2f, %.2f, %.2f] kg*m^2\n', Ixx, Iyy, Izz);
fprintf('Initial ω: [%.3f, %.3f, %.3f] rad/s\n', initial_w);
fprintf('Simulation: %.1f s, dt = %.3f s\n', total_time, dt);

% Check for symmetry
if abs(Ixx - Iyy) < 1e-6 && abs(Iyy - Izz) < 1e-6
    fprintf('SYMMETRIC inertia - no coupling expected\n');
else
    fprintf('ASYMMETRIC inertia - coupling effects expected\n');
end
fprintf('===============================\n\n');

% Simulation loop
for i = 1:N-1
    % Current state
    q = q_test(:,i);
    w = w_test(:,i);
    
    %% Step 1: Angular velocity dynamics (Euler's equations)
    wx = w(1); wy = w(2); wz = w(3);
    
    % Euler's rigid body equations
    wx_dot = ((Iyy - Izz) * wy * wz) / Ixx;
    wy_dot = ((Izz - Ixx) * wz * wx) / Iyy;
    wz_dot = ((Ixx - Iyy) * wx * wy) / Izz;
    
    % Integrate angular velocity
    w_new = w + [wx_dot; wy_dot; wz_dot] * dt;
    
    %% Step 2: Quaternion dynamics
    % Use the UPDATED angular velocity for quaternion propagation
    w_avg = (w + w_new) / 2;  % Use average for better accuracy
    
    % Quaternion derivative: q_dot = 0.5 * [0; w] ⊗ q
    q_dot = 0.5 * [-w_avg(1)*q(2) - w_avg(2)*q(3) - w_avg(3)*q(4);
                    w_avg(1)*q(1) + w_avg(3)*q(3) - w_avg(2)*q(4);
                    w_avg(2)*q(1) - w_avg(3)*q(2) + w_avg(1)*q(4);
                    w_avg(3)*q(1) + w_avg(2)*q(2) - w_avg(1)*q(3)];
    
    % Integrate quaternion
    q_new = q + q_dot * dt;
    
    % Normalize quaternion
    q_new = q_new / norm(q_new);
    
    % Prevent quaternion sign flips
    if dot(q_new, q) < 0
        q_new = -q_new;
    end
    
    % Store results
    q_test(:,i+1) = q_new;
    w_test(:,i+1) = w_new;
end

% Print final statistics
fprintf('=== SIMULATION RESULTS ===\n');
fprintf('Initial ω: [%.6f, %.6f, %.6f] rad/s\n', w_test(:,1));
fprintf('Final ω:   [%.6f, %.6f, %.6f] rad/s\n', w_test(:,end));
fprintf('Δω:        [%.6f, %.6f, %.6f] rad/s\n', w_test(:,end) - w_test(:,1));

% Calculate total angular momentum (should be conserved!)
H_initial = [Ixx*w_test(1,1), Iyy*w_test(2,1), Izz*w_test(3,1)];
H_final = [Ixx*w_test(1,end), Iyy*w_test(2,end), Izz*w_test(3,end)];
fprintf('Initial H: [%.6f, %.6f, %.6f] kg*m^2*rad/s\n', H_initial);
fprintf('Final H:   [%.6f, %.6f, %.6f] kg*m^2*rad/s\n', H_final);
fprintf('|H| change: %.8f%% (should be ~0)\n', ...
        100*abs(norm(H_final) - norm(H_initial))/norm(H_initial));
fprintf('==========================\n\n');

% Create plots
figure('Name', 'Inertia Effects Analysis', 'Position', [100, 100, 1400, 800]);

% Plot 1: Body frame orientation paths
subplot(2,3,1);
PlotBodyToECI(q_test, dt_test, struct('title_text', plot_title, 'show_cube', false));

% Plot 2: Angular velocity evolution  
subplot(2,3,2);
time = (0:N-1) * dt;
plot(time, w_test(1,:), 'r-', 'LineWidth', 2, 'DisplayName', 'ωx (roll)');
hold on;
plot(time, w_test(2,:), 'g-', 'LineWidth', 2, 'DisplayName', 'ωy (pitch)');
plot(time, w_test(3,:), 'b-', 'LineWidth', 2, 'DisplayName', 'ωz (yaw)');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Angular Velocity Evolution');
legend('Location', 'best');
grid on;

% Plot 3: Angular momentum conservation
subplot(2,3,3);
H_body = [Ixx * w_test(1,:); Iyy * w_test(2,:); Izz * w_test(3,:)];
H_magnitude = sqrt(sum(H_body.^2, 1));
plot(time, H_magnitude, 'k-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('|H| (kg⋅m²⋅rad/s)');
title('Angular Momentum Magnitude');
grid on;

% Plot 4: Phase portrait (wx vs wy)
subplot(2,3,4);
plot(w_test(1,:), w_test(2,:), 'b-', 'LineWidth', 1.5);
hold on;
plot(w_test(1,1), w_test(2,1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(w_test(1,end), w_test(2,end), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('ωx (rad/s)');
ylabel('ωy (rad/s)');
title('Phase Portrait: ωx vs ωy');
grid on;
legend('Trajectory', 'Start', 'End', 'Location', 'best');

% Plot 5: Energy analysis
subplot(2,3,5);
T_rotational = 0.5 * (Ixx * w_test(1,:).^2 + Iyy * w_test(2,:).^2 + Izz * w_test(3,:).^2);
plot(time, T_rotational, 'm-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Rotational Energy (J)');
title('Rotational Kinetic Energy');
grid on;

% Plot 6: Coupling analysis
subplot(2,3,6);
coupling_xy = abs(w_test(1,:) .* w_test(2,:));
coupling_xz = abs(w_test(1,:) .* w_test(3,:));
coupling_yz = abs(w_test(2,:) .* w_test(3,:));
semilogy(time, coupling_xy, 'r-', 'LineWidth', 1.5, 'DisplayName', '|ωx⋅ωy|');
hold on;
semilogy(time, coupling_xz, 'g-', 'LineWidth', 1.5, 'DisplayName', '|ωx⋅ωz|');
semilogy(time, coupling_yz, 'b-', 'LineWidth', 1.5, 'DisplayName', '|ωy⋅ωz|');
xlabel('Time (s)');
ylabel('Coupling Magnitude');
title('Angular Velocity Coupling');
legend('Location', 'best');
grid on;

end