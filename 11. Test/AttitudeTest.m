% Test pure roll - should show Y,Z in perfect circles around X
N = 1000;
dt_test = 0.1;
q_test = zeros(4, N);
q_test(:,1) = [1, 0, 0, 0];  % Start aligned
w_roll = [0, 0, 0.1];        % Pure roll: 0.1 rad/s about X

for i = 1:N-1
    q = q_test(:,i);
    % Quaternion derivative: q_dot = 0.5 * [0; w] * q
    q_dot = 0.5 * [-w_roll(1)*q(2) - w_roll(2)*q(3) - w_roll(3)*q(4);
                    w_roll(1)*q(1) + w_roll(3)*q(3) - w_roll(2)*q(4);
                    w_roll(2)*q(1) - w_roll(3)*q(2) + w_roll(1)*q(4);
                    w_roll(3)*q(1) + w_roll(2)*q(2) - w_roll(1)*q(3)];
    
    q_test(:,i+1) = q + q_dot * dt_test;
    q_test(:,i+1) = q_test(:,i+1) / norm(q_test(:,i+1));  % Normalize
end

% Plot pure roll test
PlotBodyToECI(q_test, dt_test, struct('title_text', 'Pure Roll Test'));