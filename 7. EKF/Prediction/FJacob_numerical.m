%==========================================================================
% Niel Theron
% Date Modified: 01-05-2025 % (Example modification date)
%==========================================================================
% The purpose of this function is to numerically compute the state
% transition Jacobian matrix F = df/dx.
%==========================================================================
function F = FJacob_numerical(x0, I0, dt, Mu)

% Ensure inputs have correct shapes
x0 = x0(:); % Ensure column vector
I0 = I0(:); % Ensure column vector

% Jacobian via numerical differentiation
n = length(x0);
F = zeros(n, n);
epsilon = 1e-6; % Adjust step size as needed

% Calculate nominal next state using the separate FFunctionF
% This avoids duplicating the logic and ensures consistency
% f0 = FFunctionF(x0, I0, dt, Mu); % Can use this for verification if needed

for i = 1:n
    dx = zeros(n, 1);
    dx(i) = epsilon;

    % Create perturbed states
    x_plus = x0 + dx;
    x_minus = x0 - dx;

    % Normalize quaternion part if perturbed
    if i >= 7 && i <= 10
       x_plus(7:10) = quatnormalize(x_plus(7:10).');
       x_minus(7:10) = quatnormalize(x_minus(7:10).');
    end

    % Calculate next state for perturbed states using the consistent function
    f_forward = FFunctionF(x_plus, I0, dt, Mu);
    f_backward = FFunctionF(x_minus, I0, dt, Mu);

    % Central difference formula
    F(:, i) = (f_forward - f_backward) / (2 * epsilon);
end

end % End of main function FJacob_numerical