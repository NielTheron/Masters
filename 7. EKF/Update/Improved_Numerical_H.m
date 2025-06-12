function H1 = Improved_Numerical_H(xm, c)
    % Catalogue vector
    d = [c(1); c(2); c(3); 1];
    
    % Allocate Jacobian
    n = length(xm);
    h0 = compute_h(xm, d); % Get baseline without normalization
    m = length(h0);
    H1 = zeros(m, n);
    
    % Use central difference method with carefully chosen step sizes
    for i = 1:n
        % Optimal step size selection
        if i >= 7 && i <= 10
            % Smaller epsilon for quaternion components
            epsilon = 1e-7;
        else
            % Adaptive step size - smaller than original for better precision
            epsilon = 1e-9 * max(1, abs(xm(i)));
        end
        
        dx = zeros(n, 1);
        dx(i) = epsilon;
        
        % Create perturbed state
        x_plus = xm + dx;
        x_minus = xm - dx;
        
        % Ensure quaternion normalization for perturbed states
        if i >= 7 && i <= 10
            x_plus(7:10) = quatnormalize(x_plus(7:10).');
            x_minus(7:10) = quatnormalize(x_minus(7:10).');
        end
        
        % Compute perturbed outputs without normalization to match analytical approach
        h_plus = compute_h(x_plus, d);
        h_minus = compute_h(x_minus, d);
        
        % Central difference approximation
        H1(:, i) = (h_plus - h_minus) / (2 * epsilon);
    end
    
    % Round to match analytical precision
    H1 = round(H1, 4, "significant");
end

function h = compute_h(x, d)

    
    % Extract state
    r = x(1:3); % Position vector
    v = x(4:6); % Velocity vector
    q = x(7:10); % Quaternion
    
    % Ensure quaternion is normalized
    q = quatnormalize(q.');
    
    % Compute orbital frame basis vectors with numerical stability
    r_norm = norm(r);
    if r_norm < 1e-10
        r_norm = 1e-10;
    end
    
    % Orbital frame z-axis points toward Earth center (nadir)
    z_orb = -r / r_norm;
    
    % Handle the case where velocity is parallel to position
    h_vec = cross(r, v);
    h_norm = norm(h_vec);
    if h_norm < 1e-10
        % Choose arbitrary orbital frame if cross product is too small
        if abs(z_orb(3)) < 0.9
            y_orb = cross(z_orb, [0; 0; 1]);
        else
            y_orb = cross(z_orb, [0; 1; 0]);
        end
    else
        % Standard case - compute y-axis perpendicular to orbital plane
        y_orb = cross(z_orb, v);
        y_orb = y_orb / norm(y_orb);
    end
    
    y_orb = y_orb / norm(y_orb);
    x_orb = cross(y_orb, z_orb);
    x_orb = x_orb / norm(x_orb);
    
    % Construct DCM from inertial to orbital frame
    Ri2o = [x_orb', 0;
             y_orb', 0;
             z_orb', 0;
             0, 0, 0, 1];
    
    % Translation matrix from inertial to orbital
    Ti2o = [1, 0, 0, -r(1);
            0, 1, 0, -r(2);
            0, 0, 1, -r(3);
            0, 0, 0, 1];
    
    % Complete transformation from inertial to orbital
    Ai2o = Ri2o * Ti2o;
    
    % Quaternion to Rotation Matrix (DCM) from Orbital to Body
    qs = q(1); qx = q(2); qy = q(3); qz = q(4);
    Ao2b = [1-2*(qy^2+qz^2), 2*(qx*qy-qs*qz), 2*(qx*qz+qs*qy), 0;
            2*(qx*qy+qs*qz), 1-2*(qx^2+qz^2), 2*(qy*qz-qs*qx), 0;
            2*(qx*qz-qs*qy), 2*(qy*qz+qs*qx), 1-2*(qx^2+qy^2), 0;
            0, 0, 0, 1];
    
    % Apply transforms to get feature vector in body frame
    h_vec = Ao2b * Ai2o * d;
    
    % Return only the direction vector (not homogeneous coordinate)
    h = h_vec(1:3);
end