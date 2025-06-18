figure;
hold on;

% Define your precise vectors
vec1_start = [0, 0, 0];
vec1_end = [4200.51138392734, 125.980389611690, 4781.87796872852];
vec1_direction = vec1_end - vec1_start;

vec2_start = [0, 0, 0];
vec2_end = [4532.53556958206, 79.8777492857434, 5156.78084885455];
vec2_direction = vec2_end - vec2_start;

vec3_start = vec2_end;  % V3 starts where V2 ends
vec3_direction = [-332.024185654718, 46.1026403259465, -374.902880126025];

% Verify V3 = V1 - V2
vec3_calculated = vec1_direction - vec2_direction;
verification_diff = vec3_direction - vec3_calculated;
fprintf('V3 verification (should be ~zero): [%.2e, %.2e, %.2e]\n', verification_diff);

% Calculate magnitudes for reference
mag1 = norm(vec1_direction);
mag2 = norm(vec2_direction);
mag3 = norm(vec3_direction);

fprintf('Vector magnitudes:\n');
fprintf('Vector 1: %.2f\n', mag1);
fprintf('Vector 2: %.2f\n', mag2);
fprintf('Vector 3: %.2f\n', mag3);

% Plot vectors with automatic scaling (scale factor = 1)
quiver3(vec1_start(1), vec1_start(2), vec1_start(3), ...
        vec1_direction(1), vec1_direction(2), vec1_direction(3), ...
        1, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.05);

quiver3(vec2_start(1), vec2_start(2), vec2_start(3), ...
        vec2_direction(1), vec2_direction(2), vec2_direction(3), ...
        1, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.05);

quiver3(vec3_start(1), vec3_start(2), vec3_start(3), ...
        vec3_direction(1), vec3_direction(2), vec3_direction(3), ...
        1, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.05);

% Set equal aspect ratio for true scaling
axis equal;
grid on;

% Labels and title
xlabel('X (km)');
ylabel('Y (km)');
zlabel('Z (km)');
title('Satellite Position Vectors (To Scale)');
legend('Vector 1', 'Vector 2', 'Vector 3', 'Location', 'best');

% Verify the vector triangle closes perfectly
triangle_closure = vec2_direction + vec3_direction - vec1_direction;
fprintf('Triangle closure check (should be ~[0,0,0]): [%.2e, %.2e, %.2e]\n', triangle_closure);

% Optional: Add text annotations showing magnitudes
text(vec1_end(1)/2, vec1_end(2)/2, vec1_end(3)/2, ...
     sprintf('V1: |%.1f|', mag1), 'FontSize', 10);
text(vec2_end(1)/2, vec2_end(2)/2, vec2_end(3)/2, ...
     sprintf('V2: |%.1f|', mag2), 'FontSize', 10);
text(vec3_start(1) + vec3_direction(1)/2, ...
     vec3_start(2) + vec3_direction(2)/2, ...
     vec3_start(3) + vec3_direction(3)/2, ...
     sprintf('V3: |%.1f|', mag3), 'FontSize', 10);

% Set viewing angle for better visualization
view(45, 30);

hold off;