%% Create Painting Trajectory for KUKA Robot
% Generates waypoints for different painting patterns

clc; clear; close all;

%% ===== CONFIGURATION =====
PATTERN = 'circle';  % Options: 'circle', 'line', 'rectangle', 'zigzag', 'spiral'
NUM_POINTS = 30;     % Number of waypoints in trajectory

% Center position (mm)
CENTER = [600, 100, 400];  % [X, Y, Z]

% Pattern parameters
RADIUS = 120;        % For circle/spiral (mm)
LENGTH = 200;        % For line/rectangle (mm)
WIDTH = 150;         % For rectangle/zigzag (mm)

% Tool orientation (degrees)
ORIENTATION = [0, 90, 0];  % [A, B, C] - perpendicular to surface

%% ===== GENERATE PATTERN =====
fprintf('Generating %s pattern with %d points...\n', PATTERN, NUM_POINTS);

switch lower(PATTERN)
    case 'circle'
        theta = linspace(0, 2*pi, NUM_POINTS);
        waypoints = zeros(NUM_POINTS, 6);
        for i = 1:NUM_POINTS
            waypoints(i, :) = [
                CENTER(1) + RADIUS * cos(theta(i)), ...  % X
                CENTER(2) + RADIUS * sin(theta(i)), ...  % Y
                CENTER(3), ...                           % Z
                ORIENTATION                              % A, B, C
            ];
        end
        
    case 'line'
        x = linspace(CENTER(1) - LENGTH/2, CENTER(1) + LENGTH/2, NUM_POINTS);
        waypoints = zeros(NUM_POINTS, 6);
        for i = 1:NUM_POINTS
            waypoints(i, :) = [x(i), CENTER(2), CENTER(3), ORIENTATION];
        end
        
    case 'rectangle'
        % Define corners
        corners = [
            CENTER(1) - LENGTH/2, CENTER(2) - WIDTH/2, CENTER(3);
            CENTER(1) + LENGTH/2, CENTER(2) - WIDTH/2, CENTER(3);
            CENTER(1) + LENGTH/2, CENTER(2) + WIDTH/2, CENTER(3);
            CENTER(1) - LENGTH/2, CENTER(2) + WIDTH/2, CENTER(3);
            CENTER(1) - LENGTH/2, CENTER(2) - WIDTH/2, CENTER(3)
        ];
        
        % Interpolate
        t = linspace(1, size(corners, 1), NUM_POINTS);
        waypoints = zeros(NUM_POINTS, 6);
        waypoints(:, 1) = interp1(1:size(corners,1), corners(:,1), t, 'pchip');
        waypoints(:, 2) = interp1(1:size(corners,1), corners(:,2), t, 'pchip');
        waypoints(:, 3) = interp1(1:size(corners,1), corners(:,3), t, 'pchip');
        waypoints(:, 4:6) = repmat(ORIENTATION, NUM_POINTS, 1);
        
    case 'zigzag'
        num_rows = 6;
        points_per_row = ceil(NUM_POINTS / num_rows);
        waypoints = zeros(NUM_POINTS, 6);
        idx = 1;
        
        for row = 1:num_rows
            y_offset = (row - 1) * (WIDTH / (num_rows - 1)) - WIDTH/2;
            
            if mod(row, 2) == 1
                x_vals = linspace(CENTER(1) - LENGTH/2, CENTER(1) + LENGTH/2, points_per_row);
            else
                x_vals = linspace(CENTER(1) + LENGTH/2, CENTER(1) - LENGTH/2, points_per_row);
            end
            
            for p = 1:points_per_row
                if idx <= NUM_POINTS
                    waypoints(idx, :) = [
                        x_vals(p), ...
                        CENTER(2) + y_offset, ...
                        CENTER(3), ...
                        ORIENTATION
                    ];
                    idx = idx + 1;
                end
            end
        end
        
    case 'spiral'
        theta = linspace(0, 4*pi, NUM_POINTS);
        radius_vals = linspace(RADIUS/4, RADIUS, NUM_POINTS);
        waypoints = zeros(NUM_POINTS, 6);
        
        for i = 1:NUM_POINTS
            waypoints(i, :) = [
                CENTER(1) + radius_vals(i) * cos(theta(i)), ...
                CENTER(2) + radius_vals(i) * sin(theta(i)), ...
                CENTER(3), ...
                ORIENTATION
            ];
        end
        
    otherwise
        error('Unknown pattern: %s', PATTERN);
end

fprintf('✓ Generated %d waypoints\n', size(waypoints, 1));

%% ===== VISUALIZE =====
figure('Name', 'KUKA Painting Trajectory', 'Position', [100 100 1000 700]);

% 3D view
subplot(2,2,[1 3]);
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'b-', 'LineWidth', 2);
hold on;
plot3(waypoints(1,1), waypoints(1,2), waypoints(1,3), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
plot3(waypoints(end,1), waypoints(end,2), waypoints(end,3), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
scatter3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 50, 1:NUM_POINTS, 'filled');
colorbar;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title(sprintf('%s Pattern (%d points)', upper(PATTERN), NUM_POINTS));
grid on; axis equal; view(45, 30);
legend('Path', 'Start', 'End', 'Waypoints');

% Top view (XY)
subplot(2,2,2);
plot(waypoints(:,1), waypoints(:,2), 'b-o', 'LineWidth', 1.5);
hold on;
plot(waypoints(1,1), waypoints(1,2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(waypoints(end,1), waypoints(end,2), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
xlabel('X (mm)'); ylabel('Y (mm)');
title('Top View (XY)');
grid on; axis equal;

% Side view (XZ)
subplot(2,2,4);
plot(waypoints(:,1), waypoints(:,3), 'b-o', 'LineWidth', 1.5);
hold on;
plot(waypoints(1,1), waypoints(1,3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(waypoints(end,1), waypoints(end,3), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
xlabel('X (mm)'); ylabel('Z (mm)');
title('Side View (XZ)');
grid on; axis equal;

%% ===== DISPLAY INFO =====
fprintf('\n=== Trajectory Information ===\n');
fprintf('Pattern:     %s\n', PATTERN);
fprintf('Points:      %d\n', size(waypoints, 1));
fprintf('X range:     [%.1f, %.1f] mm\n', min(waypoints(:,1)), max(waypoints(:,1)));
fprintf('Y range:     [%.1f, %.1f] mm\n', min(waypoints(:,2)), max(waypoints(:,2)));
fprintf('Z range:     [%.1f, %.1f] mm\n', min(waypoints(:,3)), max(waypoints(:,3)));
fprintf('Orientation: A=%.1f°, B=%.1f°, C=%.1f°\n', ORIENTATION(1), ORIENTATION(2), ORIENTATION(3));

%% ===== SAVE (FIXED - Consistent field names) =====
filename = sprintf('trajectory_%s_%dpts.mat', PATTERN, NUM_POINTS);

% Save with consistent field names
save(filename, 'waypoints', 'PATTERN', 'NUM_POINTS', 'CENTER', 'ORIENTATION');

fprintf('\n✓ Trajectory saved: %s\n', filename);
fprintf('  Variables saved: waypoints, PATTERN, NUM_POINTS, CENTER, ORIENTATION\n');
fprintf('\nNext: Run execute_painting_trajectory.m\n');