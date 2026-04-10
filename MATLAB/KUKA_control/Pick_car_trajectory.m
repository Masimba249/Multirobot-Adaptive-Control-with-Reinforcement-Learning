%% Create Pick and Place Trajectory for KUKA Robot
% FIXED VERSION - Proper array creation

clc; clear; close all;

%% ===== CONFIGURATION =====
PICK_LOCATION = [550, -50, 300];   % [X, Y, Z] in mm
PLACE_LOCATION = [550, 200, 300];  % [X, Y, Z] in mm
APPROACH_HEIGHT = 150;    % mm above object
GRIP_HEIGHT = 0;          % mm
LIFT_HEIGHT = 150;        % mm
ORIENTATION_DOWN = [0, 90, 0];  % [A, B, C] degrees

%% ===== GENERATE WAYPOINTS =====
fprintf('Generating Pick and Place trajectory...\n');

% Pre-allocate waypoints matrix (8 rows × 6 columns)
waypoints = zeros(8, 6);

% Define each waypoint explicitly
waypoints(1,:) = [500, 0, 500, 0, 90, 0];  % HOME
waypoints(2,:) = [PICK_LOCATION(1), PICK_LOCATION(2), PICK_LOCATION(3) + APPROACH_HEIGHT, ORIENTATION_DOWN];  % APPROACH PICK
waypoints(3,:) = [PICK_LOCATION(1), PICK_LOCATION(2), PICK_LOCATION(3) + GRIP_HEIGHT, ORIENTATION_DOWN];  % PICK
waypoints(4,:) = [PICK_LOCATION(1), PICK_LOCATION(2), PICK_LOCATION(3) + LIFT_HEIGHT, ORIENTATION_DOWN];  % LIFT
waypoints(5,:) = [PLACE_LOCATION(1), PLACE_LOCATION(2), PLACE_LOCATION(3) + LIFT_HEIGHT, ORIENTATION_DOWN];  % APPROACH PLACE
waypoints(6,:) = [PLACE_LOCATION(1), PLACE_LOCATION(2), PLACE_LOCATION(3) + GRIP_HEIGHT, ORIENTATION_DOWN];  % PLACE
waypoints(7,:) = [PLACE_LOCATION(1), PLACE_LOCATION(2), PLACE_LOCATION(3) + APPROACH_HEIGHT, ORIENTATION_DOWN];  % RETRACT
waypoints(8,:) = [500, 0, 500, 0, 90, 0];  % RETURN HOME

% Create waypoint names as COLUMN cell array (CRITICAL FIX)
waypoint_names = {
    'HOME - Safe starting position';
    'APPROACH PICK - Above object';
    'PICK - At object (CLOSE GRIPPER)';
    'LIFT - Raised with object';
    'APPROACH PLACE - Above drop zone';
    'PLACE - At drop position (OPEN GRIPPER)';
    'RETRACT - After placing';
    'RETURN HOME - Safe end position'
};

% Create wait times as COLUMN vector (CRITICAL FIX)
wait_times = [
    2.0;  % HOME
    2.0;  % APPROACH PICK
    3.0;  % PICK (time for gripper)
    2.0;  % LIFT
    3.0;  % MOVE TO PLACE
    2.0;  % PLACE (time for gripper)
    2.0;  % RETRACT
    2.0   % RETURN HOME
];

NUM_POINTS = size(waypoints, 1);

fprintf('✓ Generated %d waypoints\n', NUM_POINTS);

%% ===== DISPLAY SEQUENCE =====
fprintf('\n=== Pick and Place Sequence ===\n');
for i = 1:NUM_POINTS
    fprintf('[%d] %s\n', i, waypoint_names{i});
    fprintf('    X=%6.1f, Y=%6.1f, Z=%6.1f, A=%5.1f, B=%5.1f, C=%5.1f\n', ...
           waypoints(i,1), waypoints(i,2), waypoints(i,3), ...
           waypoints(i,4), waypoints(i,5), waypoints(i,6));
end

%% ===== TRAJECTORY INFORMATION =====
fprintf('\n=== Trajectory Information ===\n');
fprintf('Pick Location:    X=%.1f, Y=%.1f, Z=%.1f mm\n', PICK_LOCATION);
fprintf('Place Location:   X=%.1f, Y=%.1f, Z=%.1f mm\n', PLACE_LOCATION);
fprintf('Lateral distance: %.1f mm (Y direction)\n', abs(PLACE_LOCATION(2) - PICK_LOCATION(2)));
fprintf('Approach height:  %.1f mm\n', APPROACH_HEIGHT);
fprintf('Lift height:      %.1f mm\n', LIFT_HEIGHT);
fprintf('Total waypoints:  %d\n', NUM_POINTS);

fprintf('\nWorkspace Range:\n');
fprintf('  X: [%.1f, %.1f] mm\n', min(waypoints(:,1)), max(waypoints(:,1)));
fprintf('  Y: [%.1f, %.1f] mm\n', min(waypoints(:,2)), max(waypoints(:,2)));
fprintf('  Z: [%.1f, %.1f] mm\n', min(waypoints(:,3)), max(waypoints(:,3)));

%% ===== VISUALIZE =====
figure('Name', 'Pick and Place Trajectory', 'Position', [100 100 1200 800]);

% Main 3D view
subplot(2,3,[1 2 4 5]);
hold on; grid on; axis equal;

% Plot trajectory
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'b-', 'LineWidth', 2);

% Plot waypoints
colors = lines(NUM_POINTS);
for i = 1:NUM_POINTS
    scatter3(waypoints(i,1), waypoints(i,2), waypoints(i,3), 150, colors(i,:), ...
            'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    text(waypoints(i,1)+10, waypoints(i,2)+10, waypoints(i,3)+10, ...
         sprintf('%d', i), 'FontSize', 10, 'FontWeight', 'bold');
end

% Mark pick/place locations
scatter3(PICK_LOCATION(1), PICK_LOCATION(2), PICK_LOCATION(3), 300, 'g', ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
text(PICK_LOCATION(1), PICK_LOCATION(2), PICK_LOCATION(3)-30, 'PICK', ...
     'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

scatter3(PLACE_LOCATION(1), PLACE_LOCATION(2), PLACE_LOCATION(3), 300, 'r', ...
        'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
text(PLACE_LOCATION(1), PLACE_LOCATION(2), PLACE_LOCATION(3)-30, 'PLACE', ...
     'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('Pick and Place Trajectory - 3D View');
view(45, 30);

% Top view
subplot(2,3,3);
plot(waypoints(:,1), waypoints(:,2), 'b-o', 'LineWidth', 2, 'MarkerSize', 8);
hold on;
scatter(PICK_LOCATION(1), PICK_LOCATION(2), 200, 'g', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
scatter(PLACE_LOCATION(1), PLACE_LOCATION(2), 200, 'r', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
xlabel('X (mm)'); ylabel('Y (mm)');
title('Top View (XY)');
grid on; axis equal;

% Side view
subplot(2,3,6);
plot(waypoints(:,2), waypoints(:,3), 'b-o', 'LineWidth', 2, 'MarkerSize', 8);
hold on;
scatter(PICK_LOCATION(2), PICK_LOCATION(3), 200, 'g', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
scatter(PLACE_LOCATION(2), PLACE_LOCATION(3), 200, 'r', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
xlabel('Y (mm)'); ylabel('Z (mm)');
title('Side View (YZ)');
grid on; axis equal;

%% ===== SAVE (WITH PROPER VARIABLE TYPES) =====
PATTERN = 'pick_and_place';

% Save with explicit variable names - all properly formatted
save('trajectory_pick_and_place.mat', ...
     'waypoints', 'waypoint_names', 'wait_times', ...
     'PATTERN', 'NUM_POINTS', 'PICK_LOCATION', 'PLACE_LOCATION');

fprintf('\n✓ Trajectory saved: trajectory_pick_and_place.mat\n');
fprintf('  File location: %s\n', fullfile(pwd, 'trajectory_pick_and_place.mat'));

fprintf('\n📋 Waypoint Summary:\n');
for i = 1:NUM_POINTS
    fprintf('  [%d] %s\n', i, waypoint_names{i});
end

fprintf('\n⚠️  IMPORTANT NOTES:\n');
fprintf('  • This trajectory assumes you have a gripper on the robot\n');
fprintf('  • At waypoint 3 (PICK): Close gripper manually or via signal\n');
fprintf('  • At waypoint 6 (PLACE): Open gripper manually or via signal\n');
fprintf('  • Adjust PICK_LOCATION and PLACE_LOCATION to match your setup\n');
fprintf('  • Test at slow speed (T1 mode) first!\n');

fprintf('\n✅ READY! Now run: run_pick_and_place.m\n');