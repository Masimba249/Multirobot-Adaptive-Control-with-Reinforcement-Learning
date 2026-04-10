

clc; clear;

% Load old file
old_file = 'trajectory_circle_30pts.mat';
fprintf('Loading: %s\n', old_file);

data = load(old_file);

% Display what's in the file
fprintf('Variables in file:\n');
disp(fieldnames(data));

% Extract waypoints (check for different possible field names)
if isfield(data, 'trajectory_data')
    % Old format with struct
    waypoints = data.trajectory_data.waypoints;
    PATTERN = data.trajectory_data.pattern_type;
    NUM_POINTS = data.trajectory_data.num_waypoints;
    CENTER = [600, 100, 400];  % Default
    ORIENTATION = [0, 90, 0];  % Default
elseif isfield(data, 'waypoints')
    % Already correct format
    waypoints = data.waypoints;
    PATTERN = data.PATTERN;
    NUM_POINTS = data.NUM_POINTS;
    CENTER = data.CENTER;
    ORIENTATION = data.ORIENTATION;
else
    error('Cannot find waypoints in file!');
end

fprintf('✓ Found %d waypoints\n', size(waypoints, 1));

% Save in new format
new_file = 'trajectory_circle_25pts_FIXED.mat';
save(new_file, 'waypoints', 'PATTERN', 'NUM_POINTS', 'CENTER', 'ORIENTATION');

fprintf('✓ Fixed file saved: %s\n', new_file);
fprintf('\nNow run execute_painting_trajectory.m and select the FIXED file.\n');