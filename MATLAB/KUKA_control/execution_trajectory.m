%% Execute Painting Trajectory on KUKA Robot
% Uses your working connection method
% UPDATED: Handles different trajectory file formats

clc; clear; close all;

%% ===== CONFIGURATION =====
ROBOT_IP = '172.31.17.101';  % Your robot IP
PORT = 7000;
WAIT_TIME = 3.0;  % Seconds between waypoints (adjust based on robot speed)

%% ===== LOAD TRAJECTORY =====
fprintf('=== Loading Trajectory ===\n');

% Find trajectory files
traj_files = dir('trajectory_*.mat');
if isempty(traj_files)
    error('No trajectory files found! Run create_painting_trajectory.m first.');
end

% List available files
fprintf('Available trajectories:\n');
for i = 1:length(traj_files)
    fprintf('  [%d] %s\n', i, traj_files(i).name);
end

% Select file
if length(traj_files) == 1
    file_idx = 1;
else
    file_idx = input(sprintf('Select trajectory (1-%d): ', length(traj_files)));
end

selected_file = traj_files(file_idx).name;
fprintf('Loading: %s\n', selected_file);

data = load(selected_file);

% ===== HANDLE DIFFERENT FILE FORMATS =====
if isfield(data, 'waypoints')
    % New format - direct access
    waypoints = data.waypoints;
    if isfield(data, 'PATTERN')
        pattern_name = data.PATTERN;
    else
        pattern_name = 'unknown';
    end
    
elseif isfield(data, 'trajectory_data')
    % Old format - nested struct
    waypoints = data.trajectory_data.waypoints;
    if isfield(data.trajectory_data, 'pattern_type')
        pattern_name = data.trajectory_data.pattern_type;
    else
        pattern_name = 'unknown';
    end
    
else
    % Try to find any matrix that looks like waypoints (Nx6)
    fields = fieldnames(data);
    waypoints = [];
    for f = 1:length(fields)
        var = data.(fields{f});
        if isnumeric(var) && size(var, 2) == 6
            waypoints = var;
            fprintf('  Found waypoints in field: %s\n', fields{f});
            break;
        end
    end
    
    if isempty(waypoints)
        error('Cannot find waypoints in file! Expected Nx6 matrix.');
    end
    pattern_name = 'unknown';
end

fprintf('✓ Loaded %d waypoints\n', size(waypoints, 1));
fprintf('  Pattern: %s\n', pattern_name);

% Validate waypoints format
if size(waypoints, 2) ~= 6
    error('Invalid waypoints format! Expected Nx6 matrix [X Y Z A B C], got %dx%d', ...
          size(waypoints, 1), size(waypoints, 2));
end

%% ===== SAFETY CHECKS =====
fprintf('\n');
fprintf('╔════════════════════════════════════════════════╗\n');
fprintf('║          REAL ROBOT EXECUTION                  ║\n');
fprintf('║             SAFETY CHECKLIST                   ║\n');
fprintf('╚════════════════════════════════════════════════╝\n');
fprintf('\n');

safety_items = {
    'Robot powered ON'
    'MatlabControl.src running on SmartPAD'
    'Robot in T1 mode'
    'Deadman switch held'
    'Emergency stop accessible'
    'Workspace clear'
    'No people near robot'
    'KukaVarProxy running on controller'
};

for i = 1:length(safety_items)
    fprintf('  ☐ %s\n', safety_items{i});
end

fprintf('\n');
response = input('All checks complete? Type YES to continue: ', 's');
if ~strcmpi(response, 'YES')
    fprintf('✗ Execution cancelled.\n');
    return;
end

%% ===== CONNECT TO ROBOT =====
fprintf('\n=== Connecting to Robot ===\n');
fprintf('IP: %s, Port: %d\n', ROBOT_IP, PORT);

try
    tcp = tcpclient(ROBOT_IP, PORT, 'Timeout', 10);
    fprintf('✓ Connected to KUKA via KUKAVARPROXY\n');
catch ME
    error('Connection failed: %s', ME.message);
end

%% ===== MOVE TO START POSITION =====
fprintf('\n=== Moving to Start Position ===\n');

start_pos = waypoints(1, :);
fprintf('Target: X=%.1f, Y=%.1f, Z=%.1f, A=%.1f, B=%.1f, C=%.1f\n', ...
       start_pos(1), start_pos(2), start_pos(3), ...
       start_pos(4), start_pos(5), start_pos(6));

posStr = sprintf('{X %.3f,Y %.3f,Z %.3f,A %.3f,B %.3f,C %.3f}', ...
               start_pos(1), start_pos(2), start_pos(3), ...
               start_pos(4), start_pos(5), start_pos(6));

writeVar(tcp, 'target_pos', posStr);
pause(0.2);
writeVar(tcp, 'move_trigger', 'TRUE');

fprintf('⏳ Moving to start position...\n');
pause(WAIT_TIME + 2);
fprintf('✓ At start position\n');

input('\nPress ENTER to begin trajectory execution...', 's');

%% ===== EXECUTE TRAJECTORY =====
fprintf('\n');
fprintf('╔════════════════════════════════════════════════╗\n');
fprintf('║        EXECUTING PAINTING TRAJECTORY           ║\n');
fprintf('╚════════════════════════════════════════════════╝\n');
fprintf('\n');

start_time = tic;
success_count = 0;

for i = 1:size(waypoints, 1)
    % Display progress
    progress = (i / size(waypoints, 1)) * 100;
    fprintf('[%3d/%3d | %5.1f%%] ', i, size(waypoints, 1), progress);
    fprintf('X=%6.1f Y=%6.1f Z=%6.1f ', ...
           waypoints(i,1), waypoints(i,2), waypoints(i,3));
    fprintf('A=%5.1f B=%5.1f C=%5.1f ', ...
           waypoints(i,4), waypoints(i,5), waypoints(i,6));
    
    % Send position
    posStr = sprintf('{X %.3f,Y %.3f,Z %.3f,A %.3f,B %.3f,C %.3f}', ...
                   waypoints(i,1), waypoints(i,2), waypoints(i,3), ...
                   waypoints(i,4), waypoints(i,5), waypoints(i,6));
    
    success1 = writeVar(tcp, 'target_pos', posStr);
    pause(0.1);
    success2 = writeVar(tcp, 'move_trigger', 'TRUE');
    
    if success1 && success2
        fprintf('✓\n');
        success_count = success_count + 1;
    else
        fprintf('✗\n');
    end
    
    % Wait for movement
    pause(WAIT_TIME);
end

execution_time = toc(start_time);

%% ===== RETURN HOME =====
fprintf('\n=== Returning to Home ===\n');
home_pos = '{X 500,Y 0,Z 500,A 0,B 90,C 0}';
writeVar(tcp, 'target_pos', home_pos);
pause(0.2);
writeVar(tcp, 'move_trigger', 'TRUE');
pause(WAIT_TIME + 2);

%% ===== CLEANUP =====
clear tcp;

%% ===== SUMMARY =====
fprintf('\n');
fprintf('╔════════════════════════════════════════════════╗\n');
fprintf('║            EXECUTION COMPLETE                  ║\n');
fprintf('╚════════════════════════════════════════════════╝\n');
fprintf('\n');
fprintf('Results:\n');
fprintf('  Total waypoints:    %d\n', size(waypoints, 1));
fprintf('  Successful:         %d (%.1f%%)\n', success_count, (success_count/size(waypoints,1))*100);
fprintf('  Execution time:     %.1f seconds\n', execution_time);
fprintf('  Avg per waypoint:   %.2f seconds\n', execution_time/size(waypoints,1));
fprintf('\n✓ Done!\n');

%% ===== HELPER FUNCTION =====
function success = writeVar(tcp, varName, varValue)
    msgId = uint16(1);
    mode  = uint8(1);   % WRITE

    varNameBytes  = uint8(varName);
    varValueBytes = uint8(varValue);

    varNameLen  = uint16(length(varNameBytes));
    varValueLen = uint16(length(varValueBytes));

    contentLen = uint16( ...
        1 + ...
        2 + length(varNameBytes) + ...
        2 + length(varValueBytes) ...
    );

    msgIdBytes       = typecast(swapbytes(msgId),      'uint8');
    contentLenBytes  = typecast(swapbytes(contentLen), 'uint8');
    varNameLenBytes  = typecast(swapbytes(varNameLen), 'uint8');
    varValueLenBytes = typecast(swapbytes(varValueLen),'uint8');

    msg = [ ...
        msgIdBytes(:).' ...
        contentLenBytes(:).' ...
        mode ...
        varNameLenBytes(:).' ...
        varNameBytes ...
        varValueLenBytes(:).' ...
        varValueBytes ...
    ];

    write(tcp, msg);
    
    % Check response
    pause(0.05);
    success = true;
    if tcp.NumBytesAvailable > 0
        response = read(tcp, tcp.NumBytesAvailable);
        if length(response) >= 3
            tail = response(end-2:end);
            success = all(tail == [0 1 1]);
        end
    end
end