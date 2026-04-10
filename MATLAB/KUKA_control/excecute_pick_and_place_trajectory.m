%% Execute Pick-and-Place Trajectory on KUKA Robot — Smooth Continuous Motion
% IMPROVED VERSION: Zero-pause streaming with live telemetry + Torque/Velocity/Position plots

clc; clear; close all;

global ROBOT_STATE %#ok<GVMIS>

%% ===== CONFIGURATION =====
ROBOT_IP   = '172.31.17.101';
PORT       = 7000;
CONTROL_DT = 0.012;          % 12 ms -> ~83 Hz streaming rate

A4_GUARD_DEG = 170;
A5_GUARD_DEG = 110;

ENABLE_LIVE_PLOT = true;
PLOT_UPDATE_RATE = 8;

%% ===== LOAD TRAJECTORY =====
fprintf('===========================================================\n');
fprintf('   KUKA Pick-and-Place Execution — Continuous Streaming\n');
fprintf('===========================================================\n\n');

traj_files = dir('trajectory_pick_place_*.mat');
if isempty(traj_files)
    error('No trajectory files found! Run create_pick_and_place_trajectory.m first.');
end

fprintf('Available trajectories:\n');
for i = 1:length(traj_files)
    fprintf('  [%d] %s  (%s)\n', i, traj_files(i).name, traj_files(i).date);
end

if length(traj_files) == 1
    file_idx = 1;
    fprintf('\nAuto-selecting: %s\n', traj_files(1).name);
else
    file_idx = input(sprintf('\nSelect (1-%d): ', length(traj_files)));
    if isempty(file_idx) || file_idx < 1 || file_idx > length(traj_files)
        error('Invalid selection.');
    end
end

selected_file = traj_files(file_idx).name;
data = load(selected_file);

if ~isfield(data, 'waypoints')
    error('Missing "waypoints" in MAT file.');
end
waypoints = data.waypoints;

if isfield(data, 'key_poses')
    key_poses = data.key_poses;
else
    key_poses = waypoints([1, end], :);
end

waypoints = ensureRobotNativeEuler(waypoints);
key_poses = ensureRobotNativeEuler(key_poses);

NUM_POINTS = size(waypoints, 1);
fprintf('Loaded %d waypoints, %d key poses.\n\n', NUM_POINTS, size(key_poses,1));

%% ===== SAFETY =====
fprintf('=== SAFETY CHECKLIST ===\n');
items = {
    'Robot powered ON and initialized'
    'MatlabControl.src running (NOT paused)'
    'Robot in T1 mode, deadman switch ready'
    'Emergency stop accessible'
    'Workspace clear, object at pick location'
    'KUKAVARPROXY running on controller'
};
for i = 1:length(items)
    fprintf('  [ ] %s\n', items{i});
end

response = input('\nType YES to continue: ', 's');
if ~strcmpi(strtrim(response), 'YES')
    fprintf('Cancelled.\n'); return;
end

%% ===== CONNECT =====
fprintf('\nConnecting to %s:%d ...\n', ROBOT_IP, PORT);
try
    tcp = tcpclient(ROBOT_IP, PORT, 'Timeout', 10);
    fprintf('Connected.\n');
catch ME
    error('Connection failed: %s', ME.message);
end
cleanupObj = onCleanup(@() safeClose(tcp));

ROBOT_S = 2; ROBOT_T = 35;
[ROBOT_S, ROBOT_T] = readSTFromPosAct(tcp, ROBOT_S, ROBOT_T);
fprintf('Initial S=%d, T=%d\n', ROBOT_S, ROBOT_T);

%% ===== LOG ARRAYS =====
% Position: columns 1-3 = XYZ (mm), 4-6 = ABC (deg)
exec_pos   = nan(NUM_POINTS, 6);

% Velocity: estimated from consecutive position deltas / CONTROL_DT (mm/s)
exec_vel   = nan(NUM_POINTS, 3);   % VX VY VZ

% Speed magnitude (scalar, mm/s)
exec_speed = nan(NUM_POINTS, 1);

% Torque: read from $TORQUE_ACT  (Nm) — 6 joints
exec_torque = nan(NUM_POINTS, 6);

% Joint guards
a4_log     = nan(NUM_POINTS, 1);
a5_log     = nan(NUM_POINTS, 1);

% Time vector
exec_time_vec = nan(NUM_POINTS, 1);

a4_val = 0; a5_val = 0;
success_count = 0;
aborted = false;

axis_poll    = max(1, round(0.3  / CONTROL_DT));
st_refresh   = max(1, round(0.5  / CONTROL_DT));
torque_poll  = max(1, round(0.25 / CONTROL_DT));  % read torque every ~250 ms

%% ===== LIVE PLOT SETUP =====
if ENABLE_LIVE_PLOT
    fig = figure('Name', 'KUKA Execution — Live Telemetry', ...
        'Position', [20, 20, 1800, 980], 'Color', 'w');

    % ---- Row 1: 3D path + Position ----
    ax1 = subplot(3,3,[1,4]); hold on; grid on; axis equal; view(45,25);
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)'); title('3D Path');
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), ...
        'Color',[0.75 0.75 0.75], 'LineStyle','--', 'LineWidth', 1);
    h_exec = plot3(NaN,NaN,NaN, 'b-',  'LineWidth', 2.5);
    h_cur  = plot3(NaN,NaN,NaN, 'mo',  'MarkerSize', 12, 'MarkerFaceColor', 'm');

    ax_pos = subplot(3,3,2); hold on; grid on;
    title('Position (mm)'); xlabel('Step'); ylabel('mm');
    h_pX = animatedline(ax_pos, 'Color','r',   'LineWidth', 1.8, 'DisplayName','X');
    h_pY = animatedline(ax_pos, 'Color',[0 0.7 0], 'LineWidth', 1.8, 'DisplayName','Y');
    h_pZ = animatedline(ax_pos, 'Color','b',   'LineWidth', 1.8, 'DisplayName','Z');
    legend(ax_pos, 'Location','best');

    ax_ori = subplot(3,3,3); hold on; grid on;
    title('Orientation (deg)'); xlabel('Step'); ylabel('deg');
    h_oA = animatedline(ax_ori, 'Color','r',          'LineWidth', 1.8, 'DisplayName','A');
    h_oB = animatedline(ax_ori, 'Color',[0 0.7 0],    'LineWidth', 1.8, 'DisplayName','B');
    h_oC = animatedline(ax_ori, 'Color','b',           'LineWidth', 1.8, 'DisplayName','C');
    legend(ax_ori, 'Location','best');

    % ---- Row 2: Velocity ----
    ax_vel = subplot(3,3,5); hold on; grid on;
    title('Velocity (mm/s)'); xlabel('Step'); ylabel('mm/s');
    h_vX  = animatedline(ax_vel, 'Color','r',          'LineWidth', 1.8, 'DisplayName','Vx');
    h_vY  = animatedline(ax_vel, 'Color',[0 0.7 0],    'LineWidth', 1.8, 'DisplayName','Vy');
    h_vZ  = animatedline(ax_vel, 'Color','b',           'LineWidth', 1.8, 'DisplayName','Vz');
    legend(ax_vel, 'Location','best');

    ax_spd = subplot(3,3,6); hold on; grid on;
    title('Speed Magnitude (mm/s)'); xlabel('Step'); ylabel('mm/s');
    h_spd  = animatedline(ax_spd, 'Color',[0.5 0 0.8], 'LineWidth', 2, 'DisplayName','|V|');
    legend(ax_spd, 'Location','best');

    % ---- Row 3: Torque ----
    ax_trq = subplot(3,3,7); hold on; grid on;
    title('Joint Torque (Nm)  — from \$TORQUE\_ACT'); xlabel('Step'); ylabel('Nm');
    trqColors = {'r','g','b','m','c',[0.8 0.5 0]};
    h_trq = gobjects(6,1);
    for jj = 1:6
        h_trq(jj) = animatedline(ax_trq, 'Color', trqColors{jj}, ...
            'LineWidth', 1.5, 'DisplayName', sprintf('J%d',jj));
    end
    legend(ax_trq, 'Location','best');

    ax_guard = subplot(3,3,8); hold on; grid on;
    title('Joint Guards A4/A5 (deg)'); xlabel('Step'); ylabel('deg');
    h_a4 = animatedline(ax_guard, 'Color','m', 'LineWidth', 1.8, 'DisplayName','A4');
    h_a5 = animatedline(ax_guard, 'Color','c', 'LineWidth', 1.8, 'DisplayName','A5');
    yline(ax_guard,  A4_GUARD_DEG,  'r--', 'LineWidth', 1.2);
    yline(ax_guard, -A4_GUARD_DEG,  'r--', 'LineWidth', 1.2);
    yline(ax_guard,  A5_GUARD_DEG,  'b--', 'LineWidth', 1.2);
    yline(ax_guard, -A5_GUARD_DEG,  'b--', 'LineWidth', 1.2);
    legend(ax_guard, 'Location','best');

    ax_status = subplot(3,3,9); axis off;
    h_status = text(ax_status, 0.05, 0.95, 'Starting...', ...
        'FontSize', 10, 'VerticalAlignment','top', 'FontName','Courier');
end

%% ===== MOVE TO START =====
fprintf('\nMoving to start...\n');
sendE6Pos(tcp, waypoints(1,:));
pulseMoveTrigger(tcp);
nonBlockingWait(1.2);

[ROBOT_S, ROBOT_T] = readSTFromPosAct(tcp, ROBOT_S, ROBOT_T);
fprintf('Start reached. S=%d, T=%d\n', ROBOT_S, ROBOT_T);

input('Press ENTER to begin streaming...', 's');

%% ===== MAIN STREAMING LOOP =====
fprintf('\n=== STREAMING %d WAYPOINTS ===\n', NUM_POINTS);

t_start = tic;

for i = 1:NUM_POINTS
    t_step = tic;

    pos = waypoints(i,:);

    %% -- Send position --
    ok = sendE6Pos(tcp, pos);
    pulseMoveTrigger(tcp);
    exec_time_vec(i) = toc(t_start);

    if ok
        success_count  = success_count + 1;
        exec_pos(i,:)  = pos;

        % ---- Velocity estimation (finite difference) ----
        if i > 1 && ~any(isnan(exec_pos(i-1,:)))
            dxyz = pos(1:3) - exec_pos(i-1, 1:3);   % mm displacement
            vel  = dxyz / CONTROL_DT;                % mm/s
        else
            vel = [0 0 0];
        end
        exec_vel(i,:)   = vel;
        exec_speed(i)   = norm(vel);

        % ---- Update global ROBOT_STATE for OPC-UA timer ----
        if ~isempty(ROBOT_STATE)
            ROBOT_STATE.KUKA.Position = single(pos(1:6));
            ROBOT_STATE.KUKA.Velocity = single([vel, 0, 0, 0]);
        end
    end

    %% -- Periodic S/T refresh --
    if mod(i, st_refresh) == 0
        [sNew, tNew] = readSTFromPosAct(tcp, ROBOT_S, ROBOT_T);
        if sNew ~= ROBOT_S || tNew ~= ROBOT_T
            fprintf('  S/T changed: S=%d->%d, T=%d->%d\n', ROBOT_S, sNew, ROBOT_T, tNew);
            ROBOT_S = sNew; ROBOT_T = tNew;
        end
    end

    %% -- Torque polling from $TORQUE_ACT --
    if mod(i, torque_poll) == 0 || i == 1
        tq = readTorqueFromAxisAct(tcp);
        if ~any(isnan(tq))
            exec_torque(i,:) = tq;
            if ~isempty(ROBOT_STATE)
                ROBOT_STATE.KUKA.Torque = single(tq);
            end
        end
    end

    %% -- Joint guard polling --
    if mod(i, axis_poll) == 0 || i == 1 || i == NUM_POINTS
        [a4_val, a5_val] = readA4A5FromAxisAct(tcp);
        a4_log(i) = a4_val;
        a5_log(i) = a5_val;

        if ~isnan(a4_val) && abs(a4_val) > A4_GUARD_DEG
            fprintf('  WARNING: A4=%.1f exceeds guard!\n', a4_val);
            nonBlockingWait(0.08);
            [a4_chk, ~] = readA4A5FromAxisAct(tcp);
            if ~isnan(a4_chk) && abs(a4_chk) > A4_GUARD_DEG
                fprintf('  A4 STILL HIGH -> ABORTING\n');
                aborted = true; break;
            end
        end

        if ~isnan(a5_val) && abs(a5_val) > A5_GUARD_DEG
            fprintf('  A5=%.1f exceeds guard -> ABORTING\n', a5_val);
            aborted = true; break;
        end
    end

    %% -- Live plot update (non-blocking) --
    if ENABLE_LIVE_PLOT && (mod(i, PLOT_UPDATE_RATE) == 0 || i == NUM_POINTS)
        valid = find(~isnan(exec_pos(:,1)));
        if ~isempty(valid)
            % 3D path
            set(h_exec, 'XData', exec_pos(valid,1), ...
                        'YData', exec_pos(valid,2), ...
                        'ZData', exec_pos(valid,3));
            set(h_cur, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));

            % Position
            addpoints(h_pX, i, pos(1));
            addpoints(h_pY, i, pos(2));
            addpoints(h_pZ, i, pos(3));

            % Orientation
            addpoints(h_oA, i, pos(4));
            addpoints(h_oB, i, pos(5));
            addpoints(h_oC, i, pos(6));

            % Velocity
            if ~any(isnan(exec_vel(i,:)))
                addpoints(h_vX, i, exec_vel(i,1));
                addpoints(h_vY, i, exec_vel(i,2));
                addpoints(h_vZ, i, exec_vel(i,3));
            end
            if ~isnan(exec_speed(i))
                addpoints(h_spd, i, exec_speed(i));
            end

            % Torque (plot most recent non-NaN reading)
            tq_idx = find(~isnan(exec_torque(:,1)), 1, 'last');
            if ~isempty(tq_idx)
                for jj = 1:6
                    addpoints(h_trq(jj), i, exec_torque(tq_idx, jj));
                end
            end

            % Joint guards
            if ~isnan(a4_val), addpoints(h_a4, i, a4_val); end
            if ~isnan(a5_val), addpoints(h_a5, i, a5_val); end

            % Status panel
            elapsed = toc(t_start);
            pct     = i / NUM_POINTS * 100;
            set(h_status, 'String', sprintf( ...
                'Step:     %d / %d\nProgress: %.1f%%\nElapsed:  %.1f s\nS=%d  T=%d\nA4=%.1f  A5=%.1f\n|V|=%.1f mm/s\nTq1=%.2f Nm', ...
                i, NUM_POINTS, pct, elapsed, ROBOT_S, ROBOT_T, ...
                a4_val, a5_val, ...
                exec_speed(i), exec_torque(max(1,tq_idx),1)));

            drawnow limitrate;
        end
    end

    %% -- Precise timing spin-wait --
    while toc(t_step) < CONTROL_DT
    end
end

%% ===== RETURN HOME =====
fprintf('\nReturning home...\n');
sendE6Pos(tcp, key_poses(1,:));
pulseMoveTrigger(tcp);
nonBlockingWait(1.5);
exec_time_total = toc(t_start);

%% ===== POST-RUN SUMMARY PLOTS =====
plotKUKAPostRun(exec_pos, exec_vel, exec_speed, exec_torque, ...
                a4_log, a5_log, exec_time_vec, ...
                A4_GUARD_DEG, A5_GUARD_DEG, CONTROL_DT);

%% ===== SAVE LOG =====
timestamp = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
log_filename = sprintf('execution_log_%s.mat', timestamp);
execution_log = struct( ...
    'exec_pos',        exec_pos, ...
    'exec_vel',        exec_vel, ...
    'exec_speed',      exec_speed, ...
    'exec_torque',     exec_torque, ...
    'a4_log',          a4_log, ...
    'a5_log',          a5_log, ...
    'exec_time_vec',   exec_time_vec, ...
    'key_poses',       key_poses, ...
    'trajectory_file', selected_file, ...
    'execution_time',  exec_time_total, ...
    'success_count',   success_count, ...
    'aborted',         aborted, ...
    'control_dt',      CONTROL_DT, ...
    'timestamp',       timestamp); %#ok<NASGU>
save(log_filename, 'execution_log');

fprintf('\n===========================================================\n');
fprintf('   EXECUTION COMPLETE\n');
fprintf('   Time:     %.1f s\n', exec_time_total);
fprintf('   Success:  %d / %d\n', success_count, NUM_POINTS);
fprintf('   Aborted:  %s\n', mat2str(aborted));
fprintf('   Log:      %s\n', log_filename);
fprintf('===========================================================\n');

%% ===== POST-RUN PLOTTING FUNCTION =====
function plotKUKAPostRun(pos, vel, spd, torque, a4, a5, tvec, ...
                         A4_GUARD, A5_GUARD, dt)

    % Remove rows that were never written
    valid = ~isnan(pos(:,1));
    steps = find(valid);
    if isempty(steps)
        fprintf('[PLOT] No valid data to plot.\n');
        return
    end

    t = tvec(valid);
    if any(isnan(t))
        t = (0:sum(valid)-1).' * dt;
    end

    P  = pos(valid, :);
    V  = vel(valid, :);
    S  = spd(valid);
    TQ = torque(valid, :);
    A4 = a4(valid);
    A5 = a5(valid);

    fig = figure('Name','KUKA Post-Run Analysis', ...
        'Position',[40 40 1700 1000], 'Color','w');
    sgtitle('KUKA Execution — Post-Run Telemetry', 'FontSize',14, 'FontWeight','bold');

    % ---- 1. XYZ Position ----
    subplot(3,3,1);
    plot(t, P(:,1), 'r-', 'LineWidth', 1.8); hold on;
    plot(t, P(:,2), 'g-', 'LineWidth', 1.8);
    plot(t, P(:,3), 'b-', 'LineWidth', 1.8);
    xlabel('Time (s)'); ylabel('mm');
    title('Cartesian Position'); grid on;
    legend('X','Y','Z','Location','best');

    % ---- 2. ABC Orientation ----
    subplot(3,3,2);
    plot(t, P(:,4), 'r-', 'LineWidth', 1.8); hold on;
    plot(t, P(:,5), 'g-', 'LineWidth', 1.8);
    plot(t, P(:,6), 'b-', 'LineWidth', 1.8);
    xlabel('Time (s)'); ylabel('deg');
    title('Cartesian Orientation'); grid on;
    legend('A','B','C','Location','best');

    % ---- 3. 3D Path ----
    subplot(3,3,3);
    scatter3(P(:,1), P(:,2), P(:,3), 18, (1:size(P,1)).', 'filled');
    colormap(gca, jet); colorbar;
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title('3D Executed Path'); grid on; axis equal; view(45,25);

    % ---- 4. Velocity Components ----
    subplot(3,3,4);
    plot(t, V(:,1), 'r-', 'LineWidth', 1.8); hold on;
    plot(t, V(:,2), 'g-', 'LineWidth', 1.8);
    plot(t, V(:,3), 'b-', 'LineWidth', 1.8);
    xlabel('Time (s)'); ylabel('mm/s');
    title('Cartesian Velocity'); grid on;
    legend('Vx','Vy','Vz','Location','best');

    % ---- 5. Speed Magnitude ----
    subplot(3,3,5);
    plot(t, S, 'Color',[0.5 0 0.8], 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('mm/s');
    title('Speed Magnitude |V|'); grid on;

    % ---- 6. Torque — all 6 joints ----
    subplot(3,3,6);
    tqColors = {'r','g','b','m','c',[0.7 0.4 0]};
    tq_valid = ~isnan(TQ(:,1));
    if any(tq_valid)
        for jj = 1:6
            plot(t(tq_valid), TQ(tq_valid, jj), ...
                'Color', tqColors{jj}, 'LineWidth', 1.6, ...
                'DisplayName', sprintf('J%d',jj));
            hold on;
        end
        legend('Location','best');
    else
        text(0.5, 0.5, 'Torque data unavailable', ...
            'HorizontalAlignment','center', 'Units','normalized', ...
            'FontSize', 10, 'Color', [0.6 0.1 0.1]);
    end
    xlabel('Time (s)'); ylabel('Nm');
    title('Joint Torque (\$TORQUE\_ACT)'); grid on;

    % ---- 7. A4/A5 Guard ----
    subplot(3,3,7);
    a4v = A4; a4v(isnan(a4v)) = 0;
    a5v = A5; a5v(isnan(a5v)) = 0;
    plot(t, a4v, 'm-', 'LineWidth', 1.8); hold on;
    plot(t, a5v, 'c-', 'LineWidth', 1.8);
    yline( A4_GUARD, 'r--', 'LineWidth', 1.2);
    yline(-A4_GUARD, 'r--', 'LineWidth', 1.2);
    yline( A5_GUARD, 'b--', 'LineWidth', 1.2);
    yline(-A5_GUARD, 'b--', 'LineWidth', 1.2);
    xlabel('Time (s)'); ylabel('deg');
    title('Joint Guards A4 / A5'); grid on;
    legend('A4','A5','Location','best');

    % ---- 8. Speed histogram ----
    subplot(3,3,8);
    histogram(S(~isnan(S)), 30, 'FaceColor',[0.2 0.5 0.9], 'EdgeColor','none');
    xlabel('Speed (mm/s)'); ylabel('Count');
    title('Speed Distribution'); grid on;

    % ---- 9. Stats table ----
    subplot(3,3,9); axis off;
    tq_v = TQ(~isnan(TQ(:,1)), :);
    if isempty(tq_v), tq_max_str = 'N/A'; tq_mean_str = 'N/A';
    else
        tq_max_str  = sprintf('%.2f Nm', max(abs(tq_v(:))));
        tq_mean_str = sprintf('%.2f Nm', mean(abs(tq_v(:))));
    end
    statsStr = sprintf( ...
        ['Points:      %d\n' ...
         'Duration:    %.2f s\n' ...
         'Mean speed:  %.1f mm/s\n' ...
         'Peak speed:  %.1f mm/s\n' ...
         'X range:     %.0f–%.0f mm\n' ...
         'Y range:     %.0f–%.0f mm\n' ...
         'Z range:     %.0f–%.0f mm\n' ...
         'Peak torque: %s\n' ...
         'Mean torque: %s'], ...
        size(P,1), t(end), mean(S,'omitnan'), max(S,[],'omitnan'), ...
        min(P(:,1)), max(P(:,1)), min(P(:,2)), max(P(:,2)), ...
        min(P(:,3)), max(P(:,3)), tq_max_str, tq_mean_str);
    text(0.05, 0.95, statsStr, ...
        'Units','normalized', 'VerticalAlignment','top', ...
        'FontName','Courier', 'FontSize', 9);
    title('Execution Statistics');

    fprintf('\n[PLOT] Post-run analysis figure generated.\n');
end

%% ===== HELPER FUNCTIONS =====

function ok = sendE6Pos(tcp, pose6)
    posStr = sprintf('{X %.3f,Y %.3f,Z %.3f,A %.3f,B %.3f,C %.3f}', ...
        pose6(1), pose6(2), pose6(3), pose6(4), pose6(5), pose6(6));
    ok = writeVar(tcp, 'target_pos', posStr);
end

function pulseMoveTrigger(tcp)
    writeVar(tcp, 'move_trigger', 'TRUE');
    writeVar(tcp, 'move_trigger', 'FALSE');
end

function nonBlockingWait(seconds)
    t0 = tic;
    while toc(t0) < seconds
        drawnow limitrate;
    end
end

function [S, T] = readSTFromPosAct(tcp, S, T)
    try
        pos_act_str = readVar(tcp, '$POS_ACT');
        if ischar(pos_act_str) || isstring(pos_act_str)
            pos_str = char(pos_act_str);
            s_match = regexp(pos_str, '[,\s]S\s+(\d+)', 'tokens');
            t_match = regexp(pos_str, '[,\s]T\s+(-?\d+)', 'tokens');
            if ~isempty(s_match), S = str2double(s_match{1}{1}); end
            if ~isempty(t_match), T = str2double(t_match{1}{1}); end
        end
    catch
    end
end

function [a4, a5] = readA4A5FromAxisAct(tcp)
    a4 = NaN; a5 = NaN;
    try
        axis_str = char(readVar(tcp, '$AXIS_ACT'));
        a4m = regexp(axis_str, 'A4\s+([-\d.]+)', 'tokens');
        a5m = regexp(axis_str, 'A5\s+([-\d.]+)', 'tokens');
        if ~isempty(a4m), a4 = str2double(a4m{1}{1}); end
        if ~isempty(a5m), a5 = str2double(a5m{1}{1}); end
    catch
    end
end

function tq = readTorqueFromAxisAct(tcp)
% Reads $TORQUE_ACT from KUKAVARPROXY.
% Returns [J1 J2 J3 J4 J5 J6] in Nm, or NaN(1,6) on failure.
    tq = nan(1,6);
    try
        raw = char(readVar(tcp, '$TORQUE_ACT'));
        % Format: {A1 val,A2 val,...,A6 val}
        for jj = 1:6
            tok = regexp(raw, sprintf('A%d\\s+([\\-\\d.]+)', jj), 'tokens');
            if ~isempty(tok)
                tq(jj) = str2double(tok{1}{1});
            end
        end
    catch
    end
end

function success = writeVar(tcp, varName, varValue)
    success = false;
    try
        msgId = uint16(1);   mode = uint8(1);
        vNB = uint8(varName);  vVB = uint8(varValue);
        vNL = uint16(length(vNB));  vVL = uint16(length(vVB));
        cL  = uint16(1 + 2 + length(vNB) + 2 + length(vVB));
        msg = [typecast(swapbytes(msgId),'uint8').' ...
               typecast(swapbytes(cL),  'uint8').' mode ...
               typecast(swapbytes(vNL), 'uint8').' vNB ...
               typecast(swapbytes(vVL), 'uint8').' vVB];
        write(tcp, msg);
        t0 = tic;
        while tcp.NumBytesAvailable == 0 && toc(t0) < 0.05; end
        success = true;
        if tcp.NumBytesAvailable > 0
            r = read(tcp, tcp.NumBytesAvailable);
            if length(r) >= 3, success = all(r(end-2:end) == [0 1 1]); end
        end
    catch; end
end

function value = readVar(tcp, varName)
    value = NaN;
    try
        msgId = uint16(1);  mode = uint8(0);
        vNB = uint8(varName);  vNL = uint16(length(vNB));
        cL  = uint16(1 + 2 + length(vNB));
        msg = [typecast(swapbytes(msgId),'uint8').' ...
               typecast(swapbytes(cL),  'uint8').' mode ...
               typecast(swapbytes(vNL), 'uint8').' vNB];
        write(tcp, msg);
        t0 = tic;
        while tcp.NumBytesAvailable == 0 && toc(t0) < 0.05; end
        if tcp.NumBytesAvailable > 0
            r = read(tcp, tcp.NumBytesAvailable);
            if length(r) > 7
                vLen = double(swapbytes(typecast(r(6:7),'uint16')));
                if length(r) >= 7 + vLen
                    vStr = char(r(8:(7+vLen)));
                    value = str2double(vStr);
                    if isnan(value), value = vStr; end
                end
            end
        end
    catch; end
end

function safeClose(tcp)
    try; if isvalid(tcp), clear tcp; end; fprintf('TCP closed.\n'); catch; end
end

function w = ensureRobotNativeEuler(w)
    if isempty(w), return; end
    if abs(mean(w(:,5))) <= 90, return; end
    for k = 1:size(w,1)
        A_ = w(k,4); B_ = w(k,5); C_ = w(k,6);
        if B_ > 0
            w(k,4) = A_-180; w(k,5) = 180-B_; w(k,6) = C_-180;
        else
            w(k,4) = A_+180; w(k,5) = -180-B_; w(k,6) = C_+180;
        end
    end
    for col = [4, 6]
        for k = 2:size(w,1)
            while (w(k,col)-w(k-1,col)) >  180, w(k,col) = w(k,col)-360; end
            while (w(k,col)-w(k-1,col)) < -180, w(k,col) = w(k,col)+360; end
        end
    end
end