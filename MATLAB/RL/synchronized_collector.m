function dataset = synchronized_collector(cfg, kukaTcp)
%SYNCHRONIZED_COLLECTOR Collect time-aligned position data from all 3 robots.
%
%   dataset = synchronized_collector(cfg, kukaTcp)
%
%   Reads RoArm 1, RoArm 2, and KUKA TCP positions at each timestep,
%   computes pairwise distances and rewards, and returns a table ready
%   for RL training.
%
%   Inputs:
%     cfg     - config struct with fields:
%       .roarmIps       - cell array of 2 RoArm IP strings
%       .roarmDt        - timestep in seconds
%       .roarmEpisodes  - number of episodes
%       .roarmStepsPerEp- steps per episode
%       .roarmSpeed     - RoArm command speed
%       .kukaOffset     - [1x3] KUKA base offset from RoArm frame (mm)
%                         Default: [0, 0, 0] if arms share coordinate frame
%     kukaTcp - tcpclient object connected to KUKAVARPROXY (can be [])
%
%   Output:
%     dataset - table with columns for all 3 robot positions, distances,
%               rewards, and RL state/action features

    if nargin < 2, kukaTcp = []; end

    if ~isfield(cfg, 'kukaOffset')
        cfg.kukaOffset = [0, 0, 0];
    end

    records = [];
    runBase = datestr(now, 'yyyymmdd_HHMMSS');
    totalSteps = cfg.roarmEpisodes * cfg.roarmStepsPerEp;

    % Preallocate previous positions for velocity computation
    prev_r1 = [NaN, NaN, NaN];
    prev_r2 = [NaN, NaN, NaN];

    stepGlobal = 0;

    for ep = 1:cfg.roarmEpisodes
        run_id = sprintf('%s_ep%02d', runBase, ep);

        % Generate target waypoints for both arms
        wp_r1 = generateRoarmWaypoints(cfg.roarmStepsPerEp);
        wp_r2 = generateRoarmWaypoints(cfg.roarmStepsPerEp);

        % Ensure starting waypoints are not too close
        wp_r2 = enforceSeparation(wp_r1, wp_r2, 100);

        for k = 1:cfg.roarmStepsPerEp
            stepGlobal = stepGlobal + 1;
            t = (stepGlobal - 1) * cfg.roarmDt;

            target_r1 = wp_r1(k, :);
            target_r2 = wp_r2(k, :);

            % ---- Send commands to both RoArms ----
            sendRoarmMove(cfg.roarmIps{1}, target_r1, cfg.roarmSpeed);
            sendRoarmMove(cfg.roarmIps{2}, target_r2, cfg.roarmSpeed);

            pause(cfg.roarmDt);

            % ---- Read all positions at the same instant ----
            r1_meas = readRoarmXYZ(cfg.roarmIps{1});
            r2_meas = readRoarmXYZ(cfg.roarmIps{2});
            kuka_meas = readKukaPosition(kukaTcp, cfg.kukaOffset);

            % Replace NaN with target (fallback)
            if any(isnan(r1_meas)), r1_meas = target_r1; end
            if any(isnan(r2_meas)), r2_meas = target_r2; end

            % ---- Compute pairwise distances ----
            dists = pairwise_distances(r1_meas, r2_meas, kuka_meas);

            % ---- Compute rewards for each arm ----
            [reward_r1, info_r1] = collision_reward(dists, target_r1, r1_meas, prev_r1, cfg.roarmDt);
            [reward_r2, info_r2] = collision_reward(dists, target_r2, r2_meas, prev_r2, cfg.roarmDt);

            % ---- Build RL states ----
            state_r1 = build_rl_state('roarm_1', r1_meas, r2_meas, kuka_meas, ...
                                       target_r1, prev_r1, cfg.roarmDt, dists);
            state_r2 = build_rl_state('roarm_2', r1_meas, r2_meas, kuka_meas, ...
                                       target_r2, prev_r2, cfg.roarmDt, dists);

            % ---- Store record ----
            rec = struct();
            rec.run_id       = string(run_id);
            rec.step_idx     = stepGlobal;
            rec.time_s       = t;
            rec.episode      = ep;

            % RoArm 1
            rec.r1_target_x  = target_r1(1);
            rec.r1_target_y  = target_r1(2);
            rec.r1_target_z  = target_r1(3);
            rec.r1_meas_x    = r1_meas(1);
            rec.r1_meas_y    = r1_meas(2);
            rec.r1_meas_z    = r1_meas(3);

            % RoArm 2
            rec.r2_target_x  = target_r2(1);
            rec.r2_target_y  = target_r2(2);
            rec.r2_target_z  = target_r2(3);
            rec.r2_meas_x    = r2_meas(1);
            rec.r2_meas_y    = r2_meas(2);
            rec.r2_meas_z    = r2_meas(3);

            % KUKA TCP
            rec.kuka_x       = kuka_meas(1);
            rec.kuka_y       = kuka_meas(2);
            rec.kuka_z       = kuka_meas(3);

            % Distances
            rec.dist_r1_r2   = dists.r1_r2;
            rec.dist_r1_kuka = dists.r1_kuka;
            rec.dist_r2_kuka = dists.r2_kuka;
            rec.dist_min     = dists.min_dist;

            % Rewards
            rec.reward_r1    = reward_r1;
            rec.reward_r2    = reward_r2;
            rec.in_collision = info_r1.in_collision || info_r2.in_collision;

            % Position errors
            rec.pos_err_r1   = info_r1.pos_error;
            rec.pos_err_r2   = info_r2.pos_error;

            rec.dt           = cfg.roarmDt;

            records = [records; rec]; %#ok<AGROW>

            % Update previous positions
            prev_r1 = r1_meas;
            prev_r2 = r2_meas;
        end

        % Reset previous positions between episodes
        prev_r1 = [NaN, NaN, NaN];
        prev_r2 = [NaN, NaN, NaN];

        fprintf('Episode %d/%d complete. Collisions: %d\n', ...
            ep, cfg.roarmEpisodes, sum([records.in_collision]));
    end

    if isempty(records)
        dataset = table();
        return;
    end

    dataset = struct2table(records);
    dataset = sanitizeTable(dataset);

    % Save to CSV
    outDir = fullfile(fileparts(mfilename('fullpath')), '..', 'datasets');
    if ~isfolder(outDir), mkdir(outDir); end
    outFile = fullfile(outDir, sprintf('collision_avoidance_dataset_%s.csv', runBase));
    writetable(dataset, outFile);
    fprintf('Saved synchronized dataset: %s (%d rows)\n', outFile, height(dataset));
end

%% =====================================================================
%% HELPER FUNCTIONS
%% =====================================================================

function xyz = readKukaPosition(kukaTcp, offset)
    % Read KUKA TCP position from $POS_ACT via KUKAVARPROXY.
    % Returns position in the shared coordinate frame.
    xyz = [NaN, NaN, NaN];

    if isempty(kukaTcp) || ~isvalid(kukaTcp)
        % No KUKA connection — use a fixed known position.
        % KUKA TCP is ~1m away from RoArm workspace.
        xyz = [1000, 0, 300] + offset;
        return;
    end

    try
        value = readVarKuka(kukaTcp, '$POS_ACT');
        if ischar(value) || isstring(value)
            s = char(value);
            labels = {'X', 'Y', 'Z'};
            for k = 1:3
                tok = regexp(s, sprintf('%s\\s+([-\\d.eE]+)', labels{k}), 'tokens');
                if ~isempty(tok)
                    xyz(k) = str2double(tok{1}{1});
                end
            end
        end
    catch
        % Keep NaN — will be handled by caller
    end

    % Apply coordinate frame offset
    if all(isfinite(xyz))
        xyz = xyz + offset;
    end
end

function value = readVarKuka(tcp, varName)
    % KUKAVARPROXY read variable (same protocol as kuka_pick_and_place_gui)
    value = NaN;
    try
        msgId = uint16(1);
        mode  = uint8(0);
        varNameBytes = uint8(varName);
        varNameLen   = uint16(length(varNameBytes));
        contentLen   = uint16(1 + 2 + length(varNameBytes));

        msgIdBytes      = typecast(swapbytes(msgId),      'uint8');
        contentLenBytes = typecast(swapbytes(contentLen), 'uint8');
        varNameLenBytes = typecast(swapbytes(varNameLen), 'uint8');

        msg = [msgIdBytes(:).' contentLenBytes(:).' mode ...
               varNameLenBytes(:).' varNameBytes];

        write(tcp, msg);
        pause(0.01);

        if tcp.NumBytesAvailable > 0
            response = read(tcp, tcp.NumBytesAvailable);
            if length(response) > 7
                valueLen = double(swapbytes(typecast(response(6:7), 'uint16')));
                if length(response) >= 7 + valueLen
                    valueBytes = response(8:(7 + valueLen));
                    valueStr = char(valueBytes);
                    vnum = str2double(valueStr);
                    if isnan(vnum), value = valueStr; else, value = vnum; end
                end
            end
        end
    catch
        value = NaN;
    end
end

function sendRoarmMove(ip, xyz, spd)
    cmd = struct('T', 104, 'x', round(xyz(1)), 'y', round(xyz(2)), ...
                 'z', round(xyz(3)), 't', 0, 'spd', spd);
    url = ['http://' ip '/js?json=' jsonencode(cmd)];
    try
        opts = weboptions('Timeout', 1.2);
        webread(url, opts);
    catch
    end
end

function xyz = readRoarmXYZ(ip)
    xyz = [NaN, NaN, NaN];
    cmd = struct('T', 105);
    url = ['http://' ip '/js?json=' jsonencode(cmd)];
    try
        opts = weboptions('Timeout', 1.2);
        resp = webread(url, opts);
        if isstruct(resp)
            data = resp;
        else
            txt = char(string(resp));
            s = strfind(txt, '{'); e = strfind(txt, '}');
            if ~isempty(s) && ~isempty(e)
                data = jsondecode(txt(s(1):e(end)));
            else
                return;
            end
        end
        if isfield(data, 'x'), xyz(1) = double(data.x); end
        if isfield(data, 'y'), xyz(2) = double(data.y); end
        if isfield(data, 'z'), xyz(3) = double(data.z); end
    catch
    end
end

function W = generateRoarmWaypoints(n)
    xRange = [120, 340]; yRange = [-220, 220]; zRange = [80, 260];
    anchors = [250,0,220; 300,120,190; 300,-120,190; 210,160,130; 210,-160,130; 280,0,250];
    idx = randi(size(anchors,1), [4,1]);
    key = anchors(idx,:);
    W = zeros(n,3);
    segN = max(1, floor(n / (size(key,1)-1)));
    ptr = 1;
    for s = 1:(size(key,1)-1)
        p0 = key(s,:); p1 = key(s+1,:);
        for i = 1:segN
            if ptr > n, break; end
            alpha = (i-1)/max(1, segN-1);
            W(ptr,:) = (1-alpha)*p0 + alpha*p1;
            ptr = ptr + 1;
        end
    end
    while ptr <= n, W(ptr,:) = key(end,:); ptr = ptr+1; end
    W(:,1) = min(max(W(:,1), xRange(1)), xRange(2));
    W(:,2) = min(max(W(:,2), yRange(1)), yRange(2));
    W(:,3) = min(max(W(:,3), zRange(1)), zRange(2));
end

function wp2 = enforceSeparation(wp1, wp2, minDist)
    % Push wp2 waypoints away from wp1 if they are too close.
    for i = 1:size(wp1, 1)
        d = norm(wp1(i,:) - wp2(i,:));
        if d < minDist && d > 0
            direction = (wp2(i,:) - wp1(i,:)) / d;
            wp2(i,:) = wp1(i,:) + direction * minDist;
        elseif d == 0
            wp2(i,:) = wp2(i,:) + [minDist, 0, 0];
        end
    end
end

function T = sanitizeTable(T)
    vars = T.Properties.VariableNames;
    for i = 1:numel(vars)
        col = T.(vars{i});
        if isnumeric(col)
            col = fillmissing(col, 'previous');
            col = fillmissing(col, 'next');
            col(~isfinite(col)) = 0;
            T.(vars{i}) = col;
        end
    end
end