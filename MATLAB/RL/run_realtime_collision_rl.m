function run_realtime_collision_rl(cfg)
%RUN_REALTIME_COLLISION_RL Main entry: real-time collision avoidance RL loop.
%
%   run_realtime_collision_rl(cfg)
%
%   This is the top-level script that:
%     1. Connects to KUKA (KUKAVARPROXY) and both RoArms
%     2. Optionally loads a pre-trained policy or trains from scratch
%     3. Runs the real-time collection + training + action loop
%     4. Saves the dataset and trained policy at the end
%
%   Config fields:
%     cfg.roarmIps         - {'192.168.1.192', '192.168.1.101'}
%     cfg.roarmDt          - timestep seconds (default: 0.20)
%     cfg.roarmSpeed       - RoArm command speed (default: 5)
%     cfg.roarmEpisodes    - number of episodes (default: 20)
%     cfg.roarmStepsPerEp  - steps per episode (default: 40)
%     cfg.kukaIp           - KUKA IP (default: '172.31.17.101')
%     cfg.kukaPort         - KUKAVARPROXY port (default: 7000)
%     cfg.kukaOffset       - [1x3] coordinate frame offset (default: [1000,0,0])
%     cfg.connectKuka      - true/false (default: false for standalone RoArm)
%     cfg.policyPath       - path to pre-trained policy .mat (optional)
%     cfg.trainOnline      - true/false - train during collection (default: true)
%     cfg.trainInterval    - train every N steps (default: 5)
%     cfg.batchSize        - training batch size (default: 64)
%     cfg.bufferCapacity   - replay buffer size (default: 50000)
%     cfg.actionLimit      - max position adjustment mm (default: 30)
%     cfg.explorationNoise - exploration noise std mm (default: 10)
%     cfg.outputDir        - output directory

    if nargin < 1, cfg = struct(); end
    cfg = fillRuntimeDefaults(cfg);

    fprintf('\n========================================\n');
    fprintf(' REAL-TIME COLLISION AVOIDANCE RL\n');
    fprintf('========================================\n');
    fprintf('RoArm 1:  %s\n', cfg.roarmIps{1});
    fprintf('RoArm 2:  %s\n', cfg.roarmIps{2});
    fprintf('KUKA:     %s (connect=%d)\n', cfg.kukaIp, cfg.connectKuka);
    fprintf('Episodes: %d x %d steps @ %.2fs\n', cfg.roarmEpisodes, cfg.roarmStepsPerEp, cfg.roarmDt);
    fprintf('Online training: %d (interval=%d, batch=%d)\n', cfg.trainOnline, cfg.trainInterval, cfg.batchSize);
    fprintf('========================================\n\n');

    if ~isfolder(cfg.outputDir), mkdir(cfg.outputDir); end

    % === Connect to KUKA (optional) ===
    kukaTcp = [];
    if cfg.connectKuka
        try
            kukaTcp = tcpclient(cfg.kukaIp, cfg.kukaPort, 'Timeout', 10);
            fprintf('Connected to KUKA at %s:%d\n', cfg.kukaIp, cfg.kukaPort);
        catch ME
            warning('Could not connect to KUKA: %s\nUsing fixed KUKA position.', ME.message);
        end
    else
        fprintf('KUKA connection disabled. Using fixed position [%s] + offset.\n', ...
            num2str([1000, 0, 300]));
    end

    % === Initialize or load policy ===
    STATE_DIM  = 18;
    ACTION_DIM = 3;

    if ~isempty(cfg.policyPath) && isfile(cfg.policyPath)
        fprintf('Loading pre-trained policy: %s\n', cfg.policyPath);
        loaded = load(cfg.policyPath, 'policy');
        agent = loaded.policy.agent;
        fprintf('Policy loaded.\n');
    else
        fprintf('Training new policy from scratch.\n');
        trainCfg = struct();
        trainCfg.state_dim       = STATE_DIM;
        trainCfg.action_dim      = ACTION_DIM;
        trainCfg.action_limit    = cfg.actionLimit;
        trainCfg.buffer_capacity = cfg.bufferCapacity;
        trainCfg.batch_size      = cfg.batchSize;
        trainCfg.save_path       = '';  % don't save yet
        policy = train_collision_policy(trainCfg);
        agent = policy.agent;
    end

    % === Replay buffer for online training ===
    buf = replay_buffer(cfg.bufferCapacity, STATE_DIM, ACTION_DIM);

    % === Preallocate logging ===
    runBase = datestr(now, 'yyyymmdd_HHMMSS');
    allRecords = [];
    totalCollisions = 0;
    totalStepsGlobal = 0;

    prev_r1 = [NaN, NaN, NaN];
    prev_r2 = [NaN, NaN, NaN];
    prev_state_r1 = [];
    prev_state_r2 = [];
    prev_action_r1 = [];
    prev_action_r2 = [];
    prev_reward_r1 = 0;
    prev_reward_r2 = 0;

    fprintf('\nStarting real-time loop...\n');
    tTotalStart = tic;

    for ep = 1:cfg.roarmEpisodes
        run_id = sprintf('%s_ep%02d', runBase, ep);
        epCollisions = 0;

        % Generate base waypoints for both arms this episode
        wp_r1 = generateRoarmWaypoints_rt(cfg.roarmStepsPerEp);
        wp_r2 = generateRoarmWaypoints_rt(cfg.roarmStepsPerEp);

        % Enforce minimum starting separation
        for wi = 1:size(wp_r1, 1)
            d = norm(wp_r1(wi,:) - wp_r2(wi,:));
            if d < 100 && d > 0
                dir = (wp_r2(wi,:) - wp_r1(wi,:)) / d;
                wp_r2(wi,:) = wp_r1(wi,:) + dir * 100;
            elseif d == 0
                wp_r2(wi,:) = wp_r2(wi,:) + [100, 0, 0];
            end
        end

        prev_r1 = [NaN, NaN, NaN];
        prev_r2 = [NaN, NaN, NaN];
        prev_state_r1 = [];
        prev_state_r2 = [];

        for k = 1:cfg.roarmStepsPerEp
            totalStepsGlobal = totalStepsGlobal + 1;
            tStep = tic;

            target_r1_base = wp_r1(k, :);
            target_r2_base = wp_r2(k, :);

            % ---- READ current positions ----
            r1_meas = readRoarmXYZ_rt(cfg.roarmIps{1});
            r2_meas = readRoarmXYZ_rt(cfg.roarmIps{2});
            kuka_meas = readKukaXYZ_rt(kukaTcp, cfg.kukaOffset);

            if any(isnan(r1_meas)), r1_meas = target_r1_base; end
            if any(isnan(r2_meas)), r2_meas = target_r2_base; end

            % ---- COMPUTE distances ----
            dists = pairwise_distances(r1_meas, r2_meas, kuka_meas);

            % ---- BUILD RL states ----
            state_r1 = build_rl_state('roarm_1', r1_meas, r2_meas, kuka_meas, ...
                                       target_r1_base, prev_r1, cfg.roarmDt, dists);
            state_r2 = build_rl_state('roarm_2', r1_meas, r2_meas, kuka_meas, ...
                                       target_r2_base, prev_r2, cfg.roarmDt, dists);

            % ---- GET ACTIONS from policy ----
            action_r1 = getAction(agent, {state_r1(:)});
            action_r1 = action_r1{1}(:).';
            action_r2 = getAction(agent, {state_r2(:)});
            action_r2 = action_r2{1}(:).';

            % Add exploration noise during training
            if cfg.trainOnline
                action_r1 = action_r1 + randn(1,3) * cfg.explorationNoise;
                action_r2 = action_r2 + randn(1,3) * cfg.explorationNoise;
            end

            % Clamp actions
            action_r1 = min(max(action_r1, -cfg.actionLimit), cfg.actionLimit);
            action_r2 = min(max(action_r2, -cfg.actionLimit), cfg.actionLimit);

            % ---- APPLY adjusted targets ----
            adjusted_r1 = target_r1_base + action_r1;
            adjusted_r2 = target_r2_base + action_r2;

            % Clamp to RoArm workspace
            adjusted_r1 = clampToWorkspace(adjusted_r1);
            adjusted_r2 = clampToWorkspace(adjusted_r2);

            % ---- SEND commands ----
            sendRoarmMove_rt(cfg.roarmIps{1}, adjusted_r1, cfg.roarmSpeed);
            sendRoarmMove_rt(cfg.roarmIps{2}, adjusted_r2, cfg.roarmSpeed);

            % ---- COMPUTE rewards ----
            [reward_r1, info_r1] = collision_reward(dists, target_r1_base, r1_meas, prev_r1, cfg.roarmDt);
            [reward_r2, info_r2] = collision_reward(dists, target_r2_base, r2_meas, prev_r2, cfg.roarmDt);

            if info_r1.in_collision || info_r2.in_collision
                epCollisions = epCollisions + 1;
                totalCollisions = totalCollisions + 1;
            end

            % ---- STORE experience in replay buffer ----
            done = (k == cfg.roarmStepsPerEp);

            if ~isempty(prev_state_r1)
                buf.add(prev_state_r1, prev_action_r1, prev_reward_r1, state_r1, done);
                buf.add(prev_state_r2, prev_action_r2, prev_reward_r2, state_r2, done);
            end

            prev_state_r1  = state_r1;
            prev_state_r2  = state_r2;
            prev_action_r1 = action_r1;
            prev_action_r2 = action_r2;
            prev_reward_r1 = reward_r1;
            prev_reward_r2 = reward_r2;

            % ---- ONLINE TRAINING ----
            if cfg.trainOnline && mod(totalStepsGlobal, cfg.trainInterval) == 0 ...
                    && buf.size() >= cfg.batchSize
                [s_b, a_b, r_b, ns_b, d_b] = buf.sample(cfg.batchSize);

                % Feed batch as experiences to the agent
                for bi = 1:cfg.batchSize
                    exp.Observation     = {s_b(bi,:).'};
                    exp.Action          = {a_b(bi,:).'};
                    exp.Reward          = r_b(bi);
                    exp.NextObservation = {ns_b(bi,:).'};
                    exp.IsDone          = d_b(bi) > 0.5;
                    try
                        learn(agent, exp);
                    catch
                        % Silently continue if learn fails
                    end
                end
            end

            % ---- LOG record ----
            rec = struct();
            rec.run_id       = string(run_id);
            rec.step_idx     = totalStepsGlobal;
            rec.time_s       = (totalStepsGlobal - 1) * cfg.roarmDt;
            rec.episode      = ep;
            rec.r1_target_x  = target_r1_base(1);
            rec.r1_target_y  = target_r1_base(2);
            rec.r1_target_z  = target_r1_base(3);
            rec.r1_adjusted_x = adjusted_r1(1);
            rec.r1_adjusted_y = adjusted_r1(2);
            rec.r1_adjusted_z = adjusted_r1(3);
            rec.r1_meas_x    = r1_meas(1);
            rec.r1_meas_y    = r1_meas(2);
            rec.r1_meas_z    = r1_meas(3);
            rec.r2_target_x  = target_r2_base(1);
            rec.r2_target_y  = target_r2_base(2);
            rec.r2_target_z  = target_r2_base(3);
            rec.r2_adjusted_x = adjusted_r2(1);
            rec.r2_adjusted_y = adjusted_r2(2);
            rec.r2_adjusted_z = adjusted_r2(3);
            rec.r2_meas_x    = r2_meas(1);
            rec.r2_meas_y    = r2_meas(2);
            rec.r2_meas_z    = r2_meas(3);
            rec.kuka_x       = kuka_meas(1);
            rec.kuka_y       = kuka_meas(2);
            rec.kuka_z       = kuka_meas(3);
            rec.dist_r1_r2   = dists.r1_r2;
            rec.dist_r1_kuka = dists.r1_kuka;
            rec.dist_r2_kuka = dists.r2_kuka;
            rec.dist_min     = dists.min_dist;
            rec.reward_r1    = reward_r1;
            rec.reward_r2    = reward_r2;
            rec.action_r1_x  = action_r1(1);
            rec.action_r1_y  = action_r1(2);
            rec.action_r1_z  = action_r1(3);
            rec.action_r2_x  = action_r2(1);
            rec.action_r2_y  = action_r2(2);
            rec.action_r2_z  = action_r2(3);
            rec.in_collision = info_r1.in_collision || info_r2.in_collision;
            rec.pos_err_r1   = info_r1.pos_error;
            rec.pos_err_r2   = info_r2.pos_error;
            rec.dt           = cfg.roarmDt;

            allRecords = [allRecords; rec]; %#ok<AGROW>

            % Update previous positions
            prev_r1 = r1_meas;
            prev_r2 = r2_meas;

            % ---- Maintain loop timing ----
            elapsed = toc(tStep);
            remaining = cfg.roarmDt - elapsed;
            if remaining > 0
                pause(remaining);
            end
        end

        fprintf('Episode %2d/%d | Collisions: %d | Buffer: %d | Min dist: %.1f mm\n', ...
            ep, cfg.roarmEpisodes, epCollisions, buf.size(), ...
            min([allRecords(end-cfg.roarmStepsPerEp+1:end).dist_min]));
    end

    totalTime = toc(tTotalStart);

    fprintf('\n========================================\n');
    fprintf(' COMPLETE\n');
    fprintf('========================================\n');
    fprintf('Total time:       %.1f s\n', totalTime);
    fprintf('Total steps:      %d\n', totalStepsGlobal);
    fprintf('Total collisions: %d\n', totalCollisions);
    fprintf('Collision rate:   %.2f%%\n', 100 * totalCollisions / totalStepsGlobal);
    fprintf('Buffer size:      %d\n', buf.size());

    % === SAVE EVERYTHING ===

    % 1. Dataset CSV
    if ~isempty(allRecords)
        T = struct2table(allRecords);
        T = sanitizeTableRT(T);
        csvPath = fullfile(cfg.outputDir, sprintf('collision_rl_dataset_%s.csv', runBase));
        writetable(T, csvPath);
        fprintf('Dataset saved: %s\n', csvPath);
    end

    % 2. Replay buffer
    bufPath = fullfile(cfg.outputDir, sprintf('replay_buffer_%s.mat', runBase));
    buf.save_to_file(bufPath);
    fprintf('Replay buffer saved: %s\n', bufPath);

    % 3. Trained policy
    policyOut = struct();
    policyOut.agent     = agent;
    policyOut.cfg       = cfg;
    policyOut.timestamp = runBase;
    policyOut.totalCollisions = totalCollisions;
    policyOut.totalSteps = totalStepsGlobal;

    policyPath = fullfile(cfg.outputDir, sprintf('collision_policy_%s.mat', runBase));
    save(policyPath, 'policyOut');
    fprintf('Policy saved: %s\n', policyPath);

    % 4. Execution summary
    summary = struct();
    summary.totalTime       = totalTime;
    summary.totalSteps      = totalStepsGlobal;
    summary.totalCollisions = totalCollisions;
    summary.collisionRate   = totalCollisions / totalStepsGlobal;
    summary.episodes        = cfg.roarmEpisodes;
    summary.stepsPerEp      = cfg.roarmStepsPerEp;
    summary.bufferFinal     = buf.size();
    summaryPath = fullfile(cfg.outputDir, sprintf('run_summary_%s.mat', runBase));
    save(summaryPath, 'summary');
    fprintf('Summary saved: %s\n', summaryPath);

    % Cleanup KUKA connection
    if ~isempty(kukaTcp) && isvalid(kukaTcp)
        clear kukaTcp;
    end
end

%% =====================================================================
%% RUNTIME HELPER FUNCTIONS
%% =====================================================================

function xyz = clampToWorkspace(xyz)
    xyz(1) = min(max(xyz(1), 120), 340);
    xyz(2) = min(max(xyz(2), -220), 220);
    xyz(3) = min(max(xyz(3), 80), 260);
end

function xyz = readRoarmXYZ_rt(ip)
    xyz = [NaN, NaN, NaN];
    cmd = struct('T', 105);
    url = ['http://' ip '/js?json=' jsonencode(cmd)];
    try
        opts = weboptions('Timeout', 1.0);
        resp = webread(url, opts);
        if isstruct(resp), data = resp;
        else
            txt = char(string(resp));
            s = strfind(txt, '{'); e = strfind(txt, '}');
            if ~isempty(s) && ~isempty(e)
                data = jsondecode(txt(s(1):e(end)));
            else, return;
            end
        end
        if isfield(data,'x'), xyz(1) = double(data.x); end
        if isfield(data,'y'), xyz(2) = double(data.y); end
        if isfield(data,'z'), xyz(3) = double(data.z); end
    catch
    end
end

function sendRoarmMove_rt(ip, xyz, spd)
    cmd = struct('T', 104, 'x', round(xyz(1)), 'y', round(xyz(2)), ...
                 'z', round(xyz(3)), 't', 0, 'spd', spd);
    url = ['http://' ip '/js?json=' jsonencode(cmd)];
    try
        opts = weboptions('Timeout', 1.0);
        webread(url, opts);
    catch
    end
end

function xyz = readKukaXYZ_rt(kukaTcp, offset)
    xyz = [1000, 0, 300] + offset;  % default fixed position
    if isempty(kukaTcp) || ~isvalid(kukaTcp), return; end
    try
        msgId = uint16(1); mode = uint8(0);
        varName = '$POS_ACT';
        varNameBytes = uint8(varName);
        varNameLen   = uint16(length(varNameBytes));
        contentLen   = uint16(1 + 2 + length(varNameBytes));
        msg = [typecast(swapbytes(msgId),'uint8'), typecast(swapbytes(contentLen),'uint8'), mode, ...
               typecast(swapbytes(varNameLen),'uint8'), varNameBytes];
        write(kukaTcp, msg);
        pause(0.01);
        if kukaTcp.NumBytesAvailable > 0
            response = read(kukaTcp, kukaTcp.NumBytesAvailable);
            if length(response) > 7
                valueLen = double(swapbytes(typecast(response(6:7),'uint16')));
                if length(response) >= 7 + valueLen
                    s = char(response(8:(7+valueLen)));
                    labels = {'X','Y','Z'};
                    for k = 1:3
                        tok = regexp(s, sprintf('%s\\s+([-\\d.eE]+)', labels{k}), 'tokens');
                        if ~isempty(tok), xyz(k) = str2double(tok{1}{1}) + offset(k); end
                    end
                end
            end
        end
    catch
    end
end

function W = generateRoarmWaypoints_rt(n)
    xRange = [120, 340]; yRange = [-220, 220]; zRange = [80, 260];
    anchors = [250,0,220; 300,120,190; 300,-120,190; 210,160,130; 210,-160,130; 280,0,250];
    idx = randi(size(anchors,1), [4,1]);
    key = anchors(idx,:);
    W = zeros(n,3);
    segN = max(1, floor(n/(size(key,1)-1)));
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
    while ptr <= n, W(ptr,:) = key(end,:); ptr = ptr + 1; end
    W(:,1) = min(max(W(:,1), xRange(1)), xRange(2));
    W(:,2) = min(max(W(:,2), yRange(1)), yRange(2));
    W(:,3) = min(max(W(:,3), zRange(1)), zRange(2));
end

function T = sanitizeTableRT(T)
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

function cfg = fillRuntimeDefaults(cfg)
    if ~isfield(cfg, 'roarmIps'),        cfg.roarmIps = {'192.168.1.192', '192.168.1.101'}; end
    if ~isfield(cfg, 'roarmDt'),         cfg.roarmDt = 0.20;        end
    if ~isfield(cfg, 'roarmSpeed'),      cfg.roarmSpeed = 5.0;      end
    if ~isfield(cfg, 'roarmEpisodes'),   cfg.roarmEpisodes = 20;    end
    if ~isfield(cfg, 'roarmStepsPerEp'), cfg.roarmStepsPerEp = 40;  end
    if ~isfield(cfg, 'kukaIp'),          cfg.kukaIp = '172.31.17.101'; end
    if ~isfield(cfg, 'kukaPort'),        cfg.kukaPort = 7000;       end
    if ~isfield(cfg, 'kukaOffset'),      cfg.kukaOffset = [1000, 0, 0]; end
    if ~isfield(cfg, 'connectKuka'),     cfg.connectKuka = false;   end
    if ~isfield(cfg, 'policyPath'),      cfg.policyPath = '';       end
    if ~isfield(cfg, 'trainOnline'),     cfg.trainOnline = true;    end
    if ~isfield(cfg, 'trainInterval'),   cfg.trainInterval = 5;     end
    if ~isfield(cfg, 'batchSize'),       cfg.batchSize = 64;        end
    if ~isfield(cfg, 'bufferCapacity'),  cfg.bufferCapacity = 50000;end
    if ~isfield(cfg, 'actionLimit'),     cfg.actionLimit = 30;      end
    if ~isfield(cfg, 'explorationNoise'),cfg.explorationNoise = 10; end
    if ~isfield(cfg, 'outputDir'),       cfg.outputDir = fullfile(fileparts(mfilename('fullpath')), '..', 'datasets'); end
end