function T = collect_collision_dataset(cfg)
%COLLECT_COLLISION_DATASET Collect synchronized position dataset for collision RL.
%
%   T = collect_collision_dataset(cfg)
%
%   Modes:
%     'synthetic' - No hardware, simulated positions + noise
%     'online'    - Real RoArms + optional KUKA via KUKAVARPROXY
%
%   Config fields:
%     cfg.mode            - 'synthetic' or 'online' (default: 'synthetic')
%     cfg.roarmIps        - {'192.168.1.192', '192.168.1.101'}
%     cfg.roarmDt         - timestep seconds (default: 0.20)
%     cfg.roarmSpeed      - command speed (default: 5)
%     cfg.episodes        - number of episodes (default: 20)
%     cfg.stepsPerEp      - steps per episode (default: 50)
%     cfg.kukaPosition    - [1x3] fixed KUKA TCP (default: [1000,0,300])
%     cfg.connectKuka     - true/false (default: false)
%     cfg.kukaIp          - KUKA IP
%     cfg.kukaPort        - KUKAVARPROXY port
%     cfg.outputDir       - output folder (default: RL\results\)

    if nargin < 1, cfg = struct(); end
    cfg = fillCollectDefaults(cfg);

    fprintf('Collecting collision avoidance dataset (%s mode)...\n', cfg.mode);

    records = [];
    runBase = datestr(now, 'yyyymmdd_HHMMSS');
    stepGlobal = 0;

    % Connect to KUKA if online and requested
    kukaTcp = [];
    if strcmpi(cfg.mode, 'online') && cfg.connectKuka
        try
            kukaTcp = tcpclient(cfg.kukaIp, cfg.kukaPort, 'Timeout', 10);
            fprintf('Connected to KUKA at %s:%d\n', cfg.kukaIp, cfg.kukaPort);
        catch ME
            warning('KUKA connection failed: %s', ME.message);
        end
    end

    prev_r1 = [NaN, NaN, NaN];
    prev_r2 = [NaN, NaN, NaN];

    for ep = 1:cfg.episodes
        run_id = sprintf('%s_ep%02d', runBase, ep);

        % Generate waypoints for both arms
        wp_r1 = generateWP(cfg.stepsPerEp);
        wp_r2 = generateWP(cfg.stepsPerEp);
        wp_r2 = enforceSep(wp_r1, wp_r2, 100);

        prev_r1 = [NaN, NaN, NaN];
        prev_r2 = [NaN, NaN, NaN];

        for k = 1:cfg.stepsPerEp
            stepGlobal = stepGlobal + 1;
            t = (stepGlobal - 1) * cfg.roarmDt;

            target_r1 = wp_r1(k, :);
            target_r2 = wp_r2(k, :);

            if strcmpi(cfg.mode, 'online')
                % Send commands and read real positions
                sendRoarm(cfg.roarmIps{1}, target_r1, cfg.roarmSpeed);
                sendRoarm(cfg.roarmIps{2}, target_r2, cfg.roarmSpeed);
                pause(cfg.roarmDt);
                r1_meas = readRoarm(cfg.roarmIps{1});
                r2_meas = readRoarm(cfg.roarmIps{2});
                kuka_meas = readKuka(kukaTcp, cfg.kukaPosition);
                if any(isnan(r1_meas)), r1_meas = target_r1; end
                if any(isnan(r2_meas)), r2_meas = target_r2; end
            else
                % Synthetic: apply noise
                r1_meas = target_r1 + randn(1,3) .* [1.5, 1.5, 1.0];
                r2_meas = target_r2 + randn(1,3) .* [1.5, 1.5, 1.0];
                kuka_meas = cfg.kukaPosition + randn(1,3) .* [2.0, 2.0, 1.5];
            end

            dists = pairwise_distances(r1_meas, r2_meas, kuka_meas);
            [reward_r1, info_r1] = collision_reward(dists, target_r1, r1_meas, prev_r1, cfg.roarmDt);
            [reward_r2, info_r2] = collision_reward(dists, target_r2, r2_meas, prev_r2, cfg.roarmDt);

            rec.run_id        = string(run_id);
            rec.step_idx      = stepGlobal;
            rec.time_s        = t;
            rec.episode       = ep;
            rec.r1_target_x   = target_r1(1);
            rec.r1_target_y   = target_r1(2);
            rec.r1_target_z   = target_r1(3);
            rec.r1_meas_x     = r1_meas(1);
            rec.r1_meas_y     = r1_meas(2);
            rec.r1_meas_z     = r1_meas(3);
            rec.r2_target_x   = target_r2(1);
            rec.r2_target_y   = target_r2(2);
            rec.r2_target_z   = target_r2(3);
            rec.r2_meas_x     = r2_meas(1);
            rec.r2_meas_y     = r2_meas(2);
            rec.r2_meas_z     = r2_meas(3);
            rec.kuka_x        = kuka_meas(1);
            rec.kuka_y        = kuka_meas(2);
            rec.kuka_z        = kuka_meas(3);
            rec.dist_r1_r2    = dists.r1_r2;
            rec.dist_r1_kuka  = dists.r1_kuka;
            rec.dist_r2_kuka  = dists.r2_kuka;
            rec.dist_min      = dists.min_dist;
            rec.reward_r1     = reward_r1;
            rec.reward_r2     = reward_r2;
            rec.in_collision  = info_r1.in_collision || info_r2.in_collision;
            rec.pos_err_r1    = info_r1.pos_error;
            rec.pos_err_r2    = info_r2.pos_error;
            rec.dt            = cfg.roarmDt;

            records = [records; rec]; %#ok<AGROW>
            prev_r1 = r1_meas;
            prev_r2 = r2_meas;
        end

        collisions = sum([records(end-cfg.stepsPerEp+1:end).in_collision]);
        fprintf('Episode %2d/%d | Collisions: %d\n', ep, cfg.episodes, collisions);
    end

    if ~isempty(kukaTcp) && isvalid(kukaTcp), clear kukaTcp; end

    if isempty(records)
        T = table();
        return;
    end

    T = struct2table(records);
    T = sanitize(T);

    % Save
    if ~isfolder(cfg.outputDir), mkdir(cfg.outputDir); end
    csvPath = fullfile(cfg.outputDir, sprintf('collision_dataset_%s.csv', runBase));
    writetable(T, csvPath);
    fprintf('Dataset saved: %s (%d rows)\n', csvPath, height(T));
end

%% === Helpers ===

function W = generateWP(n)
    anchors = [250,0,220; 300,120,190; 300,-120,190; 210,160,130; 210,-160,130; 280,0,250];
    idx = randi(size(anchors,1),[4,1]); key = anchors(idx,:);
    W = zeros(n,3); segN = max(1,floor(n/(size(key,1)-1))); ptr = 1;
    for s = 1:(size(key,1)-1)
        for i = 1:segN
            if ptr > n, break; end
            W(ptr,:) = (1-(i-1)/max(1,segN-1))*key(s,:) + ((i-1)/max(1,segN-1))*key(s+1,:);
            ptr = ptr+1;
        end
    end
    while ptr<=n, W(ptr,:)=key(end,:); ptr=ptr+1; end
    W(:,1)=min(max(W(:,1),120),340);
    W(:,2)=min(max(W(:,2),-220),220);
    W(:,3)=min(max(W(:,3),80),260);
end

function wp2 = enforceSep(wp1, wp2, minD)
    for i = 1:size(wp1,1)
        d = norm(wp1(i,:)-wp2(i,:));
        if d < minD && d > 0
            wp2(i,:) = wp1(i,:) + (wp2(i,:)-wp1(i,:))/d * minD;
        elseif d == 0
            wp2(i,:) = wp2(i,:) + [minD 0 0];
        end
    end
end

function sendRoarm(ip, xyz, spd)
    cmd = struct('T',104,'x',round(xyz(1)),'y',round(xyz(2)),'z',round(xyz(3)),'t',0,'spd',spd);
    try, webread(['http://' ip '/js?json=' jsonencode(cmd)], weboptions('Timeout',1.2)); catch, end
end

function xyz = readRoarm(ip)
    xyz = [NaN NaN NaN];
    try
        resp = webread(['http://' ip '/js?json=' jsonencode(struct('T',105))], weboptions('Timeout',1.2));
        if isstruct(resp), data=resp; else
            txt=char(string(resp)); s=strfind(txt,'{'); e=strfind(txt,'}');
            if ~isempty(s)&&~isempty(e), data=jsondecode(txt(s(1):e(end))); else, return; end
        end
        if isfield(data,'x'), xyz(1)=double(data.x); end
        if isfield(data,'y'), xyz(2)=double(data.y); end
        if isfield(data,'z'), xyz(3)=double(data.z); end
    catch, end
end

function xyz = readKuka(tcp, default)
    xyz = default;
    if isempty(tcp) || ~isvalid(tcp), return; end
    try
        varName = '$POS_ACT';
        vb = uint8(varName); vl = uint16(length(vb)); cl = uint16(1+2+length(vb));
        msg = [typecast(swapbytes(uint16(1)),'uint8'), typecast(swapbytes(cl),'uint8'), uint8(0), ...
               typecast(swapbytes(vl),'uint8'), vb];
        write(tcp, msg); pause(0.01);
        if tcp.NumBytesAvailable > 0
            r = read(tcp, tcp.NumBytesAvailable);
            if length(r)>7
                vLen = double(swapbytes(typecast(r(6:7),'uint16')));
                if length(r)>=7+vLen
                    s = char(r(8:(7+vLen)));
                    for k = 1:3
                        lab = {'X','Y','Z'}; tok = regexp(s,sprintf('%s\\s+([-\\d.eE]+)',lab{k}),'tokens');
                        if ~isempty(tok), xyz(k)=str2double(tok{1}{1}); end
                    end
                end
            end
        end
    catch, end
end

function T = sanitize(T)
    for i = 1:width(T)
        col = T{:,i};
        if isnumeric(col)
            col = fillmissing(col,'previous');
            col = fillmissing(col,'next');
            col(~isfinite(col)) = 0;
            T{:,i} = col;
        end
    end
end

function cfg = fillCollectDefaults(cfg)
    if ~isfield(cfg,'mode'),          cfg.mode = 'synthetic';                    end
    if ~isfield(cfg,'roarmIps'),      cfg.roarmIps = {'192.168.1.192','192.168.1.101'}; end
    if ~isfield(cfg,'roarmDt'),       cfg.roarmDt = 0.20;                        end
    if ~isfield(cfg,'roarmSpeed'),    cfg.roarmSpeed = 5;                        end
    if ~isfield(cfg,'episodes'),      cfg.episodes = 20;                         end
    if ~isfield(cfg,'stepsPerEp'),    cfg.stepsPerEp = 50;                       end
    if ~isfield(cfg,'kukaPosition'),  cfg.kukaPosition = [1000,0,300];           end
    if ~isfield(cfg,'connectKuka'),   cfg.connectKuka = false;                   end
    if ~isfield(cfg,'kukaIp'),        cfg.kukaIp = '172.31.17.101';              end
    if ~isfield(cfg,'kukaPort'),      cfg.kukaPort = 7000;                       end
    if ~isfield(cfg,'outputDir'),     cfg.outputDir = fullfile(fileparts(mfilename('fullpath')),'results'); end
end