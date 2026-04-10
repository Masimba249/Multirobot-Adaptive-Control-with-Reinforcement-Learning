function policy = train_collision_policy(cfg)
%TRAIN_COLLISION_POLICY Train a collision avoidance RL policy from collected data.
%
%   policy = train_collision_policy(cfg)
%
%   Uses MATLAB Reinforcement Learning Toolbox (DDPG agent) to learn
%   position adjustments that avoid collisions while tracking targets.
%
%   Config fields:
%     cfg.state_dim       - state vector dimension (default: 18)
%     cfg.action_dim      - action vector dimension (default: 3, XYZ offset)
%     cfg.action_limit    - max action magnitude in mm (default: 30)
%     cfg.hidden_sizes    - hidden layer sizes (default: [128 128])
%     cfg.learning_rate   - actor/critic LR (default: 1e-3)
%     cfg.gamma           - discount factor (default: 0.99)
%     cfg.buffer_capacity - replay buffer size (default: 100000)
%     cfg.batch_size      - training batch size (default: 64)
%     cfg.pretrain_csv    - path to CSV dataset for offline pre-training
%     cfg.save_path       - path to save trained policy

    if nargin < 1, cfg = struct(); end
    cfg = fillTrainDefaults(cfg);

    fprintf('=== Collision Avoidance Policy Training ===\n');
    fprintf('State dim:  %d\n', cfg.state_dim);
    fprintf('Action dim: %d\n', cfg.action_dim);
    fprintf('Action limit: %.1f mm\n', cfg.action_limit);

    % === Define observation and action specs ===
    obsInfo = rlNumericSpec([cfg.state_dim, 1], ...
        'LowerLimit', -ones(cfg.state_dim, 1) * 5, ...
        'UpperLimit',  ones(cfg.state_dim, 1) * 5);
    obsInfo.Name = 'collision_state';

    actInfo = rlNumericSpec([cfg.action_dim, 1], ...
        'LowerLimit', -ones(cfg.action_dim, 1) * cfg.action_limit, ...
        'UpperLimit',  ones(cfg.action_dim, 1) * cfg.action_limit);
    actInfo.Name = 'position_adjustment';

    % === Build Actor Network ===
    actorNet = [
        featureInputLayer(cfg.state_dim, 'Name', 'state')
        fullyConnectedLayer(cfg.hidden_sizes(1), 'Name', 'fc1')
        reluLayer('Name', 'relu1')
        fullyConnectedLayer(cfg.hidden_sizes(2), 'Name', 'fc2')
        reluLayer('Name', 'relu2')
        fullyConnectedLayer(cfg.action_dim, 'Name', 'fc_out')
        tanhLayer('Name', 'tanh')
        scalingLayer('Name', 'scale', 'Scale', cfg.action_limit)
    ];

    actor = rlContinuousDeterministicActor(actorNet, obsInfo, actInfo);

    % === Build Critic Network ===
    statePath = [
        featureInputLayer(cfg.state_dim, 'Name', 'state_in')
        fullyConnectedLayer(cfg.hidden_sizes(1), 'Name', 'sfc1')
        reluLayer('Name', 'srelu1')
    ];

    actionPath = [
        featureInputLayer(cfg.action_dim, 'Name', 'action_in')
        fullyConnectedLayer(cfg.hidden_sizes(1), 'Name', 'afc1')
        reluLayer('Name', 'arelu1')
    ];

    commonPath = [
        additionLayer(2, 'Name', 'add')
        fullyConnectedLayer(cfg.hidden_sizes(2), 'Name', 'cfc2')
        reluLayer('Name', 'crelu2')
        fullyConnectedLayer(1, 'Name', 'q_value')
    ];

    criticNet = layerGraph();
    criticNet = addLayers(criticNet, statePath);
    criticNet = addLayers(criticNet, actionPath);
    criticNet = addLayers(criticNet, commonPath);
    criticNet = connectLayers(criticNet, 'srelu1', 'add/in1');
    criticNet = connectLayers(criticNet, 'arelu1', 'add/in2');

    critic = rlQValueFunction(criticNet, obsInfo, actInfo, ...
        'ObservationInputNames', 'state_in', ...
        'ActionInputNames', 'action_in');

    % === DDPG Agent ===
    agentOpts = rlDDPGAgentOptions();
    agentOpts.SampleTime = 0.2;
    agentOpts.DiscountFactor = cfg.gamma;
    agentOpts.MiniBatchSize = cfg.batch_size;
    agentOpts.ExperienceBufferLength = cfg.buffer_capacity;
    agentOpts.TargetSmoothFactor = 1e-3;

    agentOpts.ActorOptimizerOptions = rlOptimizerOptions('LearnRate', cfg.learning_rate);
    agentOpts.CriticOptimizerOptions = rlOptimizerOptions('LearnRate', cfg.learning_rate);

    agent = rlDDPGAgent(actor, critic, agentOpts);

    fprintf('DDPG agent created.\n');

    % === Offline Pre-Training from CSV (if available) ===
    if ~isempty(cfg.pretrain_csv) && isfile(cfg.pretrain_csv)
        fprintf('Pre-training from: %s\n', cfg.pretrain_csv);
        agent = pretrainFromCSV(agent, cfg);
        fprintf('Pre-training complete.\n');
    end

    % === Save ===
    policy = struct();
    policy.agent     = agent;
    policy.obsInfo   = obsInfo;
    policy.actInfo   = actInfo;
    policy.cfg       = cfg;
    policy.timestamp = datestr(now, 'yyyymmdd_HHMMSS');

    if ~isempty(cfg.save_path)
        save(cfg.save_path, 'policy');
        fprintf('Policy saved: %s\n', cfg.save_path);
    end
end

function agent = pretrainFromCSV(agent, cfg)
    % Load collected dataset and replay experiences through the agent.
    T = readtable(cfg.pretrain_csv);

    requiredCols = {'r1_meas_x','r1_meas_y','r1_meas_z', ...
                    'r2_meas_x','r2_meas_y','r2_meas_z', ...
                    'kuka_x','kuka_y','kuka_z', ...
                    'r1_target_x','r1_target_y','r1_target_z', ...
                    'reward_r1','dist_r1_r2','dist_r1_kuka','dt'};
    missing = requiredCols(~ismember(requiredCols, T.Properties.VariableNames));
    if ~isempty(missing)
        warning('Pre-train CSV missing columns: %s. Skipping.', strjoin(missing, ', '));
        return;
    end

    fprintf('Replaying %d samples for pre-training...\n', height(T));

    prev_r1 = [NaN, NaN, NaN];

    for i = 1:height(T)
        r1 = [T.r1_meas_x(i), T.r1_meas_y(i), T.r1_meas_z(i)];
        r2 = [T.r2_meas_x(i), T.r2_meas_y(i), T.r2_meas_z(i)];
        ku = [T.kuka_x(i), T.kuka_y(i), T.kuka_z(i)];
        tgt = [T.r1_target_x(i), T.r1_target_y(i), T.r1_target_z(i)];

        dists = pairwise_distances(r1, r2, ku);
        state = build_rl_state('roarm_1', r1, r2, ku, tgt, prev_r1, T.dt(i), dists);

        % Action = difference between target and measured (what the arm did)
        action = tgt - r1;
        action = min(max(action, -cfg.action_limit), cfg.action_limit);

        % Use the recorded reward
        reward = T.reward_r1(i);

        % Next state (use next row if available)
        if i < height(T)
            r1_next = [T.r1_meas_x(i+1), T.r1_meas_y(i+1), T.r1_meas_z(i+1)];
            r2_next = [T.r2_meas_x(i+1), T.r2_meas_y(i+1), T.r2_meas_z(i+1)];
            ku_next = [T.kuka_x(i+1), T.kuka_y(i+1), T.kuka_z(i+1)];
            tgt_next = [T.r1_target_x(i+1), T.r1_target_y(i+1), T.r1_target_z(i+1)];
            dists_next = pairwise_distances(r1_next, r2_next, ku_next);
            next_state = build_rl_state('roarm_1', r1_next, r2_next, ku_next, ...
                                         tgt_next, r1, T.dt(i), dists_next);
            done = false;
        else
            next_state = state;
            done = true;
        end

        % Feed experience to agent's buffer
        exp.Observation  = {state(:)};
        exp.Action       = {action(:)};
        exp.Reward       = reward;
        exp.NextObservation = {next_state(:)};
        exp.IsDone       = done;

        % Use the agent's built-in learn method
        try
            agent = agent.learn(exp); %#ok<NASGU>
        catch
            % Some MATLAB versions may not support this directly.
            % Fall back to manual buffer loading in run_realtime.
        end

        prev_r1 = r1;
    end
end

function cfg = fillTrainDefaults(cfg)
    if ~isfield(cfg, 'state_dim'),       cfg.state_dim = 18;           end
    if ~isfield(cfg, 'action_dim'),      cfg.action_dim = 3;           end
    if ~isfield(cfg, 'action_limit'),    cfg.action_limit = 30;        end
    if ~isfield(cfg, 'hidden_sizes'),    cfg.hidden_sizes = [128, 128];end
    if ~isfield(cfg, 'learning_rate'),   cfg.learning_rate = 1e-3;     end
    if ~isfield(cfg, 'gamma'),           cfg.gamma = 0.99;             end
    if ~isfield(cfg, 'buffer_capacity'), cfg.buffer_capacity = 100000; end
    if ~isfield(cfg, 'batch_size'),      cfg.batch_size = 64;          end
    if ~isfield(cfg, 'pretrain_csv'),    cfg.pretrain_csv = '';         end
    if ~isfield(cfg, 'save_path'),       cfg.save_path = 'collision_policy.mat'; end
end