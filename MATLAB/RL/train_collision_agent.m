function agent = train_collision_agent(cfg)
%TRAIN_COLLISION_AGENT Train DDPG agent for collision avoidance.
%
%   agent = train_collision_agent(cfg)
%
%   Uses CollisionAvoidanceEnv and trains a DDPG policy.
%   Trained agent is saved to savedAgents\ folder.
%
%   Config fields:
%     cfg.maxEpisodes     - max training episodes (default: 500)
%     cfg.maxSteps        - max steps per episode (default: 50)
%     cfg.dt              - timestep seconds (default: 0.20)
%     cfg.actionLimit     - max action mm (default: 30)
%     cfg.hiddenSizes     - [128, 128]
%     cfg.actorLR         - actor learning rate (default: 1e-4)
%     cfg.criticLR        - critic learning rate (default: 1e-3)
%     cfg.gamma           - discount factor (default: 0.99)
%     cfg.bufferLength    - replay buffer size (default: 100000)
%     cfg.miniBatchSize   - batch size (default: 64)
%     cfg.kukaPosition    - [1x3] KUKA TCP position in shared frame

    if nargin < 1, cfg = struct(); end
    cfg = fillDefaults(cfg);

    fprintf('\n========================================\n');
    fprintf(' COLLISION AVOIDANCE — DDPG TRAINING\n');
    fprintf('========================================\n');
    fprintf('Max episodes: %d\n', cfg.maxEpisodes);
    fprintf('Max steps:    %d\n', cfg.maxSteps);
    fprintf('Action limit: %.0f mm\n', cfg.actionLimit);
    fprintf('========================================\n\n');

    % === Create environment ===
    env = CollisionAvoidanceEnv();
    env.MaxSteps = cfg.maxSteps;
    env.Dt = cfg.dt;
    env.ActionLimit = cfg.actionLimit;
    env.KukaPosition = cfg.kukaPosition;

    obsInfo = getObservationInfo(env);
    actInfo = getActionInfo(env);

    % === Actor network ===
    stateDim = obsInfo.Dimension(1);   % 18
    actionDim = actInfo.Dimension(1);  % 3

    actorNet = [
        featureInputLayer(stateDim, 'Name', 'state', 'Normalization', 'none')
        fullyConnectedLayer(cfg.hiddenSizes(1), 'Name', 'fc1')
        reluLayer('Name', 'relu1')
        fullyConnectedLayer(cfg.hiddenSizes(2), 'Name', 'fc2')
        reluLayer('Name', 'relu2')
        fullyConnectedLayer(actionDim, 'Name', 'fc_out')
        tanhLayer('Name', 'tanh')
        scalingLayer('Name', 'scale', 'Scale', cfg.actionLimit)
    ];

    actor = rlContinuousDeterministicActor(actorNet, obsInfo, actInfo);

    % === Critic network ===
    statePath = [
        featureInputLayer(stateDim, 'Name', 'state_in', 'Normalization', 'none')
        fullyConnectedLayer(cfg.hiddenSizes(1), 'Name', 'sfc1')
        reluLayer('Name', 'srelu1')
    ];

    actionPath = [
        featureInputLayer(actionDim, 'Name', 'action_in', 'Normalization', 'none')
        fullyConnectedLayer(cfg.hiddenSizes(1), 'Name', 'afc1')
        reluLayer('Name', 'arelu1')
    ];

    commonPath = [
        additionLayer(2, 'Name', 'add')
        fullyConnectedLayer(cfg.hiddenSizes(2), 'Name', 'cfc2')
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
    agentOpts.SampleTime = cfg.dt;
    agentOpts.DiscountFactor = cfg.gamma;
    agentOpts.MiniBatchSize = cfg.miniBatchSize;
    agentOpts.ExperienceBufferLength = cfg.bufferLength;
    agentOpts.TargetSmoothFactor = 1e-3;
    agentOpts.ActorOptimizerOptions  = rlOptimizerOptions('LearnRate', cfg.actorLR);
    agentOpts.CriticOptimizerOptions = rlOptimizerOptions('LearnRate', cfg.criticLR);

    % Exploration noise (Ornstein-Uhlenbeck)
    agentOpts.NoiseOptions.StandardDeviation     = 10 * ones(actionDim, 1);
    agentOpts.NoiseOptions.StandardDeviationMin   = 1 * ones(actionDim, 1);
    agentOpts.NoiseOptions.StandardDeviationDecayRate = 1e-4;

    agent = rlDDPGAgent(actor, critic, agentOpts);
    fprintf('DDPG agent created.\n');

    % === Training options ===
    trainOpts = rlTrainingOptions();
    trainOpts.MaxEpisodes = cfg.maxEpisodes;
    trainOpts.MaxStepsPerEpisode = cfg.maxSteps;
    trainOpts.StopTrainingCriteria = 'AverageReward';
    trainOpts.StopTrainingValue = -10;            % converge when avg reward > -10
    trainOpts.ScoreAveragingWindowLength = 50;
    trainOpts.Verbose = true;
    trainOpts.Plots = 'training-progress';

    % Save best agent to savedAgents folder
    saveDir = fullfile(fileparts(mfilename('fullpath')), 'savedAgents');
    if ~isfolder(saveDir), mkdir(saveDir); end
    trainOpts.SaveAgentCriteria = 'EpisodeReward';
    trainOpts.SaveAgentValue = -20;
    trainOpts.SaveAgentDirectory = saveDir;

    % === TRAIN ===
    fprintf('Starting training...\n');
    trainingStats = train(agent, env, trainOpts);

    % === Save final agent ===
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    savePath = fullfile(saveDir, sprintf('collision_agent_%s.mat', timestamp));
    save(savePath, 'agent', 'trainingStats', 'cfg');
    fprintf('Agent saved: %s\n', savePath);

    % === Save training stats to results ===
    resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');
    if ~isfolder(resultsDir), mkdir(resultsDir); end
    statsPath = fullfile(resultsDir, sprintf('collision_training_stats_%s.mat', timestamp));
    save(statsPath, 'trainingStats', 'cfg');
    fprintf('Training stats saved: %s\n', statsPath);
end

function cfg = fillDefaults(cfg)
    if ~isfield(cfg, 'maxEpisodes'),   cfg.maxEpisodes = 500;           end
    if ~isfield(cfg, 'maxSteps'),      cfg.maxSteps = 50;               end
    if ~isfield(cfg, 'dt'),            cfg.dt = 0.20;                   end
    if ~isfield(cfg, 'actionLimit'),   cfg.actionLimit = 30;            end
    if ~isfield(cfg, 'hiddenSizes'),   cfg.hiddenSizes = [128, 128];    end
    if ~isfield(cfg, 'actorLR'),       cfg.actorLR = 1e-4;              end
    if ~isfield(cfg, 'criticLR'),      cfg.criticLR = 1e-3;             end
    if ~isfield(cfg, 'gamma'),         cfg.gamma = 0.99;                end
    if ~isfield(cfg, 'bufferLength'),  cfg.bufferLength = 100000;       end
    if ~isfield(cfg, 'miniBatchSize'), cfg.miniBatchSize = 64;          end
    if ~isfield(cfg, 'kukaPosition'),  cfg.kukaPosition = [1000, 0, 300]; end
end