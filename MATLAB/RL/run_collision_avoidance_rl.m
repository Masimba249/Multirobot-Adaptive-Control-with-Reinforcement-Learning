%% RUN_COLLISION_AVOIDANCE_RL — Main Entry Point
%
% This script orchestrates the full pipeline:
%   1. Collect dataset (synthetic or online)
%   2. Train DDPG collision avoidance agent
%   3. Evaluate and plot results
%
% Run this file directly or call sections independently.

%% ========== CONFIGURATION ==========
cfg = struct();

% --- Data collection ---
cfg.collect.mode        = 'synthetic';    % 'synthetic' or 'online'
cfg.collect.episodes    = 20;
cfg.collect.stepsPerEp  = 50;
cfg.collect.roarmDt     = 0.20;
cfg.collect.roarmIps    = {'192.168.1.192', '192.168.1.101'};
cfg.collect.roarmSpeed  = 5;
cfg.collect.kukaPosition = [1000, 0, 300]; % KUKA ~1m away
cfg.collect.connectKuka = false;           % set true when KUKA is online

% --- Training ---
cfg.train.maxEpisodes   = 500;
cfg.train.maxSteps      = 50;
cfg.train.dt            = 0.20;
cfg.train.actionLimit   = 30;          % mm max position adjustment
cfg.train.hiddenSizes   = [128, 128];
cfg.train.actorLR       = 1e-4;
cfg.train.criticLR      = 1e-3;
cfg.train.gamma         = 0.99;
cfg.train.bufferLength  = 100000;
cfg.train.miniBatchSize = 64;
cfg.train.kukaPosition  = [1000, 0, 300];

% --- Evaluation ---
cfg.eval.numEpisodes    = 100;
cfg.eval.maxSteps       = 50;

%% ========== STEP 1: COLLECT DATASET ==========
fprintf('\n>>> STEP 1: Collecting collision avoidance dataset...\n\n');

dataset = collect_collision_dataset(cfg.collect);
fprintf('Dataset: %d rows collected.\n\n', height(dataset));

%% ========== STEP 2: TRAIN AGENT ==========
fprintf('\n>>> STEP 2: Training collision avoidance DDPG agent...\n\n');

agent = train_collision_agent(cfg.train);
fprintf('Training complete.\n\n');

%% ========== STEP 3: EVALUATE ==========
fprintf('\n>>> STEP 3: Evaluating trained agent...\n\n');

% Find the latest saved agent
savedDir = fullfile(fileparts(mfilename('fullpath')), 'savedAgents');
agentFiles = dir(fullfile(savedDir, 'collision_agent_*.mat'));
if ~isempty(agentFiles)
    [~, latest] = max([agentFiles.datenum]);
    agentPath = fullfile(savedDir, agentFiles(latest).name);
    fprintf('Evaluating: %s\n', agentPath);
    evaluate_collision_agent(agentPath, cfg.eval);
else
    warning('No saved collision agent found. Skipping evaluation.');
end

fprintf('\n>>> PIPELINE COMPLETE.\n');