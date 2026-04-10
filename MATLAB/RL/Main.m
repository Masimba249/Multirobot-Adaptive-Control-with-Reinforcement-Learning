%% ========== COLLISION AVOIDANCE MODE ==========
% Uncomment the section below to run collision avoidance RL
% instead of the standard pick-and-place / spray training.
%
% This trains the two RoArm robots to avoid each other and the KUKA TCP
% using only position data (no torque/current sensors needed).

% --- Quick synthetic test (no hardware) ---
% run_collision_avoidance_rl

% --- Train only ---
% cfg = struct();
% cfg.maxEpisodes = 500;
% cfg.maxSteps = 50;
% cfg.kukaPosition = [1000, 0, 300];
% train_collision_agent(cfg);

% --- Evaluate a saved agent ---
% evaluate_collision_agent('savedAgents/collision_agent_XXXXXXXX_XXXXXX.mat');

% --- Collect real data from hardware ---
% cfg = struct();
% cfg.mode = 'online';
% cfg.connectKuka = true;
% cfg.kukaIp = '172.31.17.101';
% collect_collision_dataset(cfg);