%% =========================================================================
%  Main.m — MULTIROBOT ADAPTIVE CONTROL ENTRY POINT
%  =========================================================================
%  The pipeline function lives ONE LEVEL UP at MATLAB/run_full_multirobot_pipeline.m
%  This file is a convenience launcher from inside the RL folder.
%
%  Uncomment ONE mode below and press Run (F5).

% Ensure the parent MATLAB/ folder is on the path
addpath(fullfile(fileparts(mfilename('fullpath')), '..'));

%% ====== MODE 1: FULL SYNTHETIC (DEFAULT — NO HARDWARE) =================
run_full_multirobot_pipeline();

%% ====== MODE 2: FULL ONLINE / HARDWARE =================================
% cfg = struct();
% cfg.mode     = 'online';
% cfg.kukaIp   = '172.31.17.101';
% cfg.kukaPort = 7000;
% cfg.roarmIps = {'192.168.1.192', '192.168.1.101'};
% run_full_multirobot_pipeline(cfg);

%% ====== MODE 3: KUKA ONLY ==============================================
% cfg = struct();
% cfg.collectData = false; cfg.trainKuka = true;
% cfg.trainRoarm = false;  cfg.trainCollision = false;
% run_full_multirobot_pipeline(cfg);

%% ====== MODE 4: ROARM ONLY =============================================
% cfg = struct();
% cfg.collectData = true;  cfg.trainKuka = false;
% cfg.trainRoarm = true;   cfg.trainCollision = false;
% run_full_multirobot_pipeline(cfg);

%% ====== MODE 5: COLLISION AVOIDANCE ONLY ================================
% cfg = struct();
% cfg.collectData = false; cfg.trainKuka = false;
% cfg.trainRoarm = false;  cfg.trainCollision = true;
% run_full_multirobot_pipeline(cfg);

%% ====== MODE 6: DATA COLLECTION ONLY ===================================
% cfg = struct();
% cfg.collectData = true;  cfg.trainKuka = false;
% cfg.trainRoarm = false;  cfg.trainCollision = false;
% run_full_multirobot_pipeline(cfg);

%% ====== MODE 7: TRAIN ALL (SKIP COLLECTION) ============================
% cfg = struct();
% cfg.collectData = false; cfg.trainKuka = true;
% cfg.trainRoarm = true;   cfg.trainCollision = true;
% run_full_multirobot_pipeline(cfg);