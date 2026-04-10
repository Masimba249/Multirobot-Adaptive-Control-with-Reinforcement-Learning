function results = run_full_multirobot_pipeline(cfg)
%RUN_FULL_MULTIROBOT_PIPELINE  Single entry point for the entire system.
%
%   results = run_full_multirobot_pipeline()       % synthetic (no hardware)
%   results = run_full_multirobot_pipeline(cfg)     % custom config
%
%   Phases:
%     0. Setup — paths, toolbox checks, directory creation
%     1. Data Collection — position datasets + synchronized 3-robot data
%     2. KUKA RL — pick-and-place velocity optimization (TD3)
%     3. RoArm RL — adaptive painting speed (TD3)
%     4. Collision Avoidance RL — DDPG inter-robot collision avoidance
%     5. Summary — metrics table, saved results .mat
%
%   Every phase is wrapped in try/catch so one failure never blocks the rest.
%
%   Modes:
%     'synthetic' — no hardware required; uses synthetic/simulated data
%     'online'    — connects to real KUKA (KUKAVARPROXY) + RoArm-M2 (WiFi)

    if nargin < 1, cfg = struct(); end
    cfg = fillPipelineDefaults(cfg);

    %% ═══════════════════════════════════════════════════════════════════
    %  PHASE 0 — SETUP
    %  ═══════════════════════════════════════════════════════════════════
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════╗\n');
    fprintf('║  MULTIROBOT ADAPTIVE CONTROL — FULL PIPELINE                ║\n');
    fprintf('║  KUKA KR10 + 2× RoArm-M2-S + Reinforcement Learning        ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════╝\n');
    fprintf('  Mode: %s\n\n', upper(cfg.mode));

    tPipelineStart = tic;

    % --- Resolve paths ---
    scriptDir = fileparts(mfilename('fullpath'));
    if isempty(scriptDir), scriptDir = pwd; end

    projectRoot = fullfile(scriptDir, '..', '..');
    matlabRoot  = fullfile(scriptDir, '..');

    pathsToAdd = {
        scriptDir, ...                                          % MATLAB/RL
        fullfile(matlabRoot, 'KUKA_control'), ...               % MATLAB/KUKA_control
        fullfile(matlabRoot, 'KUKA_control', 'srcs'), ...       % MATLAB/KUKA_control/srcs
        fullfile(matlabRoot, 'Roarm_control'), ...              % MATLAB/Roarm_control
    };
    for p = 1:numel(pathsToAdd)
        if isfolder(pathsToAdd{p})
            addpath(genpath(pathsToAdd{p}));
        end
    end
    fprintf('[Phase 0] Paths configured. scriptDir = %s\n', scriptDir);

    % --- Output directories ---
    savedAgentsDir = fullfile(scriptDir, 'savedAgents');
    figuresDir     = fullfile(scriptDir, 'figures');
    datasetsDir    = fullfile(scriptDir, '..', 'datasets');
    for d = {savedAgentsDir, figuresDir, datasetsDir}
        if ~isfolder(d{1}), mkdir(d{1}); end
    end

    % --- Results struct ---
    R = struct();
    R.timestamp   = datestr(now, 'yyyymmdd_HHMMSS');
    R.mode        = cfg.mode;
    R.phases      = struct();
    R.phases.setup     = 'PASS';
    R.phases.collect   = 'SKIPPED';
    R.phases.kukaRL    = 'SKIPPED';
    R.phases.roarmRL   = 'SKIPPED';
    R.phases.collision = 'SKIPPED';

    %% ═══════════════════════════════════════════════════════════════════
    %  PHASE 1 — DATA COLLECTION
    %  ═══════════════════════════════════════════════════════════════════
    if cfg.collectData
        fprintf('\n[Phase 1] DATA COLLECTION\n');
        fprintf('─────────────────────────\n');

        try
            % 1a. Position-only datasets (KUKA + RoArm)
            dataCfg = cfg.datasetCfg;
            dataCfg.outputDir = scriptDir;  % put CSVs in MATLAB/RL/

            if strcmpi(cfg.mode, 'online')
                dataCfg.roarmMode = 'online';
            else
                dataCfg.roarmMode = 'synthetic';
            end

            if exist('create_position_only_rl_dataset', 'file')
                fprintf('  Running create_position_only_rl_dataset ...\n');
                dataOut = create_position_only_rl_dataset(dataCfg);
                R.dataCollection.kukaRows  = dataOut.kukaRows;
                R.dataCollection.roarmRows = dataOut.roarmRows;
                R.dataCollection.totalRows = dataOut.totalRows;
                fprintf('  Position dataset: %d KUKA + %d RoArm = %d total rows\n', ...
                    dataOut.kukaRows, dataOut.roarmRows, dataOut.totalRows);
            else
                fprintf('  create_position_only_rl_dataset not found — skipping.\n');
                fprintf('  (Using existing CSV files in MATLAB/RL/ if available.)\n');
            end

            % 1b. Synchronized 3-robot data (online mode with KUKA)
            if strcmpi(cfg.mode, 'online') && cfg.connectKuka
                fprintf('  Connecting to KUKA at %s:%d ...\n', cfg.kukaIp, cfg.kukaPort);
                kukaTcp = tcpclient(cfg.kukaIp, cfg.kukaPort, 'Timeout', 10);
                syncCfg = cfg.syncCfg;
                syncData = synchronized_collector(syncCfg, kukaTcp);
                R.dataCollection.syncRows = height(syncData);
                fprintf('  Synchronized dataset: %d rows\n', height(syncData));
                clear kukaTcp;
            end

            R.phases.collect = 'PASS';

        catch ME
            R.phases.collect = 'FAIL';
            R.collectError = ME.message;
            fprintf('  ✗ Data collection FAILED: %s\n', ME.message);
        end
    else
        fprintf('\n[Phase 1] Data collection SKIPPED (cfg.collectData = false)\n');
    end

    %% ═══════════════════════════════════════════════════════════════════
    %  PHASE 2 — KUKA PICK-AND-PLACE RL
    %  ═══════════════════════════════════════════════════════════════════
    if cfg.trainKuka
        fprintf('\n[Phase 2] KUKA PICK-AND-PLACE RL TRAINING\n');
        fprintf('──────────────────────────────────────────\n');

        kukaCSV = fullfile(scriptDir, 'kuka_rl_dataset.csv');
        if ~isfile(kukaCSV)
            fprintf('  ✗ kuka_rl_dataset.csv not found at: %s\n', kukaCSV);
            fprintf('    Run the KUKA GUI to collect data first, or ensure the file exists.\n');
            R.phases.kukaRL = 'FAIL';
            R.kukaError = 'kuka_rl_dataset.csv not found';
        else
            try
                fprintf('  Dataset found: %s\n', kukaCSV);
                fprintf('  Launching runKukaPickPlaceRL (isolated scope) ...\n');

                % Run in isolated function scope so 'clear' inside the
                % script only clears that scope, not ours.
                runKukaRL_isolated(scriptDir);

                R.phases.kukaRL = 'PASS';
                fprintf('  ✓ KUKA RL training completed.\n');

            catch ME
                R.phases.kukaRL = 'FAIL';
                R.kukaError = ME.message;
                fprintf('  ✗ KUKA RL FAILED: %s\n', ME.message);
                if ~isempty(ME.stack)
                    fprintf('    at line %d in %s\n', ME.stack(1).line, ME.stack(1).name);
                end
            end
        end
    else
        fprintf('\n[Phase 2] KUKA RL SKIPPED (cfg.trainKuka = false)\n');
    end

    %% ═══════════════════════════════════════════════════════════════════
    %  PHASE 3 — ROARM PAINTING RL
    %  ═══════════════════════════════════════════════════════════════════
    if cfg.trainRoarm
        fprintf('\n[Phase 3] ROARM PAINTING RL TRAINING\n');
        fprintf('─────────────────────────────────────\n');

        roarmCSV = fullfile(scriptDir, 'roarm_position_rl_dataset.csv');
        if ~isfile(roarmCSV)
            fprintf('  ✗ roarm_position_rl_dataset.csv not found at: %s\n', roarmCSV);
            fprintf('    Enable cfg.collectData or ensure the file exists.\n');
            R.phases.roarmRL = 'FAIL';
            R.roarmError = 'roarm_position_rl_dataset.csv not found';
        else
            try
                fprintf('  Dataset found: %s\n', roarmCSV);
                fprintf('  Launching runRoarmPaintingRL (isolated scope) ...\n');

                runRoarmRL_isolated(scriptDir);

                R.phases.roarmRL = 'PASS';
                fprintf('  ✓ RoArm RL training completed.\n');

            catch ME
                R.phases.roarmRL = 'FAIL';
                R.roarmError = ME.message;
                fprintf('  ✗ RoArm RL FAILED: %s\n', ME.message);
                if ~isempty(ME.stack)
                    fprintf('    at line %d in %s\n', ME.stack(1).line, ME.stack(1).name);
                end
            end
        end
    else
        fprintf('\n[Phase 3] RoArm RL SKIPPED (cfg.trainRoarm = false)\n');
    end

    %% ═══════════════════════════════════════════════════════════════════
    %  PHASE 4 — COLLISION AVOIDANCE RL
    %  ═══════════════════════════════════════════════════════════════════
    if cfg.trainCollision
        fprintf('\n[Phase 4] COLLISION AVOIDANCE RL\n');
        fprintf('────────────────────────────────\n');

        try
            % 4a. Collect collision dataset
            colCfg = cfg.collisionCfg.collect;
            if strcmpi(cfg.mode, 'online')
                colCfg.mode = 'online';
            else
                colCfg.mode = 'synthetic';
            end

            fprintf('  Collecting collision avoidance dataset (%s) ...\n', colCfg.mode);
            colDataset = collect_collision_dataset(colCfg);
            R.collisionCollection = height(colDataset);
            fprintf('  Collision dataset: %d rows\n', height(colDataset));

            % 4b. Train collision agent
            fprintf('  Training collision avoidance DDPG agent ...\n');
            trainCfg = cfg.collisionCfg.train;
            colAgent = train_collision_agent(trainCfg);
            fprintf('  Training complete.\n');

            % 4c. Evaluate
            agentFiles = dir(fullfile(savedAgentsDir, 'collision_agent_*.mat'));
            if ~isempty(agentFiles)
                [~, latest] = max([agentFiles.datenum]);
                agentPath = fullfile(savedAgentsDir, agentFiles(latest).name);
                fprintf('  Evaluating: %s\n', agentPath);
                evalCfg = cfg.collisionCfg.eval;
                evaluate_collision_agent(agentPath, evalCfg);
                fprintf('  Evaluation complete.\n');
            else
                fprintf('  No saved collision agent found for evaluation.\n');
            end

            R.phases.collision = 'PASS';
            fprintf('  ✓ Collision avoidance RL completed.\n');

        catch ME
            R.phases.collision = 'FAIL';
            R.collisionError = ME.message;
            fprintf('  ✗ Collision RL FAILED: %s\n', ME.message);
            if ~isempty(ME.stack)
                fprintf('    at line %d in %s\n', ME.stack(1).line, ME.stack(1).name);
            end
        end
    else
        fprintf('\n[Phase 4] Collision RL SKIPPED (cfg.trainCollision = false)\n');
    end

    %% ═══════════════════════════════════════════════════════════════════
    %  PHASE 5 — SUMMARY
    %  ═══════════════════════════════════════════════════════════════════
    elapsed = toc(tPipelineStart);
    R.totalTime_s = elapsed;

    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════╗\n');
    fprintf('║  PIPELINE SUMMARY                                           ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════╝\n');
    fprintf('  Mode:          %s\n', upper(cfg.mode));
    fprintf('  Total time:    %.1f seconds (%.1f min)\n', elapsed, elapsed/60);
    fprintf('\n');
    fprintf('  %-30s %s\n', 'Phase', 'Status');
    fprintf('  %s\n', repmat('─', 1, 45));
    fprintf('  %-30s %s\n', '0. Setup & Paths',        R.phases.setup);
    fprintf('  %-30s %s\n', '1. Data Collection',      R.phases.collect);
    fprintf('  %-30s %s\n', '2. KUKA Pick-Place RL',   R.phases.kukaRL);
    fprintf('  %-30s %s\n', '3. RoArm Painting RL',    R.phases.roarmRL);
    fprintf('  %-30s %s\n', '4. Collision Avoidance',   R.phases.collision);
    fprintf('\n');

    % Count passes
    statuses = {R.phases.setup, R.phases.collect, R.phases.kukaRL, ...
                R.phases.roarmRL, R.phases.collision};
    nPass = sum(strcmp(statuses, 'PASS'));
    nFail = sum(strcmp(statuses, 'FAIL'));
    nSkip = sum(strcmp(statuses, 'SKIPPED'));
    fprintf('  PASS: %d | FAIL: %d | SKIPPED: %d\n', nPass, nFail, nSkip);

    % Save results
    resultsFile = fullfile(scriptDir, sprintf('pipeline_results_%s.mat', R.timestamp));
    pipeline_results = R; %#ok<NASGU>
    save(resultsFile, 'pipeline_results');
    fprintf('  Results saved: %s\n\n', resultsFile);

    if nFail > 0
        fprintf('  ⚠ Some phases failed. Check error messages above.\n\n');
    else
        fprintf('  ✓ Pipeline completed successfully.\n\n');
    end

    results = R;
end

%% ═════════════════════════════════════════════════════════════════════════
%  ISOLATED WRAPPER FUNCTIONS
%  These exist so that 'clear' inside the RL scripts only clears the
%  wrapper's function scope — not the pipeline's workspace.
%% ═════════════════════════════════════════════════════════════════════════

function runKukaRL_isolated(scriptDir)
    oldDir = pwd;
    cd(scriptDir);
    try
        runKukaPickPlaceRL;
    catch ME
        cd(oldDir);
        rethrow(ME);
    end
    cd(oldDir);
end

function runRoarmRL_isolated(scriptDir)
    oldDir = pwd;
    cd(scriptDir);
    try
        runRoarmPaintingRL;
    catch ME
        cd(oldDir);
        rethrow(ME);
    end
    cd(oldDir);
end

%% ═════════════════════════════════════════════════════════════════════════
%  DEFAULT CONFIGURATION
%% ═════════════════════════════════════════════════════════════════════════

function cfg = fillPipelineDefaults(cfg)

    % --- Top-level flags ---
    if ~isfield(cfg, 'mode'),           cfg.mode           = 'synthetic'; end
    if ~isfield(cfg, 'collectData'),    cfg.collectData    = true;        end
    if ~isfield(cfg, 'trainKuka'),      cfg.trainKuka      = true;        end
    if ~isfield(cfg, 'trainRoarm'),     cfg.trainRoarm     = true;        end
    if ~isfield(cfg, 'trainCollision'), cfg.trainCollision = true;        end
    if ~isfield(cfg, 'connectKuka'),    cfg.connectKuka    = false;       end

    % --- Hardware IPs ---
    if ~isfield(cfg, 'kukaIp'),   cfg.kukaIp   = '172.31.17.101';                   end
    if ~isfield(cfg, 'kukaPort'), cfg.kukaPort = 7000;                               end
    if ~isfield(cfg, 'roarmIps'), cfg.roarmIps = {'192.168.1.192','192.168.1.101'};  end

    % --- Online mode auto-enables hardware ---
    if strcmpi(cfg.mode, 'online')
        cfg.connectKuka = true;
    end

    % --- Dataset config (for create_position_only_rl_dataset) ---
    if ~isfield(cfg, 'datasetCfg'), cfg.datasetCfg = struct(); end
    dc = cfg.datasetCfg;
    if ~isfield(dc, 'collectKuka'),     dc.collectKuka     = true;          end
    if ~isfield(dc, 'collectRoarm'),    dc.collectRoarm    = true;          end
    if ~isfield(dc, 'roarmMode'),       dc.roarmMode       = 'synthetic';   end
    if ~isfield(dc, 'roarmIps'),        dc.roarmIps        = cfg.roarmIps;  end
    if ~isfield(dc, 'roarmEpisodes'),   dc.roarmEpisodes   = 6;            end
    if ~isfield(dc, 'roarmStepsPerEp'), dc.roarmStepsPerEp = 40;           end
    if ~isfield(dc, 'roarmDt'),         dc.roarmDt         = 0.20;         end
    if ~isfield(dc, 'roarmSpeed'),      dc.roarmSpeed      = 5.0;          end
    cfg.datasetCfg = dc;

    % --- Synchronized collector config ---
    if ~isfield(cfg, 'syncCfg'), cfg.syncCfg = struct(); end
    sc = cfg.syncCfg;
    if ~isfield(sc, 'roarmIps'),        sc.roarmIps        = cfg.roarmIps;  end
    if ~isfield(sc, 'roarmDt'),         sc.roarmDt         = 0.20;         end
    if ~isfield(sc, 'roarmEpisodes'),   sc.roarmEpisodes   = 6;            end
    if ~isfield(sc, 'roarmStepsPerEp'), sc.roarmStepsPerEp = 40;           end
    if ~isfield(sc, 'roarmSpeed'),      sc.roarmSpeed      = 5;            end
    cfg.syncCfg = sc;

    % --- Collision avoidance config ---
    if ~isfield(cfg, 'collisionCfg'), cfg.collisionCfg = struct(); end
    cc = cfg.collisionCfg;

    if ~isfield(cc, 'collect'), cc.collect = struct(); end
    col = cc.collect;
    if ~isfield(col, 'mode'),        col.mode        = 'synthetic';         end
    if ~isfield(col, 'episodes'),    col.episodes    = 20;                  end
    if ~isfield(col, 'stepsPerEp'),  col.stepsPerEp  = 50;                 end
    if ~isfield(col, 'roarmDt'),     col.roarmDt     = 0.20;               end
    if ~isfield(col, 'roarmIps'),    col.roarmIps    = cfg.roarmIps;        end
    if ~isfield(col, 'roarmSpeed'),  col.roarmSpeed  = 5;                   end
    if ~isfield(col, 'kukaPosition'),col.kukaPosition= [1000, 0, 300];     end
    if ~isfield(col, 'connectKuka'), col.connectKuka = false;               end
    cc.collect = col;

    if ~isfield(cc, 'train'), cc.train = struct(); end
    tr = cc.train;
    if ~isfield(tr, 'maxEpisodes'),   tr.maxEpisodes   = 500;              end
    if ~isfield(tr, 'maxSteps'),      tr.maxSteps      = 50;               end
    if ~isfield(tr, 'dt'),            tr.dt            = 0.20;             end
    if ~isfield(tr, 'actionLimit'),   tr.actionLimit   = 30;               end
    if ~isfield(tr, 'hiddenSizes'),   tr.hiddenSizes   = [128, 128];       end
    if ~isfield(tr, 'actorLR'),       tr.actorLR       = 1e-4;             end
    if ~isfield(tr, 'criticLR'),      tr.criticLR      = 1e-3;             end
    if ~isfield(tr, 'gamma'),         tr.gamma         = 0.99;             end
    if ~isfield(tr, 'bufferLength'),  tr.bufferLength  = 100000;           end
    if ~isfield(tr, 'miniBatchSize'), tr.miniBatchSize = 64;               end
    if ~isfield(tr, 'kukaPosition'),  tr.kukaPosition  = [1000, 0, 300];   end
    cc.train = tr;

    if ~isfield(cc, 'eval'), cc.eval = struct(); end
    ev = cc.eval;
    if ~isfield(ev, 'numEpisodes'), ev.numEpisodes = 100; end
    if ~isfield(ev, 'maxSteps'),    ev.maxSteps    = 50;  end
    cc.eval = ev;

    cfg.collisionCfg = cc;
end