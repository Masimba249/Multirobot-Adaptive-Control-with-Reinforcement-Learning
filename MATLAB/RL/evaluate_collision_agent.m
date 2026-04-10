function evaluate_collision_agent(agentPath, cfg)
%EVALUATE_COLLISION_AGENT Evaluate a trained collision avoidance agent.
%
%   evaluate_collision_agent(agentPath)
%   evaluate_collision_agent(agentPath, cfg)
%
%   Runs the trained agent in CollisionAvoidanceEnv for N episodes,
%   collects metrics, and generates plots saved to RL\figures\.

    if nargin < 2, cfg = struct(); end
    if ~isfield(cfg, 'numEpisodes'), cfg.numEpisodes = 100; end
    if ~isfield(cfg, 'maxSteps'),    cfg.maxSteps = 50;     end

    fprintf('Loading agent: %s\n', agentPath);
    loaded = load(agentPath);
    if isfield(loaded, 'agent')
        agent = loaded.agent;
    elseif isfield(loaded, 'policyOut')
        agent = loaded.policyOut.agent;
    else
        error('No agent found in %s', agentPath);
    end

    env = CollisionAvoidanceEnv();
    env.MaxSteps = cfg.maxSteps;

    % Metrics storage
    ep_rewards      = zeros(cfg.numEpisodes, 1);
    ep_collisions   = zeros(cfg.numEpisodes, 1);
    ep_min_dists    = zeros(cfg.numEpisodes, 1);
    ep_avg_dists    = zeros(cfg.numEpisodes, 1);
    ep_pos_errors   = zeros(cfg.numEpisodes, 1);
    all_min_dists   = [];

    fprintf('Evaluating %d episodes...\n', cfg.numEpisodes);

    for ep = 1:cfg.numEpisodes
        obs = reset(env);
        totalReward = 0;
        minDists = [];
        collisions = 0;
        posErrors = [];

        for step = 1:cfg.maxSteps
            action = getAction(agent, {obs});
            [obs, reward, isDone, loggedSignals] = env.step(action{1});

            totalReward = totalReward + reward;
            minDists(end+1) = loggedSignals.min_dist; %#ok<AGROW>
            posErrors(end+1) = loggedSignals.pos_error; %#ok<AGROW>
            if loggedSignals.collision
                collisions = collisions + 1;
            end

            if isDone, break; end
        end

        ep_rewards(ep)    = totalReward;
        ep_collisions(ep) = collisions;
        ep_min_dists(ep)  = min(minDists);
        ep_avg_dists(ep)  = mean(minDists);
        ep_pos_errors(ep) = mean(posErrors);
        all_min_dists     = [all_min_dists, minDists]; %#ok<AGROW>
    end

    % === Print summary ===
    fprintf('\n========================================\n');
    fprintf(' EVALUATION RESULTS (%d episodes)\n', cfg.numEpisodes);
    fprintf('========================================\n');
    fprintf('Avg episode reward:    %.2f\n', mean(ep_rewards));
    fprintf('Collision episodes:    %d / %d (%.1f%%)\n', ...
        sum(ep_collisions > 0), cfg.numEpisodes, ...
        100*sum(ep_collisions > 0)/cfg.numEpisodes);
    fprintf('Total collisions:      %d\n', sum(ep_collisions));
    fprintf('Avg min distance:      %.1f mm\n', mean(ep_min_dists));
    fprintf('Global min distance:   %.1f mm\n', min(all_min_dists));
    fprintf('Avg position error:    %.1f mm\n', mean(ep_pos_errors));
    fprintf('========================================\n');

    % === Generate plots ===
    figDir = fullfile(fileparts(mfilename('fullpath')), 'figures');
    if ~isfolder(figDir), mkdir(figDir); end
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');

    % 1. Episode rewards
    fig1 = figure('Name', 'Collision RL — Episode Rewards', 'Visible', 'on');
    plot(ep_rewards, 'b-', 'LineWidth', 1.2); hold on;
    yline(mean(ep_rewards), 'r--', sprintf('Mean=%.1f', mean(ep_rewards)), 'LineWidth', 1.5);
    xlabel('Episode'); ylabel('Total Reward'); title('Episode Rewards');
    grid on;
    saveas(fig1, fullfile(figDir, sprintf('collision_eval_rewards_%s.png', timestamp)));

    % 2. Min distance per episode
    fig2 = figure('Name', 'Collision RL — Min Distance', 'Visible', 'on');
    bar(ep_min_dists, 'FaceColor', [0.2 0.6 0.9]); hold on;
    yline(50, 'r--', 'Collision boundary (50mm)', 'LineWidth', 2);
    yline(80, 'Color', [0.9 0.5 0], 'LineStyle', '--', 'Label', 'Safety zone (80mm)', 'LineWidth', 1.5);
    xlabel('Episode'); ylabel('Min Distance (mm)'); title('Minimum Pairwise Distance per Episode');
    grid on;
    saveas(fig2, fullfile(figDir, sprintf('collision_eval_min_dist_%s.png', timestamp)));

    % 3. Distance distribution
    fig3 = figure('Name', 'Collision RL — Distance Distribution', 'Visible', 'on');
    histogram(all_min_dists, 50, 'FaceColor', [0.3 0.7 0.4]);
    xline(50, 'r--', 'Collision', 'LineWidth', 2);
    xline(80, 'Color', [0.9 0.5 0], 'LineStyle', '--', 'Label', 'Safety', 'LineWidth', 1.5);
    xlabel('Min Distance (mm)'); ylabel('Count'); title('Distribution of Min Pairwise Distances');
    grid on;
    saveas(fig3, fullfile(figDir, sprintf('collision_eval_dist_hist_%s.png', timestamp)));

    % 4. Position error vs safety
    fig4 = figure('Name', 'Collision RL — Error vs Safety', 'Visible', 'on');
    scatter(ep_avg_dists, ep_pos_errors, 30, ep_rewards, 'filled');
    colorbar; colormap(jet);
    xlabel('Avg Min Distance (mm)'); ylabel('Avg Position Error (mm)');
    title('Task Accuracy vs Collision Safety (color=reward)');
    grid on;
    saveas(fig4, fullfile(figDir, sprintf('collision_eval_tradeoff_%s.png', timestamp)));

    fprintf('Figures saved to: %s\n', figDir);

    % Save evaluation results
    resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');
    evalResults = struct();
    evalResults.ep_rewards = ep_rewards;
    evalResults.ep_collisions = ep_collisions;
    evalResults.ep_min_dists = ep_min_dists;
    evalResults.ep_avg_dists = ep_avg_dists;
    evalResults.ep_pos_errors = ep_pos_errors;
    evalResults.all_min_dists = all_min_dists;
    evalResults.cfg = cfg;
    evalPath = fullfile(resultsDir, sprintf('collision_eval_%s.mat', timestamp));
    save(evalPath, 'evalResults');
    fprintf('Eval results saved: %s\n', evalPath);
end