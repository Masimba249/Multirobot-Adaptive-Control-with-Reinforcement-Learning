%% ========== COLLISION AVOIDANCE REWARD ANALYSIS ==========
% Load collision dataset if available
collisionFiles = dir(fullfile('results', 'collision_dataset_*.csv'));
if ~isempty(collisionFiles)
    [~, latest] = max([collisionFiles.datenum]);
    cT = readtable(fullfile('results', collisionFiles(latest).name));

    figure('Name', 'Collision Reward Analysis');

    subplot(2,2,1);
    plot(cT.reward_r1, 'b-'); hold on;
    plot(cT.reward_r2, 'r-');
    legend('RoArm 1', 'RoArm 2');
    xlabel('Step'); ylabel('Reward'); title('Reward per Step');
    grid on;

    subplot(2,2,2);
    plot(cT.dist_r1_r2, 'b-', 'LineWidth', 1.5); hold on;
    plot(cT.dist_r1_kuka, 'r-');
    plot(cT.dist_r2_kuka, 'g-');
    yline(50, 'r--', 'Collision');
    yline(80, 'Color', [0.9 0.5 0], 'LineStyle', '--');
    legend('R1-R2', 'R1-KUKA', 'R2-KUKA');
    xlabel('Step'); ylabel('Distance (mm)'); title('Pairwise Distances');
    grid on;

    subplot(2,2,3);
    histogram(cT.dist_min, 40); hold on;
    xline(50, 'r--', 'LineWidth', 2);
    xlabel('Min Distance (mm)'); ylabel('Count');
    title('Min Distance Distribution');
    grid on;

    subplot(2,2,4);
    scatter(cT.dist_min, cT.reward_r1, 10, 'b', 'filled', 'MarkerFaceAlpha', 0.3);
    xlabel('Min Distance (mm)'); ylabel('Reward R1');
    title('Reward vs Distance');
    grid on;

    sgtitle('Collision Avoidance Reward Analysis');

    % Save
    figDir = fullfile(fileparts(mfilename('fullpath')), 'figures');
    saveas(gcf, fullfile(figDir, 'collision_reward_analysis.png'));
    fprintf('Collision reward analysis saved.\n');
end