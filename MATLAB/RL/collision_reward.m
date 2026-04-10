function [R, info] = collision_reward(dists, target_xyz, meas_xyz, prev_meas_xyz, dt)
%COLLISION_REWARD Compute RL reward for position-based collision avoidance.
%
%   [R, info] = collision_reward(dists, target_xyz, meas_xyz, prev_meas_xyz, dt)
%
%   Inputs:
%     dists        - struct from pairwise_distances()
%     target_xyz   - [1x3] commanded target position for this arm
%     meas_xyz     - [1x3] current measured position for this arm
%     prev_meas_xyz- [1x3] previous step measured position (for velocity)
%     dt           - scalar, timestep in seconds
%
%   Outputs:
%     R    - scalar reward
%     info - struct with reward breakdown for logging
%
%   Reward design:
%     - Large negative penalty if within collision boundary
%     - Graduated penalty in comfort zone
%     - Negative tracking error (want to reach target)
%     - Small bonus for maintaining safe clearance
%     - Smoothness penalty for abrupt velocity changes

    % === Tunable parameters (mm) ===
    D_COLLISION = 50;    % Hard collision boundary
    D_SAFE      = 80;    % Soft safety boundary — strong penalty below this
    D_COMFORT   = 150;   % Comfort zone — graduated penalty below this
    D_GOOD      = 250;   % Beyond this, no distance penalty at all

    W_COLLISION = -200;  % Weight: collision event
    W_SAFETY    = -80;   % Weight: within safety zone
    W_COMFORT   = -20;   % Weight: within comfort zone
    W_TRACKING  = -0.5;  % Weight: position tracking error (per mm)
    W_SMOOTH    = -0.05; % Weight: velocity magnitude penalty
    W_SAFE_BONUS = 5;    % Bonus for staying in good clearance

    d_min = dists.min_dist;

    % --- Collision penalty (hard) ---
    if d_min < D_COLLISION
        R_collision = W_COLLISION;
    elseif d_min < D_SAFE
        % Linear ramp from W_SAFETY to 0 across [D_COLLISION, D_SAFE]
        frac = (d_min - D_COLLISION) / (D_SAFE - D_COLLISION);
        R_collision = W_SAFETY * (1 - frac);
    elseif d_min < D_COMFORT
        frac = (d_min - D_SAFE) / (D_COMFORT - D_SAFE);
        R_collision = W_COMFORT * (1 - frac);
    else
        R_collision = 0;
    end

    % --- Safe distance bonus ---
    if d_min >= D_GOOD
        R_bonus = W_SAFE_BONUS;
    elseif d_min >= D_COMFORT
        R_bonus = W_SAFE_BONUS * (d_min - D_COMFORT) / (D_GOOD - D_COMFORT);
    else
        R_bonus = 0;
    end

    % --- Task tracking reward (reach the target) ---
    pos_error = norm(target_xyz - meas_xyz);
    R_tracking = W_TRACKING * pos_error;

    % --- Smoothness penalty (penalize high instantaneous velocity) ---
    R_smooth = 0;
    if ~isempty(prev_meas_xyz) && all(isfinite(prev_meas_xyz)) && dt > 0
        velocity = norm(meas_xyz - prev_meas_xyz) / dt;
        R_smooth = W_SMOOTH * velocity;
    end

    % --- Total reward ---
    R = R_collision + R_bonus + R_tracking + R_smooth;

    % --- Breakdown for logging/debugging ---
    info = struct();
    info.R_collision = R_collision;
    info.R_bonus     = R_bonus;
    info.R_tracking  = R_tracking;
    info.R_smooth    = R_smooth;
    info.R_total     = R;
    info.d_min       = d_min;
    info.pos_error   = pos_error;
    info.in_collision = d_min < D_COLLISION;
end