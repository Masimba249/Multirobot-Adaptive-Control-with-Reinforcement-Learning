function state = build_rl_state(arm_id, r1_xyz, r2_xyz, kuka_xyz, ...
                                 target_xyz, prev_xyz, dt, dists)
%BUILD_RL_STATE Construct the RL state vector for one arm.
%
%   state = build_rl_state(arm_id, r1_xyz, r2_xyz, kuka_xyz, ...
%                           target_xyz, prev_xyz, dt, dists)
%
%   Inputs:
%     arm_id     - 'roarm_1' or 'roarm_2'
%     r1_xyz     - [1x3] RoArm 1 measured position
%     r2_xyz     - [1x3] RoArm 2 measured position
%     kuka_xyz   - [1x3] KUKA TCP measured position
%     target_xyz - [1x3] desired target position for this arm
%     prev_xyz   - [1x3] this arm's previous measured position
%     dt         - scalar timestep
%     dists      - struct from pairwise_distances()
%
%   Output:
%     state - [1x18] normalized state vector:
%       [ own_x, own_y, own_z,              % own measured position
%         target_x, target_y, target_z,     % own target position
%         err_x, err_y, err_z,              % tracking error vector
%         vel_x, vel_y, vel_z,              % own velocity estimate
%         delta_other_x, delta_other_y, delta_other_z,  % vector to other roarm
%         dist_to_other_roarm,              % scalar distance
%         dist_to_kuka ]                    % scalar distance to KUKA

    % Determine own and other positions
    if strcmpi(arm_id, 'roarm_1')
        own_xyz   = r1_xyz;
        other_xyz = r2_xyz;
        dist_other = dists.r1_r2;
        dist_kuka  = dists.r1_kuka;
    else
        own_xyz   = r2_xyz;
        other_xyz = r1_xyz;
        dist_other = dists.r1_r2;
        dist_kuka  = dists.r2_kuka;
    end

    % Tracking error
    err_xyz = target_xyz - own_xyz;

    % Velocity estimate
    if ~isempty(prev_xyz) && all(isfinite(prev_xyz)) && dt > 0
        vel_xyz = (own_xyz - prev_xyz) / dt;
    else
        vel_xyz = [0, 0, 0];
    end

    % Direction vector to other RoArm
    delta_other = other_xyz - own_xyz;

    % === Normalization constants (based on workspace) ===
    POS_SCALE  = 500;   % mm — roughly half the max workspace span
    VEL_SCALE  = 2000;  % mm/s — max expected TCP velocity
    DIST_SCALE = 1000;  % mm — max expected pairwise distance

    state = [ ...
        own_xyz / POS_SCALE, ...
        target_xyz / POS_SCALE, ...
        err_xyz / POS_SCALE, ...
        vel_xyz / VEL_SCALE, ...
        delta_other / DIST_SCALE, ...
        dist_other / DIST_SCALE, ...
        dist_kuka / DIST_SCALE ...
    ];
end