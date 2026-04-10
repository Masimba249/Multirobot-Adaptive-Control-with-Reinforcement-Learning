function state = build_collision_state(arm_id, r1_xyz, r2_xyz, kuka_xyz, ...
                                        target_xyz, prev_xyz, dt, dists)
%BUILD_COLLISION_STATE Build the RL state vector for one arm.
%
%   state = build_collision_state(arm_id, r1_xyz, r2_xyz, kuka_xyz, ...
%                                  target_xyz, prev_xyz, dt, dists)
%
%   Output: [1x18] normalized state vector:
%     [ own_x, own_y, own_z,                   % own position (normalized)
%       target_x, target_y, target_z,           % target position
%       err_x, err_y, err_z,                    % tracking error
%       vel_x, vel_y, vel_z,                    % velocity estimate
%       delta_other_x, delta_other_y, delta_other_z,  % vector to other RoArm
%       dist_to_other_roarm,                    % scalar distance
%       dist_to_kuka ]                          % scalar distance to KUKA

    % Identify own vs other
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

    % Direction to other arm
    delta_other = other_xyz - own_xyz;

    % === Normalization constants (workspace-scale) ===
    POS_SCALE  = 500;
    VEL_SCALE  = 2000;
    DIST_SCALE = 1000;

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