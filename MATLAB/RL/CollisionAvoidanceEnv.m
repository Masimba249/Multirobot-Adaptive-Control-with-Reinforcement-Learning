classdef CollisionAvoidanceEnv < rl.env.MATLABEnvironment
%COLLISIONAVOIDANCEENV RL environment for dual-RoArm + KUKA collision avoidance.
%
%   Observation: [18x1] state vector from build_collision_state()
%   Action:      [3x1]  position adjustment (dx, dy, dz) in mm
%
%   The environment simulates two RoArm robots and one KUKA TCP.
%   Each step, the agent adjusts one arm's target to avoid collisions
%   while still tracking the original waypoint.

    properties
        % Workspace limits (mm) — RoArm M2
        XRange = [120, 340]
        YRange = [-220, 220]
        ZRange = [80, 260]

        % KUKA TCP position (fixed or updated from live data)
        KukaPosition = [1000, 0, 300]   % ~1m away from RoArms
        KukaOffset = [1000, 0, 0]       % coordinate frame offset

        % Simulation state
        R1Position = [250, 0, 220]
        R2Position = [250, 100, 180]
        R1Target = [250, 0, 220]
        R2Target = [250, 100, 180]
        R1Prev = [NaN, NaN, NaN]
        R2Prev = [NaN, NaN, NaN]

        % Waypoints for both arms
        R1Waypoints = []
        R2Waypoints = []

        % Episode parameters
        StepCount = 0
        MaxSteps = 50
        Dt = 0.20             % seconds

        % Noise for simulated measurement
        MeasNoise = [1.5, 1.5, 1.0]

        % Action limit (mm)
        ActionLimit = 30

        % Active arm being controlled ('roarm_1' or 'roarm_2')
        ActiveArm = 'roarm_1'

        % Logging
        EpisodeLog = struct('rewards', [], 'min_dists', [], 'collisions', 0)
    end

    methods
        function this = CollisionAvoidanceEnv()
            % Define observation spec: 18-dimensional state
            ObservationInfo = rlNumericSpec([18, 1], ...
                'LowerLimit', -5 * ones(18, 1), ...
                'UpperLimit',  5 * ones(18, 1));
            ObservationInfo.Name = 'collision_state';
            ObservationInfo.Description = 'Position-based collision avoidance state';

            % Define action spec: 3D position adjustment
            ActionInfo = rlNumericSpec([3, 1], ...
                'LowerLimit', -30 * ones(3, 1), ...
                'UpperLimit',  30 * ones(3, 1));
            ActionInfo.Name = 'position_adjustment';
            ActionInfo.Description = 'XYZ offset to adjust target (mm)';

            this = this@rl.env.MATLABEnvironment(ObservationInfo, ActionInfo);
        end

        function [observation, reward, isDone, loggedSignals] = step(this, action)
            action = double(action(:).');
            action = min(max(action, -this.ActionLimit), this.ActionLimit);

            this.StepCount = this.StepCount + 1;

            % Get current waypoint targets
            stepIdx = min(this.StepCount, size(this.R1Waypoints, 1));
            this.R1Target = this.R1Waypoints(stepIdx, :);
            this.R2Target = this.R2Waypoints(stepIdx, :);

            % Apply action to active arm
            if strcmpi(this.ActiveArm, 'roarm_1')
                adjusted = this.R1Target + action;
            else
                adjusted = this.R2Target + action;
            end
            adjusted = this.clampWorkspace(adjusted);

            % Simulate motion with noise
            if strcmpi(this.ActiveArm, 'roarm_1')
                this.R1Prev = this.R1Position;
                this.R1Position = adjusted + randn(1,3) .* this.MeasNoise;
                this.R1Position = this.clampWorkspace(this.R1Position);

                % Other arm follows its target directly (no RL control)
                this.R2Prev = this.R2Position;
                this.R2Position = this.R2Target + randn(1,3) .* this.MeasNoise;
                this.R2Position = this.clampWorkspace(this.R2Position);
            else
                this.R2Prev = this.R2Position;
                this.R2Position = adjusted + randn(1,3) .* this.MeasNoise;
                this.R2Position = this.clampWorkspace(this.R2Position);

                this.R1Prev = this.R1Position;
                this.R1Position = this.R1Target + randn(1,3) .* this.MeasNoise;
                this.R1Position = this.clampWorkspace(this.R1Position);
            end

            % Compute distances
            dists = pairwise_distances(this.R1Position, this.R2Position, this.KukaPosition);

            % Compute reward
            if strcmpi(this.ActiveArm, 'roarm_1')
                target = this.R1Target;
                meas = this.R1Position;
                prev = this.R1Prev;
            else
                target = this.R2Target;
                meas = this.R2Position;
                prev = this.R2Prev;
            end

            [reward, info] = collision_reward(dists, target, meas, prev, this.Dt);

            % Log
            this.EpisodeLog.rewards(end+1) = reward;
            this.EpisodeLog.min_dists(end+1) = dists.min_dist;
            if info.in_collision
                this.EpisodeLog.collisions = this.EpisodeLog.collisions + 1;
            end

            % Build observation
            observation = build_collision_state(this.ActiveArm, ...
                this.R1Position, this.R2Position, this.KukaPosition, ...
                target, prev, this.Dt, dists);
            observation = observation(:);

            % Done conditions
            isDone = this.StepCount >= this.MaxSteps || info.in_collision;

            loggedSignals.reward     = reward;
            loggedSignals.min_dist   = dists.min_dist;
            loggedSignals.collision  = info.in_collision;
            loggedSignals.pos_error  = info.pos_error;
            loggedSignals.dist_r1_r2 = dists.r1_r2;
        end

        function observation = reset(this)
            this.StepCount = 0;
            this.EpisodeLog = struct('rewards', [], 'min_dists', [], 'collisions', 0);

            % Generate fresh waypoints for both arms
            this.R1Waypoints = this.generateWaypoints(this.MaxSteps);
            this.R2Waypoints = this.generateWaypoints(this.MaxSteps);

            % Enforce separation at start
            for i = 1:size(this.R1Waypoints, 1)
                d = norm(this.R1Waypoints(i,:) - this.R2Waypoints(i,:));
                if d < 100 && d > 0
                    dir = (this.R2Waypoints(i,:) - this.R1Waypoints(i,:)) / d;
                    this.R2Waypoints(i,:) = this.R1Waypoints(i,:) + dir * 100;
                elseif d == 0
                    this.R2Waypoints(i,:) = this.R2Waypoints(i,:) + [100, 0, 0];
                end
                this.R2Waypoints(i,:) = this.clampWorkspace(this.R2Waypoints(i,:));
            end

            % Set initial positions
            this.R1Position = this.R1Waypoints(1, :);
            this.R2Position = this.R2Waypoints(1, :);
            this.R1Prev = [NaN, NaN, NaN];
            this.R2Prev = [NaN, NaN, NaN];
            this.R1Target = this.R1Waypoints(1, :);
            this.R2Target = this.R2Waypoints(1, :);

            % Alternate active arm each episode
            if rand > 0.5
                this.ActiveArm = 'roarm_1';
            else
                this.ActiveArm = 'roarm_2';
            end

            dists = pairwise_distances(this.R1Position, this.R2Position, this.KukaPosition);
            observation = build_collision_state(this.ActiveArm, ...
                this.R1Position, this.R2Position, this.KukaPosition, ...
                this.R1Target, [NaN NaN NaN], this.Dt, dists);
            observation = observation(:);
        end

        function xyz = clampWorkspace(this, xyz)
            xyz(1) = min(max(xyz(1), this.XRange(1)), this.XRange(2));
            xyz(2) = min(max(xyz(2), this.YRange(1)), this.YRange(2));
            xyz(3) = min(max(xyz(3), this.ZRange(1)), this.ZRange(2));
        end

        function W = generateWaypoints(this, n)
            anchors = [250,0,220; 300,120,190; 300,-120,190; ...
                       210,160,130; 210,-160,130; 280,0,250];
            idx = randi(size(anchors,1), [4,1]);
            key = anchors(idx,:);
            W = zeros(n,3);
            segN = max(1, floor(n/(size(key,1)-1)));
            ptr = 1;
            for s = 1:(size(key,1)-1)
                p0 = key(s,:); p1 = key(s+1,:);
                for i = 1:segN
                    if ptr > n, break; end
                    alpha = (i-1)/max(1,segN-1);
                    W(ptr,:) = (1-alpha)*p0 + alpha*p1;
                    ptr = ptr + 1;
                end
            end
            while ptr <= n, W(ptr,:) = key(end,:); ptr = ptr+1; end
            W(:,1) = min(max(W(:,1), this.XRange(1)), this.XRange(2));
            W(:,2) = min(max(W(:,2), this.YRange(1)), this.YRange(2));
            W(:,3) = min(max(W(:,3), this.ZRange(1)), this.ZRange(2));
        end
    end
end