classdef replay_buffer < handle
%REPLAY_BUFFER Experience replay buffer for off-policy RL training.
%
%   buf = replay_buffer(capacity, state_dim, action_dim)
%   buf.add(state, action, reward, next_state, done)
%   [s, a, r, ns, d] = buf.sample(batch_size)

    properties
        capacity
        state_dim
        action_dim
        count
        ptr
        states
        actions
        rewards
        next_states
        dones
    end

    methods
        function obj = replay_buffer(capacity, state_dim, action_dim)
            obj.capacity   = capacity;
            obj.state_dim  = state_dim;
            obj.action_dim = action_dim;
            obj.count      = 0;
            obj.ptr        = 0;

            obj.states      = zeros(capacity, state_dim);
            obj.actions     = zeros(capacity, action_dim);
            obj.rewards     = zeros(capacity, 1);
            obj.next_states = zeros(capacity, state_dim);
            obj.dones       = zeros(capacity, 1);
        end

        function add(obj, state, action, reward, next_state, done)
            obj.ptr = mod(obj.ptr, obj.capacity) + 1;
            obj.states(obj.ptr, :)      = state;
            obj.actions(obj.ptr, :)     = action;
            obj.rewards(obj.ptr)        = reward;
            obj.next_states(obj.ptr, :) = next_state;
            obj.dones(obj.ptr)          = done;
            obj.count = min(obj.count + 1, obj.capacity);
        end

        function [s, a, r, ns, d] = sample(obj, batch_size)
            batch_size = min(batch_size, obj.count);
            idx = randperm(obj.count, batch_size);
            s  = obj.states(idx, :);
            a  = obj.actions(idx, :);
            r  = obj.rewards(idx);
            ns = obj.next_states(idx, :);
            d  = obj.dones(idx);
        end

        function n = size(obj)
            n = obj.count;
        end

        function save_to_file(obj, filepath)
            buffer_data = struct();
            buffer_data.states      = obj.states(1:obj.count, :);
            buffer_data.actions     = obj.actions(1:obj.count, :);
            buffer_data.rewards     = obj.rewards(1:obj.count);
            buffer_data.next_states = obj.next_states(1:obj.count, :);
            buffer_data.dones       = obj.dones(1:obj.count);
            save(filepath, 'buffer_data');
        end

        function load_from_file(obj, filepath)
            loaded = load(filepath, 'buffer_data');
            bd = loaded.buffer_data;
            n = size(bd.states, 1);
            n = min(n, obj.capacity);
            obj.states(1:n, :)      = bd.states(1:n, :);
            obj.actions(1:n, :)     = bd.actions(1:n, :);
            obj.rewards(1:n)        = bd.rewards(1:n);
            obj.next_states(1:n, :) = bd.next_states(1:n, :);
            obj.dones(1:n)          = bd.dones(1:n);
            obj.count = n;
            obj.ptr   = n;
        end
    end
end