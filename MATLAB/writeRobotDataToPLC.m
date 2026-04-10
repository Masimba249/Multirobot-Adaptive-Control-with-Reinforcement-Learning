function writeRobotDataToPLC(opcClient, nodeHandles, opcConnected)
    global ROBOT_STATE

    persistent n
    if isempty(n), n = 0; end
    n = n + 1;

    if ~opcConnected || isempty(opcClient) || ~isfield(nodeHandles, 'KUKA_Velocity')
        if mod(n,20)==0
            fprintf('[SIM] no live write\n');
        end
        return;
    end

    try
        writeValue(opcClient, nodeHandles.KUKA_Velocity, single(ROBOT_STATE.KUKA.Velocity));
        writeValue(opcClient, nodeHandles.KUKA_Torque,   single(ROBOT_STATE.KUKA.Torque));
        writeValue(opcClient, nodeHandles.KUKA_Position, single(ROBOT_STATE.KUKA.Position));

        writeValue(opcClient, nodeHandles.RoArm1_Velocity, single(ROBOT_STATE.RoArm1.Velocity));
        writeValue(opcClient, nodeHandles.RoArm1_Torque,   single(ROBOT_STATE.RoArm1.Torque));
        writeValue(opcClient, nodeHandles.RoArm1_Position, single(ROBOT_STATE.RoArm1.Position));

        writeValue(opcClient, nodeHandles.RoArm2_Velocity, single(ROBOT_STATE.RoArm2.Velocity));
        writeValue(opcClient, nodeHandles.RoArm2_Torque,   single(ROBOT_STATE.RoArm2.Torque));
        writeValue(opcClient, nodeHandles.RoArm2_Position, single(ROBOT_STATE.RoArm2.Position));

        if mod(n,20)==0
            rb = readValue(opcClient, nodeHandles.KUKA_Velocity);
            fprintf('[LIVE] KUKA vel = [%s]\n', num2str(rb));
        end

    catch ME
        fprintf('[OPC-UA] Write error: %s\n', ME.message);
    end
end