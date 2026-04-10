%% ==========================================================================
%  MULTIROBOT PAINTING SYSTEM — PLC OPC-UA LAUNCHER
% ==========================================================================

clear; clc;

oldTimers = timerfindall;
for k = 1:numel(oldTimers)
    try, stop(oldTimers(k)); end
    try, delete(oldTimers(k)); end
end


fprintf('==========================================================\n');
fprintf('  MULTIROBOT PAINTING SYSTEM — STARTUP\n');
fprintf('==========================================================\n\n');

% --------------------------------------------------------------------------
% SECTION 1 — PATH SETUP
% --------------------------------------------------------------------------
scriptDir = fileparts(mfilename('fullpath'));
kukaDir   = fullfile(scriptDir, 'KUKA_control');
roarmDir  = fullfile(scriptDir, 'Roarm_control');

addpath(kukaDir);
addpath(roarmDir);

fprintf('[PATH]  Added to MATLAB path:\n');
fprintf('          %s\n', kukaDir);
fprintf('          %s\n\n', roarmDir);

% --------------------------------------------------------------------------
% SECTION 2 — PLC / OPC-UA CONFIGURATION
% --------------------------------------------------------------------------
PLC_CONFIG.ServerEndpoint    = 'opc.tcp://169.254.0.1:4840';
PLC_CONFIG.Namespace         = 3;
PLC_CONFIG.UpdateRate_s      = 0.1;
PLC_CONFIG.ConnectionTimeout = 10;

% --------------------------------------------------------------------------
% SECTION 3 — SHARED ROBOT STATE
% The GUIs update this; the OPC-UA timer reads it and writes to PLC.
% --------------------------------------------------------------------------
global ROBOT_STATE; %#ok<GVMIS>

ROBOT_STATE.KUKA.Velocity   = single(ones(1,6));
ROBOT_STATE.KUKA.Torque     = single(zeros(1,6));
ROBOT_STATE.KUKA.Position   = single(zeros(1,6));

ROBOT_STATE.RoArm1.Velocity = single(zeros(1,4));
ROBOT_STATE.RoArm1.Torque   = single(zeros(1,4));
ROBOT_STATE.RoArm1.Position = single(zeros(1,4));

ROBOT_STATE.RoArm2.Velocity = single(zeros(1,4));
ROBOT_STATE.RoArm2.Torque   = single(zeros(1,4));
ROBOT_STATE.RoArm2.Position = single(zeros(1,4));

fprintf('[STATE] Shared ROBOT_STATE structure initialised.\n\n');

% --------------------------------------------------------------------------
% SECTION 4 — OPC-UA CONNECTION
% --------------------------------------------------------------------------
fprintf('[OPC-UA] Connecting to Siemens PLC at %s ...\n', PLC_CONFIG.ServerEndpoint);

opcClient    = [];
opcConnected = false;

try
    % Используем тот же способ, который у тебя уже сработал в ручном тесте
    opcClient = opcua(PLC_CONFIG.ServerEndpoint);
    setSecurityModel(opcClient, "None");

    % ЭТУ СТРОКУ УБРАТЬ:
    % opcClient.Timeout = PLC_CONFIG.ConnectionTimeout;

    connect(opcClient);

    opcConnected = true;
    fprintf('[OPC-UA] Connected successfully.\n');
    fprintf('[OPC-UA]    Server:    %s\n', opcClient.ServerDescription);
    fprintf('[OPC-UA]    Namespace: %d\n\n', PLC_CONFIG.Namespace);

catch ME
    warning(ME.identifier, '[OPC-UA] Could not connect to PLC: %s', ME.message);
    fprintf('[OPC-UA] Running in SIMULATION mode — no data written to PLC.\n\n');
end




% --------------------------------------------------------------------------
% SECTION 5 — OPC-UA NODE HANDLES
% --------------------------------------------------------------------------
nodeHandles = struct();

if opcConnected
    try
        fprintf('[OPC-UA] Resolving node handles...\n');

        KUKA_Node   = findNodeByName(opcClient.Namespace, 'KUKA', '-once');
        RoArm1_Node = findNodeByName(opcClient.Namespace, 'RoArm1', '-once');
        RoArm2_Node = findNodeByName(opcClient.Namespace, 'RoArm2', '-once');

        nodeHandles.KUKA_Velocity = findNodeByName(KUKA_Node, 'Velocity', '-once');
        nodeHandles.KUKA_Torque   = findNodeByName(KUKA_Node, 'Torque', '-once');
        nodeHandles.KUKA_Position = findNodeByName(KUKA_Node, 'Position', '-once');

        nodeHandles.RoArm1_Velocity = findNodeByName(RoArm1_Node, 'Velocity', '-once');
        nodeHandles.RoArm1_Torque   = findNodeByName(RoArm1_Node, 'Torque', '-once');
        nodeHandles.RoArm1_Position = findNodeByName(RoArm1_Node, 'Position', '-once');

        nodeHandles.RoArm2_Velocity = findNodeByName(RoArm2_Node, 'Velocity', '-once');
        nodeHandles.RoArm2_Torque   = findNodeByName(RoArm2_Node, 'Torque', '-once');
        nodeHandles.RoArm2_Position = findNodeByName(RoArm2_Node, 'Position', '-once');

        fprintf('[OPC-UA] All 9 nodes resolved.\n');

        fprintf('[OPC-UA] Testing write/read before timer...\n');
        writeValue(opcClient, nodeHandles.KUKA_Velocity, single([10 20 30 40 50 60]));
        testVK = readValue(opcClient, nodeHandles.KUKA_Velocity);
        disp(testVK);
        fprintf('[OPC-UA] Pre-timer test passed.\n\n');

    catch ME
        warning(ME.identifier, '[OPC-UA] Node resolution or test failed: %s', ME.message);
        opcConnected = false;
    end
end

if ~opcConnected || ~isfield(nodeHandles, 'KUKA_Velocity')
    fprintf('[OPC-UA] Live mode not available. Timer will NOT start.\n');
    return;
end



% --------------------------------------------------------------------------
% SECTION 6 — BACKGROUND TIMER
% --------------------------------------------------------------------------
fprintf('[TIMER]  Starting OPC-UA write timer (%.0f ms interval)...\n', ...
        PLC_CONFIG.UpdateRate_s * 1000);

oldTimers = timerfindall('Name', 'OPC_UA_Write_Timer');
if ~isempty(oldTimers)
    for k = 1:numel(oldTimers)
        try, stop(oldTimers(k));   catch, end
        try, delete(oldTimers(k)); catch, end
    end
    fprintf('[TIMER]  Removed %d previous OPC timer(s).\n', numel(oldTimers));
end

opcTimer = timer( ...
    'Name',          'OPC_UA_Write_Timer', ...
    'ExecutionMode', 'fixedRate', ...
    'Period',        PLC_CONFIG.UpdateRate_s, ...
    'BusyMode',      'drop', ...
    'TimerFcn',      @(t,e) writeRobotDataToPLC(opcClient, nodeHandles, opcConnected), ...
    'ErrorFcn',      @(t,e) fprintf('[TIMER] Error: %s\n', e.Data.message), ...
    'StopFcn',       @(t,e) fprintf('[TIMER] OPC-UA write timer stopped.\n') ...
);

start(opcTimer);
fprintf('[TIMER]  Timer running.\n\n');

assignin('base', 'opcTimer',   opcTimer);
assignin('base', 'opcClient',  opcClient);
assignin('base', 'PLC_CONFIG', PLC_CONFIG);

fprintf('  TIP: To stop the OPC-UA timer:   stop(opcTimer)\n');
fprintf('  TIP: To disconnect from PLC:     disconnect(opcClient)\n\n');

% --------------------------------------------------------------------------
% SECTION 7 — LAUNCH THE TWO ROBOT GUIs
% --------------------------------------------------------------------------
fprintf('[GUI]  Launching KUKA pick-and-place GUI...\n');
try
    kuka_pick_and_place_gui();
    fprintf('[GUI]  KUKA GUI opened.\n');
catch ME
    warning(ME.identifier, '[GUI]  Could not open KUKA GUI: %s', ME.message);
end

pause(0.5);

fprintf('[GUI]  Launching RoArm painting process GUI...\n');
try
    roarms_painting_process_gui();
    fprintf('[GUI]  RoArm GUI opened.\n');
catch ME
    warning(ME.identifier, '[GUI]  Could not open RoArm GUI: %s', ME.message);
end

fprintf('\n==========================================================\n');
fprintf('  SYSTEM READY\n');
fprintf('  Use the KUKA GUI   to control the KUKA robot\n');
fprintf('  Use the RoArm GUI  to control the two RoArm arms\n');
fprintf('  OPC-UA streams live to PLC at %.0f ms intervals\n', ...
        PLC_CONFIG.UpdateRate_s * 1000);
fprintf('==========================================================\n\n');