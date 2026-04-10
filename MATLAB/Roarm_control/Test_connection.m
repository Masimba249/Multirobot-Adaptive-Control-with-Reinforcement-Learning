%% ================================================================
%  RoArm_Painting_System_FINAL.m  — WITH TELEMETRY PLOTS
%% ================================================================

function RoArm_Painting_System_FINAL()
    clc; close all;
    cfg = buildConfig();
    buildMainGUI(cfg);
end

%% ================================================================
%  SECTION 1: CONFIGURATION  (unchanged)
%% ================================================================
function cfg = buildConfig()
    cfg.TABLE_X   = 795;   cfg.TABLE_Y   = 595;
    cfg.B1        = [50,   595/2, 0];
    cfg.B2        = [745,  595/2, 0];
    cfg.L0 = 123; cfg.L1 = 239; cfg.L2 = 280; cfg.L3 = 0;
    cfg.MAX_REACH = 480;   cfg.MIN_REACH = 50;
    cfg.Z_MIN     = 5;     cfg.Z_MAX     = 320;
    cfg.OBJ_DX = 105;  cfg.OBJ_DY = 210;  cfg.OBJ_DZ = 75;
    cfg.OBJ_CX = 795/2;   cfg.OBJ_CY = 595/2;  cfg.OBJ_Z0 = 0;
    cfg.STANDOFF  = 10;  cfg.STRIPE_H = 20;  cfg.APPROACH = 20;
    cfg.PAINT_SPD = 5.0;
    cfg.IP1       = '192.168.1.192';
    cfg.IP2       = '192.168.1.101';
    cfg.BASE_URL  = '/js?json=';
    cfg.HTTP_TO   = 0.8;  cfg.READ_TO   = 0.3;
    cfg.MAX_RET   = 1;    cfg.RET_DLY   = 0.05;
    cfg.POLL_INT  = 0.15;
    cfg.SETTLE_TOL     = 5.0;
    cfg.SETTLE_TIMEOUT = 1.0;
    cfg.SETTLE_POLL    = 0.25;
end

%% ================================================================
%  SECTION 2: GUI BUILDER
%  NEW: adds a telemetry plot panel below the workspace
%% ================================================================
function buildMainGUI(cfg)
    C.bg    = [0.10 0.10 0.13];
    C.panel = [0.16 0.16 0.20];
    C.acc1  = [0.10 0.55 1.00];
    C.acc2  = [1.00 0.50 0.10];
    C.grn   = [0.10 0.90 0.42];
    C.txt   = [0.92 0.92 0.92];
    C.dark  = [0.06 0.06 0.08];

    % Taller figure to fit telemetry row
    fig = uifigure('Name','RoArm-M2-S Live Painting Controller', ...
        'Position',[50 20 1400 920], ...
        'Color',C.bg,'Resize','off');

    uilabel(fig,'Text','  Live Painting System Session HMI Monitor', ...
        'Position',[0 880 1400 36], ...
        'FontSize',16,'FontWeight','bold', ...
        'FontColor',C.acc1,'BackgroundColor',C.bg, ...
        'HorizontalAlignment','center');

    [h.e1x, h.e1y, h.e1z, h.btn1, h.lbl1, h.tel1] = ...
        makeArmPanel(fig, 'ARM 1 (Front)', [10 590 350 280], C, C.acc1, cfg, 1);
    [h.e2x, h.e2y, h.e2z, h.btn2, h.lbl2, h.tel2] = ...
        makeArmPanel(fig, 'ARM 2 (Rear)', [370 590 350 280], C, C.acc2, cfg, 2);

    h = makeConfigPanel(fig, h, C, cfg, [730 590 660 280]);

    % 3D workspace (top-left)
    h.ax = uiaxes(fig,'Position',[10 295 580 285], ...
        'Color',C.dark,'XColor',C.txt,'YColor',C.txt,'ZColor',C.txt, ...
        'GridColor',[0.25 0.25 0.28],'FontSize',8);
    hold(h.ax,'on'); grid(h.ax,'on');
    xlabel(h.ax,'X (mm)'); ylabel(h.ax,'Y (mm)'); zlabel(h.ax,'Z (mm)');
    title(h.ax,'Workspace & Paths','Color',C.txt,'FontSize',9);
    view(h.ax,42,26);
    drawWorkspace(h.ax, cfg, C);

    % Live telemetry axes (bottom row — 4 plots)
    axBg = [0.06 0.06 0.08];
    axFg = [0.85 0.85 0.85];

    h.ax_pos1 = uiaxes(fig,'Position',[10  10 320 270], ...
        'Color',axBg,'XColor',axFg,'YColor',axFg,'GridColor',[0.25 0.25 0.28],'FontSize',7);
    hold(h.ax_pos1,'on'); grid(h.ax_pos1,'on');
    title(h.ax_pos1,'ARM1 Position (mm)','Color',axFg,'FontSize',8);
    xlabel(h.ax_pos1,'Step'); ylabel(h.ax_pos1,'mm');

    h.ax_pos2 = uiaxes(fig,'Position',[340 10 320 270], ...
        'Color',axBg,'XColor',axFg,'YColor',axFg,'GridColor',[0.25 0.25 0.28],'FontSize',7);
    hold(h.ax_pos2,'on'); grid(h.ax_pos2,'on');
    title(h.ax_pos2,'ARM2 Position (mm)','Color',axFg,'FontSize',8);
    xlabel(h.ax_pos2,'Step'); ylabel(h.ax_pos2,'mm');

    h.ax_spd1 = uiaxes(fig,'Position',[670 10 320 270], ...
        'Color',axBg,'XColor',axFg,'YColor',axFg,'GridColor',[0.25 0.25 0.28],'FontSize',7);
    hold(h.ax_spd1,'on'); grid(h.ax_spd1,'on');
    title(h.ax_spd1,'ARM1 Speed |V| (mm/s)','Color',axFg,'FontSize',8);
    xlabel(h.ax_spd1,'Step'); ylabel(h.ax_spd1,'mm/s');

    h.ax_spd2 = uiaxes(fig,'Position',[1000 10 320 270], ...
        'Color',axBg,'XColor',axFg,'YColor',axFg,'GridColor',[0.25 0.25 0.28],'FontSize',7);
    hold(h.ax_spd2,'on'); grid(h.ax_spd2,'on');
    title(h.ax_spd2,'ARM2 Speed |V| (mm/s)','Color',axFg,'FontSize',8);
    xlabel(h.ax_spd2,'Step'); ylabel(h.ax_spd2,'mm/s');

    % Animatedlines for live updates
    h.aln_p1x = animatedline(h.ax_pos1,'Color','r','LineWidth',1.4,'DisplayName','X');
    h.aln_p1y = animatedline(h.ax_pos1,'Color',[0 0.8 0.2],'LineWidth',1.4,'DisplayName','Y');
    h.aln_p1z = animatedline(h.ax_pos1,'Color','b','LineWidth',1.4,'DisplayName','Z');
    legend(h.ax_pos1,'Location','best','TextColor',axFg,'Color',axBg,'FontSize',7);

    h.aln_p2x = animatedline(h.ax_pos2,'Color','r','LineWidth',1.4,'DisplayName','X');
    h.aln_p2y = animatedline(h.ax_pos2,'Color',[0 0.8 0.2],'LineWidth',1.4,'DisplayName','Y');
    h.aln_p2z = animatedline(h.ax_pos2,'Color','b','LineWidth',1.4,'DisplayName','Z');
    legend(h.ax_pos2,'Location','best','TextColor',axFg,'Color',axBg,'FontSize',7);

    h.aln_s1 = animatedline(h.ax_spd1,'Color',[0.5 0.1 0.9],'LineWidth',1.8);
    h.aln_s2 = animatedline(h.ax_spd2,'Color',[1.0 0.5 0.1],'LineWidth',1.8);

    % Log area
    logPan = uipanel(fig,'Title','Live Event Log', ...
        'Position',[1330 10 60 560], ...     % narrow, right side
        'BackgroundColor',C.panel,'ForegroundColor',C.txt, ...
        'FontSize',9,'BorderType','line');

    logPan = uipanel(fig,'Title','Live Event Log', ...
        'Position',[600 295 790 285], ...
        'BackgroundColor',C.panel,'ForegroundColor',C.txt, ...
        'FontSize',9,'BorderType','line');
    h.log = uitextarea(logPan,'Position',[4 4 778 258], ...
        'BackgroundColor',C.dark,'FontColor',[0.55 1.0 0.55], ...
        'FontSize',8,'Editable','off', ...
        'Value',{'[System] Ready.', ...
                 'Object: 210mm(X) x 105mm(Y) x 75mm(Z)', ...
                 'Telemetry plots update live during painting.'});

    uilabel(fig,'Text','Status:', ...
        'Position',[10 876 55 22],'FontColor',C.txt,'BackgroundColor',C.bg,'FontSize',9);
    h.progLbl = uilabel(fig,'Text','⚠ Not Connected', ...
        'Position',[68 876 400 22],'FontColor',[1 0.5 0],'BackgroundColor',C.bg, ...
        'FontSize',10,'FontWeight','bold');

    % Telemetry log storage (cell arrays grown during painting)
    h.tel_log1 = [];   % Nx4: [step, x, y, z]
    h.tel_log2 = [];
    h.step1    = 0;
    h.step2    = 0;
    h.stopFlag = false;

    fig.UserData = h;
    h.btn1.ButtonPushedFcn = @(~,~) cbMove(fig, 1, cfg);
    h.btn2.ButtonPushedFcn = @(~,~) cbMove(fig, 2, cfg);
    rewireConfigButtons(fig, cfg);
    fig.UserData = h;
end

%% ================================================================
%  ARM PANEL  (unchanged)
%% ================================================================
function [ex,ey,ez,btn,lblIK,lblTel] = makeArmPanel(fig, titleStr, pos, C, acc, cfg, armIdx)
    pan = uipanel(fig,'Title',titleStr,'Position',pos, ...
        'BackgroundColor',C.panel,'ForegroundColor',acc, ...
        'FontSize',10,'FontWeight','bold','BorderType','line');
    row = @(n) 230 - n*40;  lw = 95;  ew = 110;
    if armIdx == 1
        defaultX = cfg.OBJ_CX; defaultY = cfg.OBJ_CY-40; defaultZ = 50;
    else
        defaultX = cfg.OBJ_CX; defaultY = cfg.OBJ_CY+40; defaultZ = 50;
    end
    uilabel(pan,'Text','X global (mm):','Position',[8 row(0) lw 20], ...
        'FontColor',C.txt,'BackgroundColor',C.panel,'FontSize',9);
    ex = uieditfield(pan,'numeric','Position',[lw+8 row(0) ew 24],'Value',defaultX, ...
        'Limits',[-100 cfg.TABLE_X+100],'BackgroundColor',C.dark,'FontColor',C.txt,'FontSize',10);
    uilabel(pan,'Text','Y global (mm):','Position',[8 row(1) lw 20], ...
        'FontColor',C.txt,'BackgroundColor',C.panel,'FontSize',9);
    ey = uieditfield(pan,'numeric','Position',[lw+8 row(1) ew 24],'Value',defaultY, ...
        'Limits',[-100 cfg.TABLE_Y+100],'BackgroundColor',C.dark,'FontColor',C.txt,'FontSize',10);
    uilabel(pan,'Text','Z height (mm):','Position',[8 row(2) lw 20], ...
        'FontColor',C.txt,'BackgroundColor',C.panel,'FontSize',9);
    ez = uieditfield(pan,'numeric','Position',[lw+8 row(2) ew 24],'Value',defaultZ, ...
        'Limits',[0 350],'BackgroundColor',C.dark,'FontColor',C.txt,'FontSize',10);
    btn = uibutton(pan,'Text','Move','Position',[8 row(3) 210 30], ...
        'BackgroundColor',acc,'FontColor','k','FontSize',10,'FontWeight','bold');
    lblIK  = uilabel(pan,'Text','Ready','Position',[8 row(4) 330 20], ...
        'FontColor',[0.5 0.9 0.5],'BackgroundColor',C.panel,'FontSize',8,'WordWrap','on');
    uilabel(pan,'Text','Live:','Position',[8 row(5) 40 18], ...
        'FontColor',[0.5 0.5 0.6],'BackgroundColor',C.panel,'FontSize',8);
    lblTel = uilabel(pan,'Text','—','Position',[50 row(5) 288 18], ...
        'FontColor',[0.6 0.9 1.0],'BackgroundColor',C.panel,'FontSize',8);
end

function h = makeConfigPanel(fig, h, C, cfg, pos)
    pan = uipanel(fig,'Title','Settings & Control','Position',pos, ...
        'BackgroundColor',C.panel,'ForegroundColor',C.txt, ...
        'FontSize',10,'FontWeight','bold','BorderType','line','Tag','cfgPanel');
    lw = 140; ew = 85;  row = @(n) 245 - n*37;
    uilabel(pan,'Text','Stripe (mm):','Position',[8 row(0) lw 20], ...
        'FontColor',C.txt,'BackgroundColor',C.panel,'FontSize',9);
    h.eStripe = uieditfield(pan,'numeric','Position',[lw+5 row(0) ew 22], ...
        'Value',cfg.STRIPE_H,'Limits',[5 50],'BackgroundColor',C.dark,'FontColor',C.txt);
    uilabel(pan,'Text','Standoff (mm):','Position',[8 row(1) lw 20], ...
        'FontColor',C.txt,'BackgroundColor',C.panel,'FontSize',9);
    h.eStandoff = uieditfield(pan,'numeric','Position',[lw+5 row(1) ew 22], ...
        'Value',cfg.STANDOFF,'Limits',[10 100],'BackgroundColor',C.dark,'FontColor',C.txt);
    uilabel(pan,'Text','Speed:','Position',[8 row(2) lw 20], ...
        'FontColor',C.txt,'BackgroundColor',C.panel,'FontSize',9);
    h.eSpeed = uieditfield(pan,'numeric','Position',[lw+5 row(2) ew 22], ...
        'Value',cfg.PAINT_SPD,'Limits',[1 20],'BackgroundColor',C.dark,'FontColor',C.txt);
    uilabel(pan,'Text','ARM 1 IP:','Position',[8 row(3) lw 20], ...
        'FontColor',C.txt,'BackgroundColor',C.panel,'FontSize',9);
    h.eIP1 = uieditfield(pan,'text','Position',[lw+5 row(3) 200 22], ...
        'Value',cfg.IP1,'BackgroundColor',C.dark,'FontColor',C.txt);
    uilabel(pan,'Text','ARM 2 IP:','Position',[8 row(4) lw 20], ...
        'FontColor',C.txt,'BackgroundColor',C.panel,'FontSize',9);
    h.eIP2 = uieditfield(pan,'text','Position',[lw+5 row(4) 200 22], ...
        'Value',cfg.IP2,'BackgroundColor',C.dark,'FontColor',C.txt);
    uibutton(pan,'Text','Ping','Position',[8 row(5) 100 27], ...
        'BackgroundColor',[0.18 0.42 0.68],'FontColor','w','FontSize',9,'Tag','btnPing');
    uibutton(pan,'Text','HOME','Position',[115 row(5) 100 27], ...
        'BackgroundColor',[0.28 0.28 0.52],'FontColor','w','FontSize',9,'Tag','btnHome');
    uibutton(pan,'Text','Telemetry','Position',[222 row(5) 110 27], ...
        'BackgroundColor',[0.16 0.38 0.16],'FontColor','w','FontSize',9,'Tag','btnTel');
    uibutton(pan,'Text','▶  START PAINTING','Position',[8 55 630 40], ...
        'BackgroundColor',[0.08 0.82 0.38],'FontColor',[0 0.08 0], ...
        'FontSize',13,'FontWeight','bold','Tag','btnStart');
    uibutton(pan,'Text','■  STOP','Position',[8 8 630 32], ...
        'BackgroundColor',[0.88 0.18 0.18],'FontColor','w', ...
        'FontSize',11,'FontWeight','bold','Tag','btnStop');
end

function rewireConfigButtons(fig, cfg)
    findobj(fig,'Tag','btnPing').ButtonPushedFcn  = @(~,~) cbPing(fig, cfg);
    findobj(fig,'Tag','btnHome').ButtonPushedFcn  = @(~,~) cbHome(fig, cfg);
    findobj(fig,'Tag','btnTel').ButtonPushedFcn   = @(~,~) cbTel(fig, cfg);
    findobj(fig,'Tag','btnStart').ButtonPushedFcn = @(~,~) cbStart(fig, cfg);
    findobj(fig,'Tag','btnStop').ButtonPushedFcn  = @(~,~) cbStop(fig, cfg);
end

%% ================================================================
%  COORDINATE TRANSFORM / REACHABILITY  (unchanged)
%% ================================================================
function [lx, ly, lz] = globalToLocal(gx, gy, gz, armIdx, cfg)
    if armIdx == 1
        lx = gx - cfg.B1(1);  ly = gy - cfg.B1(2);
    else
        lx = cfg.B2(1) - gx;  ly = cfg.B2(2) - gy;
    end
    lz = gz;
end

function [ok, msg] = checkReach(lx, ly, lz, cfg)
    r  = sqrt(lx^2 + ly^2);
    zR = lz - cfg.L0;
    d3 = sqrt(r^2 + zR^2);
    if lx < 0
        ok = false; msg = sprintf('BEHIND BASE: lx=%.1f', lx); return
    end
    if d3 > (cfg.L1+cfg.L2+cfg.L3)
        ok = false; msg = sprintf('OUT OF REACH: %.1fmm', d3); return
    end
    if d3 > cfg.MAX_REACH
        ok = false; msg = sprintf('BEYOND MAX_REACH: %.1fmm', d3); return
    end
    if r < cfg.MIN_REACH && lz < (cfg.L0+20)
        ok = false; msg = sprintf('TOO CLOSE: r=%.1fmm', r); return
    end
    if lz < cfg.Z_MIN || lz > cfg.Z_MAX
        ok = false; msg = sprintf('Z=%.1f out of [%d-%d]', lz, cfg.Z_MIN, cfg.Z_MAX); return
    end
    ok = true;
    msg = sprintf('OK r=%.1f d3D=%.1f', r, d3);
end

%% ================================================================
%  HTTP  (unchanged)
%% ================================================================
function [resp, success] = httpSend(ip, baseURL, jsonCmd, timeout, maxRet, retDly)
    url = ['http://' char(ip) char(baseURL) urlencode(jsonCmd)];
    resp = ''; success = false;
    for k = 1:maxRet
        try
            resp = webread(url, weboptions('Timeout', timeout));
            success = true; return;
        catch
            if k < maxRet, pause(retDly); end
        end
    end
end

function data = httpReadStatus(ip, timeout)
    cmd = struct('T', 105);
    url = ['http://' char(ip) '/js?json=' urlencode(jsonencode(cmd))];
    data = [];
    try
        resp = webread(url, weboptions('Timeout', timeout));
        s = strfind(resp,'{'); e = strfind(resp,'}');
        if ~isempty(s) && ~isempty(e)
            data = jsondecode(resp(s(1):e(end)));
        else
            data = jsondecode(resp);
        end
    catch; end
end

function [x, y, z] = parseTel(data)
    x = getField(data,{'x','X','px','posx'});
    y = getField(data,{'y','Y','py','posy'});
    z = getField(data,{'z','Z','pz','posz'});
end

function v = getField(s, names)
    v = 0;
    if isempty(s) || ~isstruct(s), return; end
    for k = 1:numel(names)
        if isfield(s, names{k})
            try; v = double(s.(names{k})); return; catch; end
        end
    end
end

function [connected, arm1ok, arm2ok] = validateConnection(ip1, ip2, timeout)
    arm1ok = ~isempty(httpReadStatus(ip1, timeout));
    arm2ok = ~isempty(httpReadStatus(ip2, timeout));
    connected = arm1ok && arm2ok;
end

function arrived = waitForArrival(ip, targetLocal, cfg)
    tStart = tic;  arrived = false;
    while toc(tStart) < cfg.SETTLE_TIMEOUT
        pause(cfg.SETTLE_POLL);
        data = httpReadStatus(ip, cfg.READ_TO);
        if isempty(data), continue; end
        [rx,ry,rz] = parseTel(data);
        if sqrt((rx-targetLocal(1))^2+(ry-targetLocal(2))^2+(rz-targetLocal(3))^2) <= cfg.SETTLE_TOL
            arrived = true; return
        end
    end
end

%% ================================================================
%  PATH GENERATION  (unchanged)
%% ================================================================
function paths = generateAllPaths(cfg)
    so=cfg.STANDOFF; sH=cfg.STRIPE_H; app=cfg.APPROACH;
    cx=cfg.OBJ_CX; cy=cfg.OBJ_CY;
    dx=cfg.OBJ_DX/2; dy=cfg.OBJ_DY/2; dz=cfg.OBJ_DZ; z0=cfg.OBJ_Z0;
    zBot=z0+8; zTop=z0+dz-8;
    nStrZ=max(1,floor((zTop-zBot)/sH)+1);
    zLevels=linspace(zBot,zTop,nStrZ);

    xFront=cx-dx-so; pathFront=[];
    for i=1:nStrZ, z=zLevels(i);
        if mod(i,2)==1
            pathFront=[pathFront; xFront,cy-dy-app,z; xFront,cy-dy,z; xFront,cy+dy,z; xFront,cy+dy+app,z]; %#ok<AGROW>
        else
            pathFront=[pathFront; xFront,cy+dy+app,z; xFront,cy+dy,z; xFront,cy-dy,z; xFront,cy-dy-app,z]; %#ok<AGROW>
        end
    end
    yRight=cy-dy-so; pathRight=[];
    for i=1:nStrZ, z=zLevels(i);
        if mod(i,2)==1
            pathRight=[pathRight; cx-dx-app,yRight,z; cx-dx,yRight,z; cx+dx,yRight,z; cx+dx+app,yRight,z]; %#ok<AGROW>
        else
            pathRight=[pathRight; cx+dx+app,yRight,z; cx+dx,yRight,z; cx-dx,yRight,z; cx-dx-app,yRight,z]; %#ok<AGROW>
        end
    end
    xRear=cx+dx+so; pathRear=[];
    for i=1:nStrZ, z=zLevels(i);
        if mod(i,2)==1
            pathRear=[pathRear; xRear,cy-dy-app,z; xRear,cy-dy,z; xRear,cy+dy,z; xRear,cy+dy+app,z]; %#ok<AGROW>
        else
            pathRear=[pathRear; xRear,cy+dy+app,z; xRear,cy+dy,z; xRear,cy-dy,z; xRear,cy-dy-app,z]; %#ok<AGROW>
        end
    end
    yLeft=cy+dy+so; pathLeft=[];
    for i=1:nStrZ, z=zLevels(i);
        if mod(i,2)==1
            pathLeft=[pathLeft; cx-dx-app,yLeft,z; cx-dx,yLeft,z; cx+dx,yLeft,z; cx+dx+app,yLeft,z]; %#ok<AGROW>
        else
            pathLeft=[pathLeft; cx+dx+app,yLeft,z; cx+dx,yLeft,z; cx-dx,yLeft,z; cx-dx-app,yLeft,z]; %#ok<AGROW>
        end
    end
    zTop_face=z0+dz+so;
    xLeft_top=cx-dx+8; xRight_top=cx+dx-8;
    nStrX=max(1,floor((xRight_top-xLeft_top)/sH)+1);
    xLevels=linspace(xLeft_top,xRight_top,nStrX);
    pathTop=[];
    for i=1:nStrX, x=xLevels(i);
        if mod(i,2)==1
            pathTop=[pathTop; x,cy-dy-app,zTop_face; x,cy-dy,zTop_face; x,cy+dy,zTop_face; x,cy+dy+app,zTop_face]; %#ok<AGROW>
        else
            pathTop=[pathTop; x,cy+dy+app,zTop_face; x,cy+dy,zTop_face; x,cy-dy,zTop_face; x,cy-dy-app,zTop_face]; %#ok<AGROW>
        end
    end
    paths.arm1_front=pathFront; paths.arm1_right=pathRight;
    paths.arm2_rear=pathRear;   paths.arm2_left=pathLeft;
    paths.arm2_top=pathTop;
end

function [validCount, totalCount, issues] = validatePaths(paths, cfg)
    faceNames={'arm1_front','arm1_right','arm2_rear','arm2_left','arm2_top'};
    armIdxs=[1,1,2,2,2];
    totalCount=0; validCount=0; issues={};
    for f=1:numel(faceNames)
        pts=paths.(faceNames{f}); n=size(pts,1); faceValid=0;
        for i=1:n
            [lx,ly,lz]=globalToLocal(pts(i,1),pts(i,2),pts(i,3),armIdxs(f),cfg);
            [ok,~]=checkReach(lx,ly,lz,cfg);
            if ok, faceValid=faceValid+1; end
        end
        totalCount=totalCount+n; validCount=validCount+faceValid;
        if faceValid<n
            issues{end+1}=sprintf('%s: %d/%d reachable',faceNames{f},faceValid,n); %#ok<AGROW>
        end
    end
end

%% ================================================================
%  EXECUTE PATH — NOW WITH LIVE PLOT UPDATES & TELEMETRY LOG
%% ================================================================
function telLog = executePath(ip, pathG, armIdx, cfg, fig, label)
% Returns telLog: Nx4 matrix [step, x_local, y_local, z_local]
    n = size(pathG, 1);
    logMsg(fig, sprintf('[%s] Starting %d waypoints', label, n));
    h = fig.UserData;

    telLog  = nan(n, 4);   % [step, lx, ly, lz]
    prevPos = nan(1, 3);
    skipped = 0;  failed = 0;

    for i = 1:n
        h = fig.UserData;
        if isfield(h,'stopFlag') && h.stopFlag
            logMsg(fig, sprintf('[%s] STOPPED at waypoint %d', label, i));
            telLog = telLog(1:i-1, :);
            return
        end

        [lx, ly, lz] = globalToLocal(pathG(i,1), pathG(i,2), pathG(i,3), armIdx, cfg);
        [ok, wMsg] = checkReach(lx, ly, lz, cfg);
        if ~ok
            skipped = skipped + 1;
            logMsg(fig, sprintf('  [skip %d/%d] %s', i, n, wMsg));
            continue
        end

        spd = h.eSpeed.Value;
        cmd = sprintf('{"T":104,"x":%d,"y":%d,"z":%d,"t":0,"spd":%.2f}', ...
            round(lx), round(ly), round(lz), spd);
        [~, success] = httpSend(ip, cfg.BASE_URL, cmd, cfg.HTTP_TO, cfg.MAX_RET, cfg.RET_DLY);

        if ~success
            failed = failed + 1;
            logMsg(fig, sprintf('  [FAIL %d/%d]', i, n));
            if failed >= 5
                logMsg(fig, sprintf('[%s] Too many failures — aborting', label));
                return
            end
            continue
        end

        waitForArrival(ip, [lx, ly, lz], cfg);

        % Read actual telemetry position
        data_live = httpReadStatus(ip, cfg.READ_TO);
        if ~isempty(data_live)
            [rx, ry, rz] = parseTel(data_live);
            telLog(i, :) = [i, rx, ry, rz];

            % Live plot update
            h = fig.UserData;
            globalStep = h.(sprintf('step%d', armIdx)) + 1;
            h.(sprintf('step%d', armIdx)) = globalStep;
            fig.UserData = h;

            addpoints(h.(sprintf('aln_p%dx', armIdx)), globalStep, rx);
            addpoints(h.(sprintf('aln_p%dy', armIdx)), globalStep, ry);
            addpoints(h.(sprintf('aln_p%dz', armIdx)), globalStep, rz);

            % Speed from consecutive telemetry readings
            if ~any(isnan(prevPos))
                spd_live = norm([rx-prevPos(1), ry-prevPos(2), rz-prevPos(3)]) / cfg.SETTLE_POLL;
                addpoints(h.(sprintf('aln_s%d', armIdx)), globalStep, spd_live);
            end
            prevPos = [rx, ry, rz];

            drawnow limitrate;
        end

        % 3D path dot
        if armIdx == 1
            plot3(h.ax, pathG(i,1), pathG(i,2), pathG(i,3), ...
                '.', 'Color',[0.15 0.55 1.0], 'MarkerSize', 5);
        else
            plot3(h.ax, pathG(i,1), pathG(i,2), pathG(i,3), ...
                '.', 'Color',[1.0 0.55 0.15], 'MarkerSize', 5);
        end
        drawnow limitrate;

        h = fig.UserData;
        h.progLbl.Text = sprintf('[%s] %d%%  (%d/%d)', label, round(i/n*100), i, n);
        fig.UserData = h;
    end

    telLog = telLog(~isnan(telLog(:,1)), :);
    logMsg(fig, sprintf('[%s] Complete — %d sent, %d skipped, %d failed', ...
        label, n-skipped-failed, skipped, failed));
end

%% ================================================================
%  POST-RUN TELEMETRY PLOT  (NEW)
%% ================================================================
function plotRoArmPostRun(telLog1, telLog2)
% telLog: Nx4 [step, lx, ly, lz]  in robot-local mm

    hasA1 = ~isempty(telLog1) && size(telLog1,1) > 1;
    hasA2 = ~isempty(telLog2) && size(telLog2,1) > 1;

    if ~hasA1 && ~hasA2
        fprintf('[PLOT] No RoArm telemetry data to plot.\n');
        return
    end

    fig = figure('Name','RoArm Post-Run Analysis', ...
        'Position',[60 60 1600 960], 'Color','w');
    sgtitle('RoArm Painting — Post-Run Telemetry', ...
        'FontSize', 14, 'FontWeight','bold');

    armLabels = {'ARM1','ARM2'};
    telLogs   = {telLog1, telLog2};
    colors    = {[0.1 0.5 1.0], [1.0 0.5 0.1]};

    for a = 1:2
        tl = telLogs{a};
        col = colors{a};
        if isempty(tl) || size(tl,1) < 2
            continue
        end

        steps = tl(:,1);
        lx    = tl(:,2);  ly = tl(:,3);  lz = tl(:,4);

        % Estimated velocity (finite difference)
        dt_est = 0.25;   % matches SETTLE_POLL
        vlx = [0; diff(lx)] / dt_est;
        vly = [0; diff(ly)] / dt_est;
        vlz = [0; diff(lz)] / dt_est;
        spd = sqrt(vlx.^2 + vly.^2 + vlz.^2);

        % NOTE: RoArm HTTP API (T:105) does not return torque.
        % Torque column would require custom firmware that exposes
        % motor current — shown as N/A in the plot.

        % ---- Position ----
        subplot(3, 4, (a-1)*1 + 1);
        plot(steps, lx, 'r-', 'LineWidth', 1.8); hold on;
        plot(steps, ly, 'g-', 'LineWidth', 1.8);
        plot(steps, lz, 'b-', 'LineWidth', 1.8);
        xlabel('Step'); ylabel('mm');
        title(sprintf('%s — Position (local)', armLabels{a}));
        legend('lx','ly','lz','Location','best'); grid on;

        % ---- 3D Path ----
        subplot(3, 4, (a-1)*1 + 3);
        scatter3(lx, ly, lz, 20, (1:length(lx)).', 'filled');
        colormap(gca, jet); colorbar;
        xlabel('lx'); ylabel('ly'); zlabel('lz');
        title(sprintf('%s — 3D Path', armLabels{a}));
        grid on; axis equal; view(42, 26);

        % ---- Velocity ----
        subplot(3, 4, (a-1)*1 + 5);
        plot(steps, vlx, 'r-', 'LineWidth', 1.6); hold on;
        plot(steps, vly, 'g-', 'LineWidth', 1.6);
        plot(steps, vlz, 'b-', 'LineWidth', 1.6);
        xlabel('Step'); ylabel('mm/s');
        title(sprintf('%s — Velocity (est.)', armLabels{a}));
        legend('Vlx','Vly','Vlz','Location','best'); grid on;

        % ---- Speed Magnitude ----
        subplot(3, 4, (a-1)*1 + 7);
        plot(steps, spd, 'Color', col, 'LineWidth', 2);
        xlabel('Step'); ylabel('mm/s');
        title(sprintf('%s — Speed |V|', armLabels{a}));
        grid on;

        % ---- Torque placeholder ----
        subplot(3, 4, (a-1)*1 + 9);
        axis off;
        text(0.5, 0.6, sprintf('%s\nTorque not available\nvia T:105 HTTP API\n\nRequires firmware\nextension for motor\ncurrent data.', ...
            armLabels{a}), ...
            'HorizontalAlignment','center', 'Units','normalized', ...
            'FontSize', 9, 'Color', [0.5 0.2 0.2]);
        title(sprintf('%s — Torque', armLabels{a}));

        % ---- Stats ----
        subplot(3, 4, (a-1)*1 + 11);
        axis off;
        statsStr = sprintf( ...
            ['%s Stats\n' ...
             'Waypoints:   %d\n' ...
             'lx range:    %.0f – %.0f mm\n' ...
             'ly range:    %.0f – %.0f mm\n' ...
             'lz range:    %.0f – %.0f mm\n' ...
             'Mean speed:  %.1f mm/s\n' ...
             'Peak speed:  %.1f mm/s\n' ...
             'Torque:      N/A (HTTP API)'], ...
            armLabels{a}, length(steps), ...
            min(lx), max(lx), min(ly), max(ly), min(lz), max(lz), ...
            mean(spd), max(spd));
        text(0.05, 0.95, statsStr, 'Units','normalized', ...
            'VerticalAlignment','top', 'FontName','Courier', 'FontSize', 8);
        title(sprintf('%s — Statistics', armLabels{a}));
    end

    fprintf('\n[PLOT] RoArm post-run analysis figure generated.\n');
end

%% ================================================================
%  VISUALIZATION  (unchanged)
%% ================================================================
function drawWorkspace(ax, cfg, C)
    tx=[0 cfg.TABLE_X cfg.TABLE_X 0]; ty=[0 0 cfg.TABLE_Y cfg.TABLE_Y];
    fill3(ax,tx,ty,zeros(1,4),[0.20 0.20 0.25],'FaceAlpha',0.22,'EdgeColor','none');
    plot3(ax,[tx tx(1)],[ty ty(1)],zeros(1,5),'Color',[0.38 0.38 0.45],'LineWidth',1.1);
    drawBox(ax,cfg.OBJ_CX-cfg.OBJ_DX/2,cfg.OBJ_CY-cfg.OBJ_DY/2,cfg.OBJ_Z0, ...
        cfg.OBJ_DX,cfg.OBJ_DY,cfg.OBJ_DZ,[0.92 0.68 0.18],0.38);
    text(ax,cfg.OBJ_CX,cfg.OBJ_CY,cfg.OBJ_DZ+15,'210x105x75mm', ...
        'Color',[1 0.85 0.3],'FontSize',7,'HorizontalAlignment','center');
    drawRobotArm(ax,cfg.B1(1),cfg.B1(2),0,C.acc1,0);
    drawRobotArm(ax,cfg.B2(1),cfg.B2(2),0,C.acc2,180);
    text(ax,cfg.B1(1)+12,cfg.B1(2)+25,80,'ARM 1','Color',C.acc1,'FontSize',9,'FontWeight','bold');
    text(ax,cfg.B2(1)+12,cfg.B2(2)-35,80,'ARM 2','Color',C.acc2,'FontSize',9,'FontWeight','bold');
    th=linspace(0,2*pi,64); r=cfg.MAX_REACH;
    plot3(ax,cfg.B1(1)+r*cos(th),cfg.B1(2)+r*sin(th),zeros(1,64),'--','Color',[C.acc1 0.35],'LineWidth',0.7);
    plot3(ax,cfg.B2(1)+r*cos(th),cfg.B2(2)+r*sin(th),zeros(1,64),'--','Color',[C.acc2 0.35],'LineWidth',0.7);
    axis(ax,'equal'); xlim(ax,[-70 cfg.TABLE_X+70]); ylim(ax,[-70 cfg.TABLE_Y+70]); zlim(ax,[0 280]);
end

function drawRobotArm(ax,x0,y0,z0,color,rotation_deg)
    theta=deg2rad(rotation_deg);
    [xc,yc,zc]=cylinder(25,20); zc=zc*54;
    for i=1:size(xc,1), for j=1:size(xc,2)
        xr=xc(i,j)*cos(theta)-yc(i,j)*sin(theta);
        yr=xc(i,j)*sin(theta)+yc(i,j)*cos(theta);
        xc(i,j)=xr+x0; yc(i,j)=yr+y0; zc(i,j)=zc(i,j)+z0;
    end; end
    surf(ax,xc,yc,zc,'FaceColor',color,'FaceAlpha',0.7,'EdgeColor','none');
    ls=[x0,y0,z0+54]; le=[x0+30*cos(theta),y0+30*sin(theta),z0+100];
    plot3(ax,[ls(1) le(1)],[ls(2) le(2)],[ls(3) le(3)],'LineWidth',8,'Color',color);
    l2e=[le(1)+70*cos(theta),le(2)+70*sin(theta),le(3)+20];
    plot3(ax,[le(1) l2e(1)],[le(2) l2e(2)],[le(3) l2e(3)],'LineWidth',7,'Color',color);
    l3e=[l2e(1)+50*cos(theta),l2e(2)+50*sin(theta),l2e(3)-10];
    plot3(ax,[l2e(1) l3e(1)],[l2e(2) l3e(2)],[l2e(3) l3e(3)],'LineWidth',5,'Color',color);
    [xg,yg,zg]=sphere(10);
    surf(ax,xg*12+l3e(1),yg*12+l3e(2),zg*12+l3e(3), ...
        'FaceColor',[0.9 0.9 0.9],'FaceAlpha',0.8,'EdgeColor','none');
    for pt=[ls; le; l2e]
        plot3(ax,pt(1),pt(2),pt(3),'o','MarkerSize',7, ...
            'MarkerFaceColor',[0.3 0.3 0.3],'Color','k');
    end
end

function drawBox(ax,x0,y0,z0,dx,dy,dz,col,alpha)
    V=[x0 y0 z0; x0+dx y0 z0; x0+dx y0+dy z0; x0 y0+dy z0;
       x0 y0 z0+dz; x0+dx y0 z0+dz; x0+dx y0+dy z0+dz; x0 y0+dy z0+dz];
    F=[1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch(ax,'Vertices',V,'Faces',F,'FaceColor',col,'FaceAlpha',alpha, ...
        'EdgeColor',[1 1 1],'LineWidth',0.65);
end

%% ================================================================
%  CALLBACKS
%% ================================================================
function cbMove(fig, armIdx, cfg)
    h = fig.UserData;
    if armIdx==1, gx=h.e1x.Value; gy=h.e1y.Value; gz=h.e1z.Value; ip=h.eIP1.Value; lblIK=h.lbl1; lblTel=h.tel1;
    else,         gx=h.e2x.Value; gy=h.e2y.Value; gz=h.e2z.Value; ip=h.eIP2.Value; lblIK=h.lbl2; lblTel=h.tel2; end
    [lx,ly,lz]=globalToLocal(gx,gy,gz,armIdx,cfg);
    [ok,wMsg]=checkReach(lx,ly,lz,cfg);
    lblIK.Text=wMsg;
    if ~ok, lblIK.FontColor=[1 0.3 0.3]; logMsg(fig,['  X ' wMsg]); return; end
    lblIK.FontColor=[0.1 0.9 0.4];
    spd=h.eSpeed.Value;
    cmd=sprintf('{"T":104,"x":%d,"y":%d,"z":%d,"t":0,"spd":%.2f}',round(lx),round(ly),round(lz),spd);
    [resp,success]=httpSend(ip,cfg.BASE_URL,cmd,cfg.HTTP_TO,cfg.MAX_RET,cfg.RET_DLY);
    if ~success, logMsg(fig,'  X Robot not responding!'); return; end
    logMsg(fig,sprintf('  Sent. Response: %s',strtrim(resp)));
    pause(0.5);
    data=httpReadStatus(ip,cfg.READ_TO);
    if ~isempty(data), [rx,ry,rz]=parseTel(data); lblTel.Text=sprintf('X=%.1f  Y=%.1f  Z=%.1f',rx,ry,rz); end
end

function cbPing(fig, cfg)
    h=fig.UserData; ips={h.eIP1.Value,h.eIP2.Value};
    arm1ok=false; arm2ok=false;
    for k=1:2
        data=httpReadStatus(ips{k},1.5);
        if ~isempty(data)
            [rx,ry,rz]=parseTel(data);
            logMsg(fig,sprintf('[ARM%d] OK X=%.1f Y=%.1f Z=%.1f',k,rx,ry,rz));
            if k==1,arm1ok=true;else,arm2ok=true;end
        else
            logMsg(fig,sprintf('[ARM%d] X NO RESPONSE (%s)',k,ips{k}));
        end
    end
    if arm1ok&&arm2ok,       h.progLbl.Text='OK Both Arms Connected';     h.progLbl.FontColor=[0.1 0.9 0.4];
    elseif arm1ok||arm2ok,   h.progLbl.Text='! Partial Connection';       h.progLbl.FontColor=[1 0.6 0];
    else,                    h.progLbl.Text='X Not Connected';             h.progLbl.FontColor=[1 0.3 0.3]; end
    fig.UserData=h;
end

function cbHome(fig, cfg)
    h=fig.UserData;
    httpSend(h.eIP1.Value,cfg.BASE_URL,'{"T":100}',cfg.HTTP_TO,cfg.MAX_RET,cfg.RET_DLY);
    httpSend(h.eIP2.Value,cfg.BASE_URL,'{"T":100}',cfg.HTTP_TO,cfg.MAX_RET,cfg.RET_DLY);
    logMsg(fig,'[System] HOME sent to both arms');
end

function homeOneArm(ip, cfg)
    httpSend(ip,cfg.BASE_URL,'{"T":100}',cfg.HTTP_TO,cfg.MAX_RET,cfg.RET_DLY);
end

function cbTel(fig, cfg)
    h=fig.UserData;
    for k=1:2
        if k==1, ip=h.eIP1.Value; lbl=h.tel1; else, ip=h.eIP2.Value; lbl=h.tel2; end
        data=httpReadStatus(ip,cfg.READ_TO);
        if ~isempty(data)
            [rx,ry,rz]=parseTel(data);
            lbl.Text=sprintf('X=%.1f  Y=%.1f  Z=%.1f',rx,ry,rz);
            logMsg(fig,sprintf('[ARM%d] X=%.1f Y=%.1f Z=%.1f',k,rx,ry,rz));
        else
            logMsg(fig,sprintf('[ARM%d] Read failed',k));
        end
    end
end

function cbStart(fig, cfg)
    h=fig.UserData; h.stopFlag=false; fig.UserData=h;
    ip1=h.eIP1.Value; ip2=h.eIP2.Value;

    logMsg(fig,'==============================');
    logMsg(fig,'  VALIDATING CONNECTION...');
    [connected,arm1ok,arm2ok]=validateConnection(ip1,ip2,2.0);
    if ~connected
        logMsg(fig,'X CONNECTION FAILED');
        if ~arm1ok, logMsg(fig,sprintf('  ARM1 (%s) offline',ip1)); end
        if ~arm2ok, logMsg(fig,sprintf('  ARM2 (%s) offline',ip2)); end
        uialert(fig,'Robots not connected. Click Ping to verify.','Connection Required','Icon','error');
        return
    end
    logMsg(fig,'OK Both connected');

    cfg.STRIPE_H=h.eStripe.Value; cfg.STANDOFF=h.eStandoff.Value; cfg.PAINT_SPD=h.eSpeed.Value;
    paths=generateAllPaths(cfg);
    [validCount,totalCount,issues]=validatePaths(paths,cfg);
    logMsg(fig,sprintf('  %d/%d waypoints reachable',validCount,totalCount));
    for k=1:numel(issues), logMsg(fig,sprintf('  WARNING: %s',issues{k})); end
    if validCount==0
        uialert(fig,'No reachable waypoints!','Path Error','Icon','error'); return
    end

    cbHome(fig,cfg); pause(2.0);

    % Reset telemetry logs and step counters
    h=fig.UserData; h.tel_log1=[]; h.tel_log2=[]; h.step1=0; h.step2=0; fig.UserData=h;

    logMsg(fig,'--- PHASE 1: ARM1 (Front + Right) ---');
    tl1a = executePath(ip1, paths.arm1_front, 1, cfg, fig, 'ARM1-Front');
    h=fig.UserData; if h.stopFlag, homeOneArm(ip1,cfg); cbStop(fig,cfg); return; end
    tl1b = executePath(ip1, paths.arm1_right, 1, cfg, fig, 'ARM1-Right');
    h=fig.UserData; if h.stopFlag, homeOneArm(ip1,cfg); cbStop(fig,cfg); return; end

    logMsg(fig,'--- ARM1 done. Sending HOME ---');
    homeOneArm(ip1,cfg); pause(2.0);

    logMsg(fig,'--- PHASE 2: ARM2 (Rear + Left + Top) ---');
    tl2a = executePath(ip2, paths.arm2_rear,  2, cfg, fig, 'ARM2-Rear');
    h=fig.UserData; if h.stopFlag, homeOneArm(ip2,cfg); cbStop(fig,cfg); return; end
    tl2b = executePath(ip2, paths.arm2_left,  2, cfg, fig, 'ARM2-Left');
    h=fig.UserData; if h.stopFlag, homeOneArm(ip2,cfg); cbStop(fig,cfg); return; end
    tl2c = executePath(ip2, paths.arm2_top,   2, cfg, fig, 'ARM2-Top');

    homeOneArm(ip2,cfg); pause(1.0);

    % Combine telemetry logs
    telLog1 = [tl1a; tl1b];
    telLog2 = [tl2a; tl2b; tl2c];

    h=fig.UserData;
    h.progLbl.Text='OK COMPLETE 100%'; h.progLbl.FontColor=[0.1 0.9 0.4];
    fig.UserData=h;

    logMsg(fig,'==============================');
    logMsg(fig,'  PAINTING COMPLETE — ALL 5 FACES');
    logMsg(fig,'  Generating telemetry plots...');
    logMsg(fig,'==============================');

    % Post-run plots
    plotRoArmPostRun(telLog1, telLog2);
end

function cbStop(fig, cfg)
    h=fig.UserData; h.stopFlag=true; fig.UserData=h;
    httpSend(h.eIP1.Value,cfg.BASE_URL,'{"T":0}',cfg.HTTP_TO,1,0.05);
    httpSend(h.eIP2.Value,cfg.BASE_URL,'{"T":0}',cfg.HTTP_TO,1,0.05);
    logMsg(fig,'EMERGENCY STOP sent');
end

function logMsg(fig, msg)
    h=fig.UserData;
    if ~isfield(h,'log')||~isvalid(h.log), return; end
    ts=datestr(now,'HH:MM:SS'); %#ok<TNOW1,DATST>
    cur=h.log.Value; if ischar(cur),cur={cur};end
    h.log.Value=[cur; {['[' ts ']  ' msg]}];
    try; scroll(h.log,'bottom'); catch; end
    drawnow limitrate;
end

function result = ternary(condition, trueVal, falseVal)
    if condition, result=trueVal; else, result=falseVal; end
end