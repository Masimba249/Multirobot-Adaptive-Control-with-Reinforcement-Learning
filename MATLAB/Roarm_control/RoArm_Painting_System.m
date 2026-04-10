%% ================================================================
%  RoArm_Painting_System.m
%  Waveshare RoArm-M2-S  |  Dual Robot Horizontal Painting GUI
%
%  TABLE  : 795 mm (X) x 595 mm (Y)
%  ARM 1  : Base at (397.5,   0) — faces +Y (inward)
%  ARM 2  : Base at (397.5, 595) — faces -Y (inward)
%  OBJECT : 105 x 210 x 75 mm, centred on table (397.5, 297.5)
%
%  RoArm-M2-S uses onboard IK via HTTP command T:104
%  MATLAB only validates reachability, then sends XYZ in local frame
%% ================================================================

function RoArm_Painting_System()
    clc; close all;
    cfg = buildConfig();
    buildMainGUI(cfg);
end

%% ================================================================
%  SECTION 1 – CONFIGURATION
%% ================================================================
function cfg = buildConfig()
    cfg.TABLE_X   = 795;
    cfg.TABLE_Y   = 595;
    cfg.B1        = [795/2,   0,   0];   % ARM1 base (front)
    cfg.B2        = [795/2, 595,   0];   % ARM2 base (rear)

    % RoArm-M2-S link lengths (mm)
    cfg.L0        = 54;    % base column to shoulder pivot
    cfg.L1        = 100;   % shoulder -> elbow
    cfg.L2        = 100;   % elbow -> wrist
    cfg.L3        = 80;    % wrist -> tool tip
    cfg.MAX_REACH = 275;   % max safe horizontal+vertical reach (mm)
    cfg.MIN_REACH = 40;    % min reach from base axis (mm)

    % Object dimensions (mm)
    cfg.OBJ_DX    = 105;
    cfg.OBJ_DY    = 210;
    cfg.OBJ_DZ    = 75;
    cfg.OBJ_CX    = 795/2;
    cfg.OBJ_CY    = 595/2;
    cfg.OBJ_Z0    = 0;

    % Painting defaults
    cfg.STANDOFF  = 25;
    cfg.STRIPE_H  = 15;
    cfg.APPROACH  = 35;
    cfg.PAINT_SPD = 3.5;

    % Network defaults
    cfg.IP1       = '192.168.1.192';
    cfg.IP2       = '192.168.1.101';
    cfg.BASE_URL  = '/js?json=';
    cfg.HTTP_TO   = 2.5;
    cfg.MAX_RET   = 2;
    cfg.RET_DLY   = 0.20;
    cfg.POLL_INT  = 0.18;

    % Z limits
    cfg.Z_MIN     = 10;
    cfg.Z_MAX     = 334;
end

%% ================================================================
%  SECTION 2 – MAIN GUI BUILDER
%% ================================================================
function buildMainGUI(cfg)
    % Colour palette
    C.bg    = [0.12 0.12 0.15];
    C.panel = [0.18 0.18 0.22];
    C.acc1  = [0.10 0.55 1.00];
    C.acc2  = [1.00 0.50 0.10];
    C.grn   = [0.10 0.88 0.42];
    C.txt   = [0.93 0.93 0.93];
    C.dark  = [0.08 0.08 0.10];

    fig = uifigure( ...
        'Name',    'Dual RoArm-M2-S | Painting Controller', ...
        'Position',[60 50 1100 720], ...
        'Color',    C.bg, ...
        'Resize',  'off');

    % Title bar
    uilabel(fig, ...
        'Text','  Dual Waveshare RoArm-M2-S  —  Painting Controller', ...
        'Position',[0 678 1100 38], ...
        'FontSize',17,'FontWeight','bold', ...
        'FontColor',C.acc1,'BackgroundColor',C.bg, ...
        'HorizontalAlignment','center');

    % ARM 1 panel
    [h.e1x, h.e1y, h.e1z, h.btn1Move, h.lbl1IK, h.lbl1Tel] = ...
        makeArmPanel(fig, 'ARM 1  (Front – faces +Y)', ...
        [10 385 330 285], C, C.acc1, cfg);

    % ARM 2 panel
    [h.e2x, h.e2y, h.e2z, h.btn2Move, h.lbl2IK, h.lbl2Tel] = ...
        makeArmPanel(fig, 'ARM 2  (Rear – faces -Y)', ...
        [350 385 330 285], C, C.acc2, cfg);

    % Config panel (edit fields created here, buttons wired later)
    h = makeConfigPanel(fig, h, C, cfg, [690 385 400 285]);

    % 3-D preview axes
    h.ax = uiaxes(fig,'Position',[10 10 570 365], ...
        'Color',C.dark,'XColor',C.txt,'YColor',C.txt,'ZColor',C.txt, ...
        'GridColor',[0.28 0.28 0.32],'FontSize',8);
    hold(h.ax,'on'); grid(h.ax,'on');
    xlabel(h.ax,'Global X (mm)');
    ylabel(h.ax,'Global Y (mm)');
    zlabel(h.ax,'Z (mm)');
    title(h.ax,'3D Workspace Preview','Color',C.txt,'FontSize',10);
    view(h.ax,40,28);
    drawWorkspace(h.ax, cfg, C);

    % Log panel
    logPan = uipanel(fig,'Title','Event Log', ...
        'Position',[590 10 500 365], ...
        'BackgroundColor',C.panel,'ForegroundColor',C.txt, ...
        'FontSize',9,'BorderType','line');
    h.log = uitextarea(logPan,'Position',[4 4 487 337], ...
        'BackgroundColor',C.dark,'FontColor',[0.6 1.0 0.6], ...
        'FontSize',8,'Editable','off', ...
        'Value',{'[System] RoArm-M2-S Painting GUI ready.'});

    % Progress label
    uilabel(fig,'Text','Progress:', ...
        'Position',[10 672 70 22], ...
        'FontColor',C.txt,'BackgroundColor',C.bg,'FontSize',9);
    h.progLbl = uilabel(fig,'Text','  —', ...
        'Position',[82 672 260 22], ...
        'FontColor',C.grn,'BackgroundColor',C.bg, ...
        'FontSize',10,'FontWeight','bold');

    % ---- Store ALL handles now (h.log exists) ----
    fig.UserData = h;

    % ---- Wire Move buttons (fig is a handle — safe to capture) ----
    h.btn1Move.ButtonPushedFcn = @(~,~) cbMoveArm(fig, 1, cfg);
    h.btn2Move.ButtonPushedFcn = @(~,~) cbMoveArm(fig, 2, cfg);

    % ---- Wire config-panel buttons via tagged lookup ----
    rewireConfigButtons(fig, cfg);

    % ---- Flush final state ----
    fig.UserData = h;
end

%% ================================================================
%  ARM PANEL FACTORY
%% ================================================================
function [ex,ey,ez,btnMove,lblIK,lblTel] = makeArmPanel( ...
        fig, titleStr, pos, C, acc, cfg)

    pan = uipanel(fig,'Title',titleStr, ...
        'Position',pos, ...
        'BackgroundColor',C.panel,'ForegroundColor',acc, ...
        'FontSize',10,'FontWeight','bold','BorderType','line');

    row = @(n) 225 - n*38;
    lw = 90; ew = 105;

    uilabel(pan,'Text','X global (mm):', ...
        'Position',[8 row(0) lw 22], ...
        'FontColor',C.txt,'BackgroundColor',C.panel,'FontSize',9);
    ex = uieditfield(pan,'numeric', ...
        'Position',[lw+10 row(0) ew 24], ...
        'Value',cfg.OBJ_CX, ...
        'Limits',[-60 cfg.TABLE_X+60], ...
        'BackgroundColor',C.dark,'FontColor',C.txt,'FontSize',10);

    uilabel(pan,'Text','Y global (mm):', ...
        'Position',[8 row(1) lw 22], ...
        'FontColor',C.txt,'BackgroundColor',C.panel,'FontSize',9);
    ey = uieditfield(pan,'numeric', ...
        'Position',[lw+10 row(1) ew 24], ...
        'Value',cfg.OBJ_CY, ...
        'Limits',[-60 cfg.TABLE_Y+60], ...
        'BackgroundColor',C.dark,'FontColor',C.txt,'FontSize',10);

    uilabel(pan,'Text','Z height (mm):', ...
        'Position',[8 row(2) lw 22], ...
        'FontColor',C.txt,'BackgroundColor',C.panel,'FontSize',9);
    ez = uieditfield(pan,'numeric', ...
        'Position',[lw+10 row(2) ew 24], ...
        'Value',cfg.OBJ_DZ/2, ...
        'Limits',[0 350], ...
        'BackgroundColor',C.dark,'FontColor',C.txt,'FontSize',10);

    btnMove = uibutton(pan,'Text','  Move to XYZ', ...
        'Position',[8 row(3) 205 30], ...
        'BackgroundColor',acc,'FontColor','k', ...
        'FontSize',10,'FontWeight','bold');

    lblIK = uilabel(pan,'Text','Workspace check: waiting', ...
        'Position',[8 row(4) 308 20], ...
        'FontColor',[0.5 0.9 0.5],'BackgroundColor',C.panel, ...
        'FontSize',8,'WordWrap','on');

    uilabel(pan,'Text','Live pos:', ...
        'Position',[8 row(5) 60 18], ...
        'FontColor',[0.55 0.55 0.65],'BackgroundColor',C.panel,'FontSize',8);
    lblTel = uilabel(pan,'Text','X=—  Y=—  Z=—', ...
        'Position',[70 row(5) 255 18], ...
        'FontColor',[0.65 0.90 1.0],'BackgroundColor',C.panel,'FontSize',8);
end

%% ================================================================
%  CONFIG PANEL FACTORY  — edit fields only; buttons wired later
%% ================================================================
function h = makeConfigPanel(fig, h, C, cfg, pos)
    pan = uipanel(fig,'Title','Configuration & Control', ...
        'Position',pos, ...
        'BackgroundColor',C.panel,'ForegroundColor',C.txt, ...
        'FontSize',10,'FontWeight','bold','BorderType','line', ...
        'Tag','cfgPanel');

    lw = 135; ew = 80;
    row = @(n) 240 - n*36;

    % --- Edit fields ---
    uilabel(pan,'Text','Stripe height (mm):', ...
        'Position',[8 row(0) lw 20], ...
        'FontColor',C.txt,'BackgroundColor',C.panel,'FontSize',9);
    h.eStripe = uieditfield(pan,'numeric', ...
        'Position',[lw+5 row(0) ew 22],'Value',cfg.STRIPE_H, ...
        'Limits',[5 40],'BackgroundColor',C.dark,'FontColor',C.txt);

    uilabel(pan,'Text','Standoff dist (mm):', ...
        'Position',[8 row(1) lw 20], ...
        'FontColor',C.txt,'BackgroundColor',C.panel,'FontSize',9);
    h.eStandoff = uieditfield(pan,'numeric', ...
        'Position',[lw+5 row(1) ew 22],'Value',cfg.STANDOFF, ...
        'Limits',[10 80],'BackgroundColor',C.dark,'FontColor',C.txt);

    uilabel(pan,'Text','Paint speed (1-20):', ...
        'Position',[8 row(2) lw 20], ...
        'FontColor',C.txt,'BackgroundColor',C.panel,'FontSize',9);
    h.eSpeed = uieditfield(pan,'numeric', ...
        'Position',[lw+5 row(2) ew 22],'Value',cfg.PAINT_SPD, ...
        'Limits',[1 20],'BackgroundColor',C.dark,'FontColor',C.txt);

    uilabel(pan,'Text','ARM 1 IP:', ...
        'Position',[8 row(3) lw 20], ...
        'FontColor',C.txt,'BackgroundColor',C.panel,'FontSize',9);
    h.eIP1 = uieditfield(pan,'text', ...
        'Position',[lw+5 row(3) 135 22],'Value',cfg.IP1, ...
        'BackgroundColor',C.dark,'FontColor',C.txt);

    uilabel(pan,'Text','ARM 2 IP:', ...
        'Position',[8 row(4) lw 20], ...
        'FontColor',C.txt,'BackgroundColor',C.panel,'FontSize',9);
    h.eIP2 = uieditfield(pan,'text', ...
        'Position',[lw+5 row(4) 135 22],'Value',cfg.IP2, ...
        'BackgroundColor',C.dark,'FontColor',C.txt);

    % --- Buttons (tagged; callbacks wired in rewireConfigButtons) ---
    uibutton(pan,'Text','Ping Arms', ...
        'Position',[8 row(5) 110 26], ...
        'BackgroundColor',[0.2 0.45 0.7],'FontColor','w','FontSize',9, ...
        'Tag','btnPing');

    uibutton(pan,'Text','HOME Both', ...
        'Position',[124 row(5) 110 26], ...
        'BackgroundColor',[0.30 0.30 0.55],'FontColor','w','FontSize',9, ...
        'Tag','btnHome');

    uibutton(pan,'Text','Read Telemetry', ...
        'Position',[240 row(5) 130 26], ...
        'BackgroundColor',[0.18 0.40 0.18],'FontColor','w','FontSize',9, ...
        'Tag','btnTel');

    uibutton(pan,'Text','START PAINTING', ...
        'Position',[8 row(6)-4 362 36], ...
        'BackgroundColor',[0.08 0.80 0.35], ...
        'FontColor',[0 0.05 0],'FontSize',13,'FontWeight','bold', ...
        'Tag','btnStart');

    uibutton(pan,'Text','EMERGENCY STOP', ...
        'Position',[8 5 362 30], ...
        'BackgroundColor',[0.85 0.15 0.15], ...
        'FontColor','w','FontSize',11,'FontWeight','bold', ...
        'Tag','btnStop');
end

%% ================================================================
%  REWIRE CONFIG BUTTONS
%  findobj result MUST be stored in a variable before property assign
%% ================================================================
function rewireConfigButtons(fig, cfg)
    btn = findobj(fig, 'Tag', 'btnPing');
    btn.ButtonPushedFcn = @(~,~) cbPing(fig, cfg);

    btn = findobj(fig, 'Tag', 'btnHome');
    btn.ButtonPushedFcn = @(~,~) cbHome(fig, cfg);

    btn = findobj(fig, 'Tag', 'btnTel');
    btn.ButtonPushedFcn = @(~,~) cbReadTelemetry(fig, cfg);

    btn = findobj(fig, 'Tag', 'btnStart');
    btn.ButtonPushedFcn = @(~,~) cbStartPainting(fig, cfg);

    btn = findobj(fig, 'Tag', 'btnStop');
    btn.ButtonPushedFcn = @(~,~) cbStop(fig, cfg);
end

%% ================================================================
%  SECTION 3 – WORKSPACE VALIDATION
%  RoArm-M2-S handles IK internally (T:104).
%  MATLAB only checks whether the point is physically reachable.
%% ================================================================
function [ok, msg] = checkReachability(lx, ly, lz, cfg)
    r      = sqrt(lx^2 + ly^2);
    zR     = lz - cfg.L0;
    dist3D = sqrt(r^2 + zR^2);

    if dist3D > cfg.MAX_REACH
        ok  = false;
        msg = sprintf('OUT OF REACH: dist=%.1f mm  (max %.0f mm)', ...
            dist3D, cfg.MAX_REACH);
        return
    end
    if r < cfg.MIN_REACH
        ok  = false;
        msg = sprintf('TOO CLOSE to base: r=%.1f mm  (min %d mm)', ...
            r, cfg.MIN_REACH);
        return
    end
    if lz < cfg.Z_MIN || lz > cfg.Z_MAX
        ok  = false;
        msg = sprintf('Z=%.1f out of range [%d, %d] mm', ...
            lz, cfg.Z_MIN, cfg.Z_MAX);
        return
    end
    ok  = true;
    msg = sprintf('OK  r=%.1f mm  3D=%.1f mm', r, dist3D);
end

%% ================================================================
%  SECTION 4 – COORDINATE TRANSFORM  (global table -> robot local)
%
%  ARM1 base at (TABLE_X/2, 0), faces +Y:
%    local_X (forward) = gy - 0
%    local_Y (lateral) = gx - TABLE_X/2
%
%  ARM2 base at (TABLE_X/2, TABLE_Y), faces -Y:
%    local_X (forward) = TABLE_Y - gy
%    local_Y (lateral) = gx - TABLE_X/2
%% ================================================================
function [lx, ly, lz] = globalToLocal(gx, gy, gz, armIdx, cfg)
    if armIdx == 1
        lx = gy - cfg.B1(2);
        ly = gx - cfg.B1(1);
    else
        lx = cfg.B2(2) - gy;
        ly = gx - cfg.B2(1);
    end
    lz = gz;
end

%% ================================================================
%  SECTION 5 – PAINTING PATH GENERATION  (global coordinates)
%% ================================================================
function paths = generateAllPaths(cfg)
    so  = cfg.STANDOFF;
    sH  = cfg.STRIPE_H;
    app = cfg.APPROACH;
    cx  = cfg.OBJ_CX;
    cy  = cfg.OBJ_CY;
    dx  = cfg.OBJ_DX / 2;
    dy  = cfg.OBJ_DY / 2;
    dz  = cfg.OBJ_DZ;
    z0  = cfg.OBJ_Z0;

    zBot  = z0 + 5;
    zTop_ = z0 + dz - 5;
    nStrZ = max(1, ceil((zTop_ - zBot) / sH));
    nStrX = max(1, ceil(cfg.OBJ_DX / sH));

    %% Front face – ARM1 (min-Y face, closest to ARM1)
    yFront = cy - dy - so;
    front1 = zeros(nStrZ*4, 3);
    for i = 1:nStrZ
        z = min(zBot + (i-1)*sH, zTop_);
        r = (i-1)*4 + 1;
        front1(r  ,:) = [cx-dx-app, yFront, z];
        front1(r+1,:) = [cx-dx,     yFront, z];
        front1(r+2,:) = [cx+dx,     yFront, z];
        front1(r+3,:) = [cx+dx+app, yFront, z];
    end

    %% Rear face – ARM2 (max-Y face, closest to ARM2)
    yRear  = cy + dy + so;
    front2 = zeros(nStrZ*4, 3);
    for i = 1:nStrZ
        z = min(zBot + (i-1)*sH, zTop_);
        r = (i-1)*4 + 1;
        front2(r  ,:) = [cx-dx-app, yRear, z];
        front2(r+1,:) = [cx-dx,     yRear, z];
        front2(r+2,:) = [cx+dx,     yRear, z];
        front2(r+3,:) = [cx+dx+app, yRear, z];
    end

    %% Top face – ARM1 paints front half (Y: cy-dy -> cy)
    zTopFace = z0 + dz + so;
    top1 = zeros(nStrX*4, 3);
    for i = 1:nStrX
        x = min((cx-dx) + (i-1)*sH, cx+dx);
        r = (i-1)*4 + 1;
        top1(r  ,:) = [x, cy-dy-app,  zTopFace];
        top1(r+1,:) = [x, cy-dy,      zTopFace];
        top1(r+2,:) = [x, cy,         zTopFace];
        top1(r+3,:) = [x, cy+app/2,   zTopFace];
    end

    %% Top face – ARM2 paints rear half (Y: cy -> cy+dy)
    top2 = zeros(nStrX*4, 3);
    for i = 1:nStrX
        x = min((cx-dx) + (i-1)*sH, cx+dx);
        r = (i-1)*4 + 1;
        top2(r  ,:) = [x, cy-app/2,  zTopFace];
        top2(r+1,:) = [x, cy,        zTopFace];
        top2(r+2,:) = [x, cy+dy,     zTopFace];
        top2(r+3,:) = [x, cy+dy+app, zTopFace];
    end

    %% Left face – ARM1 (min-X side)
    xLeft = cx - dx - so;
    left1 = zeros(nStrZ*4, 3);
    for i = 1:nStrZ
        z = min(zBot + (i-1)*sH, zTop_);
        r = (i-1)*4 + 1;
        left1(r  ,:) = [xLeft, cy-dy-app, z];
        left1(r+1,:) = [xLeft, cy-dy,     z];
        left1(r+2,:) = [xLeft, cy+dy,     z];
        left1(r+3,:) = [xLeft, cy+dy+app, z];
    end

    %% Right face – ARM2 (max-X side)
    xRight = cx + dx + so;
    right2 = zeros(nStrZ*4, 3);
    for i = 1:nStrZ
        z = min(zBot + (i-1)*sH, zTop_);
        r = (i-1)*4 + 1;
        right2(r  ,:) = [xRight, cy-dy-app, z];
        right2(r+1,:) = [xRight, cy-dy,     z];
        right2(r+2,:) = [xRight, cy+dy,     z];
        right2(r+3,:) = [xRight, cy+dy+app, z];
    end

    paths.front1 = front1;
    paths.front2 = front2;
    paths.top1   = top1;
    paths.top2   = top2;
    paths.left1  = left1;
    paths.right2 = right2;
end

%% ================================================================
%  SECTION 6 – HTTP COMMUNICATION
%% ================================================================
function resp = httpSend(ip, baseURL, jsonStr, timeout, maxRet, retDly)
    url  = ['http://' ip baseURL urlencode(jsonStr)];
    resp = '';
    for k = 1:maxRet
        try
            resp = webread(url, weboptions('Timeout', timeout));
            return;
        catch
            if k < maxRet
                pause(retDly);
            end
        end
    end
end

function data = httpRead(ip, timeout)
    cmd = '{"T":105}';
    url = ['http://' ip '/js?json=' urlencode(cmd)];
    data = [];
    try
        raw = webread(url, weboptions('Timeout', timeout));
        s = strfind(raw, '{');
        e = strfind(raw, '}');
        if ~isempty(s) && ~isempty(e)
            data = jsondecode(raw(s(1):e(end)));
        end
    catch
    end
end

function [x, y, z] = parseTelemetry(data)
    x = extractField(data, {'x','X','px'});
    y = extractField(data, {'y','Y','py'});
    z = extractField(data, {'z','Z','pz'});
end

function v = extractField(s, names)
    v = 0;
    if isempty(s) || ~isstruct(s), return; end
    for k = 1:numel(names)
        if isfield(s, names{k})
            try
                v = double(s.(names{k}));
                return;
            catch
            end
        end
    end
end

%% ================================================================
%  SECTION 7 – EXECUTE ONE PATH  (single arm, sequential)
%% ================================================================
function executePath(ip, pathGlobal, armIdx, cfg, fig, label)
    n = size(pathGlobal, 1);
    logMsg(fig, sprintf('[%s] Starting — %d waypoints', label, n));

    for i = 1:n
        % Always re-fetch handles (GUI values may have changed)
        h = fig.UserData;

        gx = pathGlobal(i,1);
        gy = pathGlobal(i,2);
        gz = pathGlobal(i,3);

        % Convert to robot-local frame
        [lx, ly, lz] = globalToLocal(gx, gy, gz, armIdx, cfg);

        % Validate reachability
        [ok, wMsg] = checkReachability(lx, ly, lz, cfg);
        if ~ok
            logMsg(fig, sprintf('  [skip %d] %s', i, wMsg));
            continue
        end

        % Build T:104 command — RoArm onboard IK takes over
        spd = h.eSpeed.Value;
        cmd = sprintf( ...
            '{"T":104,"x":%.1f,"y":%.1f,"z":%.1f,"t":0,"spd":%.1f}', ...
            lx, ly, lz, spd);
        httpSend(ip, cfg.BASE_URL, cmd, cfg.HTTP_TO, cfg.MAX_RET, cfg.RET_DLY);

        % Plot waypoint on 3-D preview
        if armIdx == 1
            plot3(h.ax, gx, gy, gz, '.', ...
                'Color',[0.15 0.55 1.0], 'MarkerSize',5);
        else
            plot3(h.ax, gx, gy, gz, '.', ...
                'Color',[1.0 0.55 0.15], 'MarkerSize',5);
        end
        drawnow limitrate;

        % Update progress label
        pct = round(i/n*100);
        h.progLbl.Text = sprintf('[%s]  %d / %d  (%d%%)', label, i, n, pct);

        pause(cfg.POLL_INT);
    end

    logMsg(fig, sprintf('[%s] Done.', label));
end

%% ================================================================
%  SECTION 8 – 3-D WORKSPACE DRAWING
%% ================================================================
function drawWorkspace(ax, cfg, C)
    % Table surface
    tx = [0 cfg.TABLE_X cfg.TABLE_X 0];
    ty = [0 0 cfg.TABLE_Y cfg.TABLE_Y];
    fill3(ax, tx, ty, zeros(1,4), [0.22 0.22 0.28], ...
        'FaceAlpha',0.25,'EdgeColor','none');
    plot3(ax, [tx tx(1)], [ty ty(1)], zeros(1,5), ...
        'Color',[0.4 0.4 0.5],'LineWidth',1.2);

    % Object bounding box
    drawBox3D(ax, ...
        cfg.OBJ_CX - cfg.OBJ_DX/2, ...
        cfg.OBJ_CY - cfg.OBJ_DY/2, ...
        cfg.OBJ_Z0, ...
        cfg.OBJ_DX, cfg.OBJ_DY, cfg.OBJ_DZ, ...
        [0.95 0.70 0.20], 0.40);

    text(ax, cfg.OBJ_CX, cfg.OBJ_CY, cfg.OBJ_DZ+15, ...
        sprintf('Object\n%.0fx%.0fx%.0f mm', ...
        cfg.OBJ_DX, cfg.OBJ_DY, cfg.OBJ_DZ), ...
        'Color',[1 0.85 0.3],'FontSize',7, ...
        'HorizontalAlignment','center');

    % Robot base markers
    plot3(ax, cfg.B1(1), cfg.B1(2), 0, 'o', ...
        'MarkerSize',12,'MarkerFaceColor',C.acc1,'Color',C.acc1, ...
        'DisplayName','ARM 1 Base');
    plot3(ax, cfg.B2(1), cfg.B2(2), 0, 'o', ...
        'MarkerSize',12,'MarkerFaceColor',C.acc2,'Color',C.acc2, ...
        'DisplayName','ARM 2 Base');

    text(ax, cfg.B1(1)+15, cfg.B1(2)+20, 15, 'ARM 1', ...
        'Color',C.acc1,'FontSize',9,'FontWeight','bold');
    text(ax, cfg.B2(1)+15, cfg.B2(2)-30, 15, 'ARM 2', ...
        'Color',C.acc2,'FontSize',9,'FontWeight','bold');

    % Max-reach circles (top view)
    th = linspace(0, 2*pi, 72);
    r  = cfg.MAX_REACH;
    plot3(ax, cfg.B1(1)+r*cos(th), cfg.B1(2)+r*sin(th), zeros(1,72), ...
        '--','Color',[C.acc1 0.4],'LineWidth',0.8);
    plot3(ax, cfg.B2(1)+r*cos(th), cfg.B2(2)+r*sin(th), zeros(1,72), ...
        '--','Color',[C.acc2 0.4],'LineWidth',0.8);

    axis(ax,'equal');
    xlim(ax,[-60 cfg.TABLE_X+60]);
    ylim(ax,[-60 cfg.TABLE_Y+60]);
    zlim(ax,[0 300]);
    legend(ax,'Location','northeast','TextColor',C.txt);
end

function drawBox3D(ax, x0,y0,z0, dx,dy,dz, col, alpha)
    V = [x0    y0    z0;
         x0+dx y0    z0;
         x0+dx y0+dy z0;
         x0    y0+dy z0;
         x0    y0    z0+dz;
         x0+dx y0    z0+dz;
         x0+dx y0+dy z0+dz;
         x0    y0+dy z0+dz];
    F = [1 2 3 4;
         5 6 7 8;
         1 2 6 5;
         2 3 7 6;
         3 4 8 7;
         4 1 5 8];
    patch(ax,'Vertices',V,'Faces',F, ...
        'FaceColor',col,'FaceAlpha',alpha, ...
        'EdgeColor',[1 1 1],'LineWidth',0.7);
end

%% ================================================================
%  SECTION 9 – CALLBACKS
%  Every callback receives fig as first arg and reads fig.UserData
%  to get the live handle struct.
%% ================================================================

% ----------------------------------------------------------------
function cbMoveArm(fig, armIdx, cfg)
    h = fig.UserData;

    if armIdx == 1
        gx = h.e1x.Value;  gy = h.e1y.Value;  gz = h.e1z.Value;
        ip     = h.eIP1.Value;
        lblIK  = h.lbl1IK;
        lblTel = h.lbl1Tel;
    else
        gx = h.e2x.Value;  gy = h.e2y.Value;  gz = h.e2z.Value;
        ip     = h.eIP2.Value;
        lblIK  = h.lbl2IK;
        lblTel = h.lbl2Tel;
    end

    logMsg(fig, sprintf('[ARM%d] Move -> X=%.1f  Y=%.1f  Z=%.1f mm (global)', ...
        armIdx, gx, gy, gz));

    [lx, ly, lz] = globalToLocal(gx, gy, gz, armIdx, cfg);
    [ok, wMsg]   = checkReachability(lx, ly, lz, cfg);
    lblIK.Text   = wMsg;

    if ~ok
        lblIK.FontColor = [1 0.3 0.3];
        logMsg(fig, ['  FAIL: ' wMsg]);
        return
    end

    lblIK.FontColor = [0.1 0.9 0.4];
    logMsg(fig, sprintf('  Local frame: lx=%.1f  ly=%.1f  lz=%.1f', lx, ly, lz));

    spd = h.eSpeed.Value;
    cmd = sprintf( ...
        '{"T":104,"x":%.1f,"y":%.1f,"z":%.1f,"t":0,"spd":%.1f}', ...
        lx, ly, lz, spd);
    resp = httpSend(ip, cfg.BASE_URL, cmd, cfg.HTTP_TO, cfg.MAX_RET, cfg.RET_DLY);
    logMsg(fig, sprintf('  Sent OK. Response: %s', strtrim(resp)));

    % Mark target on 3-D preview
    drawArmTarget(h.ax, gx, gy, gz, armIdx);

    % Read back live position
    pause(0.5);
    data = httpRead(ip, cfg.HTTP_TO);
    if ~isempty(data)
        [rx,ry,rz] = parseTelemetry(data);
        lblTel.Text = sprintf('X=%.1f  Y=%.1f  Z=%.1f', rx, ry, rz);
    end
end

% ----------------------------------------------------------------
function cbPing(fig, cfg)
    h   = fig.UserData;
    ips = {h.eIP1.Value, h.eIP2.Value};
    for k = 1:2
        data = httpRead(ips{k}, 1.5);
        if ~isempty(data)
            [rx,ry,rz] = parseTelemetry(data);
            logMsg(fig, sprintf('[ARM%d] ONLINE  X=%.1f Y=%.1f Z=%.1f', ...
                k, rx, ry, rz));
        else
            logMsg(fig, sprintf('[ARM%d] NO RESPONSE  (%s)', k, ips{k}));
        end
    end
end

% ----------------------------------------------------------------
function cbHome(fig, cfg)
    h = fig.UserData;
    httpSend(h.eIP1.Value, cfg.BASE_URL, '{"T":100}', ...
        cfg.HTTP_TO, cfg.MAX_RET, cfg.RET_DLY);
    httpSend(h.eIP2.Value, cfg.BASE_URL, '{"T":100}', ...
        cfg.HTTP_TO, cfg.MAX_RET, cfg.RET_DLY);
    logMsg(fig, '[System] HOME sent to both arms.');
end

% ----------------------------------------------------------------
function cbReadTelemetry(fig, cfg)
    h = fig.UserData;
    ips  = {h.eIP1.Value,  h.eIP2.Value};
    lbls = {h.lbl1Tel,     h.lbl2Tel   };
    for k = 1:2
        data = httpRead(ips{k}, cfg.HTTP_TO);
        if ~isempty(data)
            [rx,ry,rz] = parseTelemetry(data);
            lbls{k}.Text = sprintf('X=%.1f  Y=%.1f  Z=%.1f', rx, ry, rz);
            logMsg(fig, sprintf('[ARM%d] X=%.1f  Y=%.1f  Z=%.1f', k, rx, ry, rz));
        else
            logMsg(fig, sprintf('[ARM%d] Telemetry read failed.', k));
        end
    end
end

% ----------------------------------------------------------------
function cbStartPainting(fig, cfg)
    h = fig.UserData;

    % Read live GUI values
    cfg.STRIPE_H  = h.eStripe.Value;
    cfg.STANDOFF  = h.eStandoff.Value;
    cfg.PAINT_SPD = h.eSpeed.Value;
    ip1 = h.eIP1.Value;
    ip2 = h.eIP2.Value;

    logMsg(fig, '================================================');
    logMsg(fig, '  PAINTING SEQUENCE STARTED');
    logMsg(fig, sprintf('  Stripe=%.0fmm  Standoff=%.0fmm  Speed=%.1f', ...
        cfg.STRIPE_H, cfg.STANDOFF, cfg.PAINT_SPD));
    logMsg(fig, '================================================');

    % Generate all waypoint paths
    paths = generateAllPaths(cfg);
    logMsg(fig, sprintf( ...
        'Paths: F1=%d  F2=%d  T1=%d  T2=%d  L=%d  R=%d pts', ...
        size(paths.front1,1), size(paths.front2,1), ...
        size(paths.top1,1),   size(paths.top2,1), ...
        size(paths.left1,1),  size(paths.right2,1)));

    % Home arms before starting
    cbHome(fig, cfg);
    pause(2.0);

    % Execution sequence [path, armIdx, ip, label]
    seq = { ...
        paths.front1, 1, ip1, 'ARM1-FrontFace';
        paths.front2, 2, ip2, 'ARM2-RearFace';
        paths.top1,   1, ip1, 'ARM1-TopFront';
        paths.top2,   2, ip2, 'ARM2-TopRear';
        paths.left1,  1, ip1, 'ARM1-LeftFace';
        paths.right2, 2, ip2, 'ARM2-RightFace'};

    for fi = 1:size(seq, 1)
        executePath(seq{fi,3}, seq{fi,1}, seq{fi,2}, cfg, fig, seq{fi,4});
        pause(0.5);
    end

    % Return home
    cbHome(fig, cfg);

    % Update progress label via UserData (always re-fetch)
    h = fig.UserData;
    h.progLbl.Text = 'COMPLETE  100%';
    fig.UserData   = h;

    logMsg(fig, 'PAINTING COMPLETE. Both arms homed.');
end

% ----------------------------------------------------------------
function cbStop(fig, cfg)
    h = fig.UserData;
    httpSend(h.eIP1.Value, cfg.BASE_URL, '{"T":0}', cfg.HTTP_TO, 1, 0.05);
    httpSend(h.eIP2.Value, cfg.BASE_URL, '{"T":0}', cfg.HTTP_TO, 1, 0.05);
    logMsg(fig, 'EMERGENCY STOP sent to both arms.');
end

%% ================================================================
%  SECTION 10 – UTILITY HELPERS
%% ================================================================
function logMsg(fig, msg)
    h = fig.UserData;
    if ~isfield(h,'log') || ~isvalid(h.log), return; end
    ts  = datestr(now,'HH:MM:SS'); %#ok<TNOW1,DATST>
    cur = h.log.Value;
    if ischar(cur), cur = {cur}; end
    h.log.Value = [cur; {['[' ts ']  ' msg]}];
    try; scroll(h.log,'bottom'); catch; end
    drawnow limitrate;
end

function drawArmTarget(ax, gx, gy, gz, armIdx)
    tag = sprintf('armTarget%d', armIdx);
    old = findobj(ax, 'Tag', tag);
    if ~isempty(old), delete(old); end
    col = [0.1 0.55 1.0];
    if armIdx == 2, col = [1.0 0.55 0.1]; end
    plot3(ax, gx, gy, gz, 'p', ...
        'MarkerSize',14, ...
        'MarkerFaceColor',col, ...
        'Color',col, ...
        'Tag',tag);
end