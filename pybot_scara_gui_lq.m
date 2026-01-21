function pybot_scara_gui_lq
% PyBot SCARA GUI (MATLAB R2025b safe) - FIXED TIMER
clc;

S = struct();
S.sp = [];
S.isOpen = false;
S.portName = "";
S.baud = 115200;
S.timer = [];
S.lastRx = "";
S.eStop = false;

P = struct();
P.L1 = 91.61;   % mm
P.L2 = 105.92;  % mm
P.Zmin = -120;  % mm
P.Zmax =  120;  % mm

LQ = struct();
LQ.enable = false;
LQ.alpha = 0.15;
LQ.maxStepDeg = 2.0;
LQ.maxStepZ   = 1.0;

S.refCmd = [0 0 0 90 90];
S.curCmd = [0 0 0 90 90];

% -------------------- UI --------------------
fig = uifigure('Name','PyBot SCARA GUI (R2025b) - IK/FK + LQR','Position',[120 80 1000 620]);
fig.CloseRequestFcn = @onClose;

serialPanel = uipanel(fig,'Title','Serial','Position',[10 560 980 50]);

uilabel(serialPanel,'Text','Port:','Position',[10 8 40 22]);
ddPort  = uidropdown(serialPanel,'Items',{'(refresh)'} ,'Value','(refresh)','Position',[55 8 120 22]);

btnRefresh = uibutton(serialPanel,'Text','Refresh','Position',[185 8 80 22], ...
    'ButtonPushedFcn',@(~,~)refreshPorts());

uilabel(serialPanel,'Text','Baud:','Position',[280 8 45 22]);
ddBaud  = uidropdown(serialPanel,'Items',{'115200','230400','250000','500000'}, ...
    'Value','115200','Position',[330 8 90 22]);

btnOpen  = uibutton(serialPanel,'Text','Open Port','Position',[435 8 90 22], ...
    'ButtonPushedFcn',@(~,~)openPort());
btnClose = uibutton(serialPanel,'Text','Close','Position',[535 8 70 22], ...
    'ButtonPushedFcn',@(~,~)closePort());

btnPing  = uibutton(serialPanel,'Text','Ping','Position',[615 8 60 22], ...
    'ButtonPushedFcn',@(~,~)sendLine("PING"));

lblStatus = uilabel(serialPanel,'Text','Status: CLOSED','Position',[690 8 280 22]);

cmdPanel = uipanel(fig,'Title','Command','Position',[10 10 480 540]);

uilabel(cmdPanel,'Text','Joint Command','FontWeight','bold','Position',[15 490 200 22]);

uilabel(cmdPanel,'Text','θ1 (deg):','Position',[15 470 70 22]);
edTh1 = uieditfield(cmdPanel,'numeric','Value',30,'Limits',[-180 180],'Position',[90 470 90 22]);

uilabel(cmdPanel,'Text','θ2 (deg):','Position',[200 470 70 22]);
edTh2 = uieditfield(cmdPanel,'numeric','Value',15,'Limits',[-180 180],'Position',[275 470 90 22]);

uilabel(cmdPanel,'Text','Z (mm):','Position',[15 435 70 22]);
edZ = uieditfield(cmdPanel,'numeric','Value',-20,'Limits',[P.Zmin P.Zmax],'Position',[90 435 90 22]);

btnGO = uibutton(cmdPanel,'Text','GO','FontSize',11,'FontWeight','bold', ...
    'Position',[275 430 100 35],'ButtonPushedFcn',@(~,~)onGO());

btnSTOP = uibutton(cmdPanel,'Text','STOP ALL (Emergency)','FontSize',14,'FontWeight','bold', ...
    'Position',[15 350 350 45],'BackgroundColor',[0.85 0.2 0.2],'FontColor',[1 1 1], ...
    'ButtonPushedFcn',@(~,~)onSTOP());

btnZERO = uibutton(cmdPanel,'Text','ZERO / TARE','FontSize',12,'FontWeight','bold', ...
    'Position',[370 350 95 45],'ButtonPushedFcn',@(~,~)onZERO());

uilabel(cmdPanel,'Text','LQR (simplified smoothing)','FontWeight','bold','Position',[15 300 250 22]);

cbLQR = uicheckbox(cmdPanel,'Text','Enable smoothing','Value',false,'Position',[15 270 150 22], ...
    'ValueChangedFcn',@(~,~)onLqrToggle());

uilabel(cmdPanel,'Text','alpha (0..1):','Position',[180 270 85 22]);
edAlpha = uieditfield(cmdPanel,'numeric','Value',LQ.alpha,'Limits',[0.01 1],'Position',[270 270 70 22]);

uilabel(cmdPanel,'Text','max step deg:','Position',[15 240 90 22]);
edMaxDeg = uieditfield(cmdPanel,'numeric','Value',LQ.maxStepDeg,'Limits',[0.1 20],'Position',[110 240 70 22]);

uilabel(cmdPanel,'Text','max step Z(mm):','Position',[200 240 95 22]);
edMaxZ = uieditfield(cmdPanel,'numeric','Value',LQ.maxStepZ,'Limits',[0.1 10],'Position',[300 240 70 22]);

uilabel(cmdPanel,'Text','IK / FK','FontWeight','bold','Position',[15 195 200 22]);

uilabel(cmdPanel,'Text','X (mm):','Position',[15 165 60 22]);
edX = uieditfield(cmdPanel,'numeric','Value',120,'Position',[75 165 80 22]);

uilabel(cmdPanel,'Text','Y (mm):','Position',[170 165 60 22]);
edY = uieditfield(cmdPanel,'numeric','Value',80,'Position',[230 165 80 22]);

btnSolveIK = uibutton(cmdPanel,'Text','Solve IK -> Joint fields','Position',[15 130 175 28], ...
    'ButtonPushedFcn',@(~,~)onSolveIK());

btnSolveFK = uibutton(cmdPanel,'Text','Solve FK -> X,Y','Position',[200 130 120 28], ...
    'ButtonPushedFcn',@(~,~)onSolveFK());

uilabel(cmdPanel,'Text','Elbow:','Position',[335 130 45 22]);
ddElbow = uidropdown(cmdPanel,'Items',{'0','1'},'Value','0','Position',[385 130 60 22]);

rightPanel = uipanel(fig,'Title','Telemetry & Log','Position',[500 10 490 540]);

uilabel(rightPanel,'Text','Angles (deg):','FontWeight','bold','Position',[15 490 100 22]);
lblAngles = uilabel(rightPanel,'Text','0.00, 0.00, 0.00','Position',[120 490 300 22]);

uilabel(rightPanel,'Text','Steps:','FontWeight','bold','Position',[15 460 60 22]);
lblSteps = uilabel(rightPanel,'Text','0, 0, 0','Position',[80 460 300 22]);

uilabel(rightPanel,'Text','E-Stop:','FontWeight','bold','Position',[15 430 60 22]);
lblEStop = uilabel(rightPanel,'Text','0','Position',[80 430 60 22]);

uilabel(rightPanel,'Text','Port:','FontWeight','bold','Position',[160 430 40 22]);
lblPortOpen = uilabel(rightPanel,'Text','(closed)','Position',[205 430 250 22]);

uilabel(rightPanel,'Text','Log','FontWeight','bold','Position',[15 395 50 22]);
logBox = uitextarea(rightPanel,'Editable','off','Position',[15 15 460 380]);
logBox.Value = strings(0,1);
uilabel(cmdPanel,'Text','Grip Rot (deg):','Position',[15 400 110 22]);
edRot = uieditfield(cmdPanel,'numeric','Value',90,'Limits',[0 180],'Position',[130 400 90 22]);

uilabel(cmdPanel,'Text','Grip (deg):','Position',[240 400 80 22]);
edGrip = uieditfield(cmdPanel,'numeric','Value',90,'Limits',[70 100],'Position',[320 400 90 22]);

refreshPorts();

S.timer = timer('ExecutionMode','fixedSpacing','Period',0.05,'TimerFcn',@(~,~)timerTick());
start(S.timer);

logMsg("GUI ready. Select COM then click Open Port.");

% -------------------- callbacks --------------------
    function refreshPorts()
        try
            ports = serialportlist("available");
            if isempty(ports)
                ddPort.Items = {'(no ports)'};
                ddPort.Value = '(no ports)';
            else
                ddPort.Items = cellstr(ports);
                ddPort.Value = ddPort.Items{1};
            end
            logMsg("Ports refreshed.");
        catch ME
            ddPort.Items = {'(error)'};
            ddPort.Value = '(error)';
            logMsg("ERROR: serialportlist failed: " + string(ME.message));
        end
    end

    function openPort()
        if S.isOpen
            logMsg("Already open.");
            return;
        end

        port = string(ddPort.Value);
        baud = str2double(string(ddBaud.Value));

        if port == "(no ports)" || port == "(error)" || port == "(refresh)"
            logMsg("ERROR: Select a valid COM port.");
            return;
        end

        try
            S.sp = serialport(port, baud, "Timeout", 0.05);
            configureTerminator(S.sp,"LF");
            flush(S.sp);
            S.isOpen = true;
            S.portName = port;
            S.baud = baud;
            lblStatus.Text = "Status: OPEN (" + port + " @ " + string(baud) + ")";
            lblPortOpen.Text = "open: " + port;
            logMsg("Opened " + port + " @ " + string(baud) + ".");
            sendLine("PING");
        catch ME
            S.isOpen = false;
            S.sp = [];
            lblStatus.Text = "Status: OPEN FAILED";
            lblPortOpen.Text = "(closed)";
            logMsg("ERROR: Open failed: " + string(ME.message));
        end
    end

    function closePort()
        if ~S.isOpen
            logMsg("Serial not open.");
            return;
        end
        try, delete(S.sp); catch, end
        S.sp = [];
        S.isOpen = false;
        lblStatus.Text = "Status: CLOSED";
        lblPortOpen.Text = "(closed)";
        logMsg("Port closed.");
    end

    function sendLine(lineStr)
        lineStr = string(lineStr);
        if ~S.isOpen || isempty(S.sp)
            logMsg("ERROR: Serial not open.");
            return;
        end
        try
            writeline(S.sp, lineStr);
            logMsg("TX: " + lineStr);
        catch ME
            logMsg("ERROR TX: " + string(ME.message));
        end
    end

    function pollSerial()
        if ~S.isOpen || isempty(S.sp), return; end
        try
            while S.sp.NumBytesAvailable > 0
                rx = readline(S.sp);
                rx = strtrim(string(rx));
                if rx == "", continue; end
                S.lastRx = rx;
                logMsg("RX: " + rx);
                parseTelemetry(rx);
            end
        catch ME
            logMsg("ERROR RX: " + string(ME.message));
        end
    end

    function parseTelemetry(rx)
        if startsWith(rx,"TLM", "IgnoreCase",true)
            parts = split(rx,",");
            if numel(parts) >= 8
                th1 = str2double(parts(2));
                th2 = str2double(parts(3));
                z   = str2double(parts(4));
                st1 = str2double(parts(5));
                st2 = str2double(parts(6));
                stz = str2double(parts(7));
                es  = str2double(parts(8));
                if ~isnan(th1) && ~isnan(th2) && ~isnan(z)
                    lblAngles.Text = sprintf("%.2f, %.2f, %.2f", th1, th2, z);
                end
                lblSteps.Text = sprintf("%.0f, %.0f, %.0f", st1, st2, stz);
                lblEStop.Text = sprintf("%.0f", es);
            end
        end
    end

    function onGO()
        th1 = edTh1.Value;
        th2 = edTh2.Value;
        z   = max(P.Zmin, min(P.Zmax, edZ.Value));
        edZ.Value = z;
        rot  = edRot.Value;
        grip = edGrip.Value;  
        LQ.enable = cbLQR.Value;
        LQ.alpha = edAlpha.Value;
        LQ.maxStepDeg = edMaxDeg.Value;
        LQ.maxStepZ = edMaxZ.Value;

        S.refCmd = [th1 th2 z rot grip]; 
        if ~LQ.enable
         sendCmd(th1, th2, z, rot, grip);
         S.curCmd = [th1 th2 z rot grip];
        else
          logMsg("LQR smoothing active: moving towards target.");

        end
    end

    function sendCmd(th1, th2, z, rot, grip)
    lineStr = sprintf("CMD,%.2f,%.2f,%.2f,%.2f,%.2f", th1, th2, z, rot, grip);
    sendLine(lineStr);
    end
    function stepShapedOnce()
    tgt = S.refCmd;   % [th1 th2 z rot grip]
    cur = S.curCmd;
    d = tgt - cur;

    % joints
    d(1) = max(-LQ.maxStepDeg, min(LQ.maxStepDeg, d(1)));
    d(2) = max(-LQ.maxStepDeg, min(LQ.maxStepDeg, d(2)));
    d(3) = max(-LQ.maxStepZ,   min(LQ.maxStepZ,   d(3)));

    % servos (خليهم يتحركوا بسلاسة برضو: نفس maxStepDeg كمثال)
    d(4) = max(-LQ.maxStepDeg, min(LQ.maxStepDeg, d(4)));
    d(5) = max(-LQ.maxStepDeg, min(LQ.maxStepDeg, d(5)));

    nxt = cur + LQ.alpha * d;

    if all(abs(tgt - nxt) < [0.05 0.05 0.05 0.2 0.2])
        nxt = tgt;
    end

    S.curCmd = nxt;
    sendCmd(nxt(1), nxt(2), nxt(3), nxt(4), nxt(5));
end


    function onSTOP()
        S.eStop = true;
        lblEStop.Text = "1";
        sendLine("STOP");
        logMsg("STOP sent (Emergency).");
    end

    function onZERO()
        S.eStop = false;
        lblEStop.Text = "0";
        sendLine("ZERO");
        logMsg("ZERO/TARE sent.");
    end

    function onLqrToggle()
        if cbLQR.Value
            logMsg("LQR smoothing enabled.");
        else
            logMsg("LQR smoothing disabled.");
        end
    end

    function onSolveIK()
        x = edX.Value; y = edY.Value;
        elbow = str2double(ddElbow.Value);
        [th1, th2, ok] = ik2R(x,y,P.L1,P.L2, elbow);
        if ~ok
            logMsg("ERR: IK failed (out of reach).");
            return;
        end
        edTh1.Value = th1;
        edTh2.Value = th2;
        logMsg(sprintf("IK solved -> th1=%.2f th2=%.2f", th1, th2));
    end

    function onSolveFK()
        th1 = edTh1.Value;
        th2 = edTh2.Value;
        [x,y] = fk2R(th1,th2,P.L1,P.L2);
        edX.Value = x;
        edY.Value = y;
        logMsg(sprintf("FK -> X=%.2f Y=%.2f", x, y));
    end

    function logMsg(msg)
        msg = string(msg);
        t = string(datetime('now','Format','HH:mm:ss'));
        line = "[" + t + "] " + msg;
        v = string(logBox.Value);
        v = [v; line];
        if numel(v) > 300, v = v(end-299:end); end
        logBox.Value = v;
        drawnow limitrate;
    end

    function timerTick()
        % poll RX first
        pollSerial();

        % shaping loop when enabled
        if LQ.enable && S.isOpen && ~isempty(S.sp)
            if any(abs(S.refCmd - S.curCmd) > [0.05 0.05 0.05])
                stepShapedOnce();
            end
        end
    end

    function onClose(~,~)
        try
            if ~isempty(S.timer) && isvalid(S.timer)
                stop(S.timer);
                delete(S.timer);
            end
        catch, end
        try
            if S.isOpen, closePort(); end
        catch, end
        delete(fig);
    end

end

% -------------------- Kinematics helpers --------------------
function [x,y] = fk2R(th1_deg, th2_deg, L1, L2)
th1 = deg2rad(th1_deg);
th2 = deg2rad(th2_deg);
x = L1*cos(th1) + L2*cos(th1+th2);
y = L1*sin(th1) + L2*sin(th1+th2);
end

function [th1_deg, th2_deg, ok] = ik2R(x,y,L1,L2, elbow)
ok = true;
r2 = x.^2 + y.^2;
c2 = (r2 - L1^2 - L2^2) / (2*L1*L2);
if c2 < -1 || c2 > 1
    ok = false; th1_deg = 0; th2_deg = 0; return;
end
s2 = sqrt(max(0,1 - c2^2));
if elbow == 1, s2 = -s2; end
th2 = atan2(s2, c2);
k1 = L1 + L2*cos(th2);
k2 = L2*sin(th2);
th1 = atan2(y,x) - atan2(k2,k1);
th1_deg = rad2deg(th1);
th2_deg = rad2deg(th2);
end