function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)

if nargin < 2 || isempty(ctrlState)
    ctrlState = struct();
end

cMin = CTRL.VER.cMin;
cMax = CTRL.VER.cMax;

zs_dot = zeros(4,1);
zu_dot = zeros(4,1);

if isfield(suspState,'zs_dot')
    zs_dot = suspState.zs_dot(:);
end
if isfield(suspState,'zu_dot')
    zu_dot = suspState.zu_dot(:);
end

zs_dot = [zs_dot; zeros(max(0,4-length(zs_dot)),1)];
zu_dot = [zu_dot; zeros(max(0,4-length(zu_dot)),1)];

zs_dot = zs_dot(1:4);
zu_dot = zu_dot(1:4);

v_rel = zs_dot - zu_dot;

dampingCmd = cMin * ones(4,1);

for i = 1:4
    if zs_dot(i) * v_rel(i) > 0
        dampingCmd(i) = cMax;
    else
        dampingCmd(i) = cMin;
    end
end

dampingCmd = max(min(dampingCmd, cMax), cMin);

end