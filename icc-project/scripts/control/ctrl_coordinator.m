function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)

%% Steering saturation
if isfield(latCmd,'steerAngle')
    steer = latCmd.steerAngle;
else
    steer = 0;
end
actuatorCmd.steerAngle = max(min(steer, LIM.MAX_STEER_ANGLE), -LIM.MAX_STEER_ANGLE);

%% Parameters
if isfield(VEH,'r_w')
    rw = VEH.r_w;
elseif isfield(VEH,'wheel_radius')
    rw = VEH.wheel_radius;
else
    rw = 0.33;
end

if isfield(VEH,'track_f')
    halfTrackF = VEH.track_f / 2;
else
    halfTrackF = 0.78;
end

if isfield(VEH,'track_r')
    halfTrackR = VEH.track_r / 2;
else
    halfTrackR = 0.78;
end

Tmax = LIM.MAX_BRAKE_TRQ;
brakeTorque = zeros(4,1);  % [FL; FR; RL; RR]

%% Longitudinal brake allocation: 60:40 split
Fx = 0;
if isfield(lonCmd,'Fx_total')
    Fx = lonCmd.Fx_total;
end

if Fx < 0
    T_total = abs(Fx) * rw;

    brakeTorque(1) = 0.60 * T_total / 2;
    brakeTorque(2) = 0.60 * T_total / 2;
    brakeTorque(3) = 0.40 * T_total / 2;
    brakeTorque(4) = 0.40 * T_total / 2;
end

%% ESC / DYC brake allocation
Mz = 0;
if isfield(latCmd,'yawMoment')
    Mz = latCmd.yawMoment;
end

ratioF = 0.35;
dycScale = 5.0;

dTf = dycScale * abs(Mz) * ratioF     * rw / max(halfTrackF,0.1);
dTr = dycScale * abs(Mz) * (1-ratioF) * rw / max(halfTrackR,0.1);


% Positive Mz: left brake increase
% Negative Mz: right brake increase
if Mz > 0
    brakeTorque(2) = brakeTorque(2) + dTf; % FR
    brakeTorque(4) = brakeTorque(4) + dTr; % RR
elseif Mz < 0
    brakeTorque(1) = brakeTorque(1) + dTf; % FL
    brakeTorque(3) = brakeTorque(3) + dTr; % RL
end

%% Prevent excessive brake in high-speed turn
% A7/D1에서 LTR 악화 방지용
if vx > 18
    brakeTorque = brakeTorque * 0.92;
end

%% Saturation
brakeTorque = max(min(brakeTorque, Tmax), 0);

actuatorCmd.brakeTorque  = brakeTorque;
actuatorCmd.dampingCoeff = verCmd;

end