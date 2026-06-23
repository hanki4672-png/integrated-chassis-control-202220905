function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)

deltaAdd.steerAngle = 0;
deltaAdd.yawMoment  = 0;

if dt <= 1e-5 || isnan(dt) || isinf(dt)
    dt = 1e-3;
end

if nargin < 5 || isempty(ctrlState)
    ctrlState = struct();
end
if ~isfield(ctrlState,'intError')
    ctrlState.intError = 0;
end
if ~isfield(ctrlState,'prevError')
    ctrlState.prevError = 0;
end
if ~isfield(ctrlState,'prevYawRef')
    ctrlState.prevYawRef = yawRateRef;
end

%% yawRateRef 변화율
yawRef_dot = 0;
if dt > 1e-5
    yawRef_dot = (yawRateRef - ctrlState.prevYawRef) / dt;
end
ctrlState.prevYawRef = yawRateRef;

yawError = yawRateRef - yawRate;

%% 상태 기반 gain scheduling
isAggressiveTurn = (vx > 12) && (abs(yawRateRef) > 0.03);

if isAggressiveTurn
    beta_th = 0.040;
    K_beta = 260000;
    Mz_max = 40000;

    Kp_scale = 1.6;   % 기존 3.2보다 낮춤
    Ki_scale = 0.10;
    Kd_scale = 0.18;
else
    beta_th = 0.140;
    K_beta  = 25000;
    Mz_max  = 5000;

    Kp_scale = 1.1;
    Ki_scale = 0.15;
    Kd_scale = 0.03;
end

%% 속도 스케줄링
if vx < 5
    f_vx = 0.25;
elseif vx < 20
    f_vx = 0.85;
else
    f_vx = max(0.50, 18 / max(vx,1));
end

%% anti-windup
if abs(yawError) < 0.04
    intScale = 0.2;
else
    intScale = 1.0;
end

ctrlState.intError = ctrlState.intError + yawError * dt * intScale;
ctrlState.intError = max(min(ctrlState.intError, CTRL.LAT.intMax), -CTRL.LAT.intMax);

yawDot = (yawError - ctrlState.prevError) / dt;
ctrlState.prevError = yawError;

%% AFS
Kp_eff = CTRL.LAT.Kp * Kp_scale * 0.010;
Ki_eff = CTRL.LAT.Ki * Ki_scale * 0.010;
Kd_eff = CTRL.LAT.Kd * Kd_scale * 0.010;

u_afs = Kp_eff * yawError ...
      + Ki_eff * ctrlState.intError ...
      + Kd_eff * yawDot;

deltaAdd.steerAngle = u_afs * f_vx;
deltaAdd.steerAngle = max(min(deltaAdd.steerAngle, LIM.MAX_STEER_ANGLE), -LIM.MAX_STEER_ANGLE);

%% ESC beta limiter
if isAggressiveTurn
    deltaAdd.yawMoment = -40000 * sign(slipAngle);
else
    if abs(slipAngle) > beta_th
        betaErr = abs(slipAngle) - beta_th;
        deltaAdd.yawMoment = -K_beta * sign(slipAngle) * betaErr * f_vx;
    else
        deltaAdd.yawMoment = 0;
    end
end

deltaAdd.yawMoment = max(min(deltaAdd.yawMoment, 40000), -40000);
end
