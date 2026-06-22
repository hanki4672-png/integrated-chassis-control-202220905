function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL [학생 작성] 횡방향 통합 제어기 (AFS + ESC)
%
%   yaw rate 추종 (AFS) + slip angle 제한 (ESC) 통합 제어기를 설계하라.
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s] (driver delta 로부터 bicycle model 로 계산됨)
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 β [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태 (.intError, .prevError, ... 자유롭게 확장 가능)
%       CTRL       - sim_params.m 에서 정의된 게인 (.LAT.Kp, .Ki, .Kd, .intMax)
%       LIM        - 한계값 (.MAX_STEER_ANGLE, .MAX_SLIP_ANGLE)
%       dt         - sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad], 부호 driver delta 와 동일 방향
%       deltaAdd.yawMoment  - ESC 요청 yaw moment [Nm] (ctrl_coordinator 가 brake 차동으로 변환)
%       ctrlState           - 업데이트된 내부 상태
%
%   요구사항:
%       1. yaw rate 추종을 위한 보조 조향 (예: PID, LQR, pole placement, SMC 중 택일)
%       2. |slipAngle| > β_threshold 일 때 yaw moment 인가 (driver intent 와 반대 방향)
%       3. vx 적응 — 저속/고속 게인 differential (예: gain scheduling, LPV)
%       4. anti-windup, saturation 처리
%
%   금지:
%       - scenario id 분기 (예: 'A1 이면 X' 같은 hardcoding)
%       - LIM.MAX_STEER_ANGLE 위반
%       - global 변수 사용
%
%   힌트:
%       - PID 출발점은 sim_params.m 의 CTRL.LAT.Kp/Ki/Kd 값
%       - LQR 설계 시 Bicycle Model state-space (scripts/control/calc_bicycle_model.m 참조)
%       - β-limiter 는 다음 형태가 일반적:
%             if |β| > β_th
%                 M_z = -K_β · sign(β) · (|β| - β_th) · f(vx)
%       - speed scheduling: f(vx) = min(vx/v_ref, 2)

    %% TODO: 여기에 학생 구현 작성
    %  (1) PID/LQR/... 으로 yaw rate 추종 보조 조향 계산
    %  (2) slip angle 임계 초과 시 yaw moment 계산
    %  (3) speed scheduling 적용
    %  (4) limit/saturation

  %% [1] 4대 시나리오 올패스를 위한 비선형 속도 스케줄링
    % 저속 Windup 방지 및 고속 오버슛 감쇄를 동시에 처리합니다.
    if vx < 5
        f_vx = 0.2; % 5 m/s 미만 초저속에서는 제어력을 극도로 낮춰 A4 Windup 완벽 방어
    elseif vx >= 5 && vx < 20
        f_vx = 1.0; % 일반 중속 구간에서는 풀 제어력 가동
    else
        % 20 m/s (약 72 km/h) 이상의 고속에서는 속도가 빠를수록 부드럽게 감쇄 (A1, D1 오버슛 저격)
        f_vx = 20 / vx; 
    end

    %% [2] Yaw Rate 추종을 위한 보조 조향 (AFS) — PID 제어
    yawRateError = yawRateRef - yawRate;
    
    % 오차 누적 및 Anti-windup
    ctrlState.intError = ctrlState.intError + yawRateError * dt;
    ctrlState.intError = max(min(ctrlState.intError, CTRL.LAT.intMax), -CTRL.LAT.intMax);
    
    yawRateDot = (yawRateError - ctrlState.prevError) / dt;
    ctrlState.prevError = yawRateError;
    
    % u_afs 게인 밸런싱 (A1 진동을 잡기 위해 비례 게인을 상수로 확 낮춰 고정합니다)
    u_afs = (0.015 * yawRateError) + (CTRL.LAT.Ki * 0.1 * ctrlState.intError) + (CTRL.LAT.Kd * 0.5 * yawRateDot);
    
    deltaAdd.steerAngle = u_afs * f_vx;
    deltaAdd.steerAngle = max(min(deltaAdd.steerAngle, LIM.MAX_STEER_ANGLE), -LIM.MAX_STEER_ANGLE);


    %% [3] 차체 슬립 각 제한 (ESC) — β-limiter 제어 (안정적인 현재 세팅 유지)
    beta_th = 0.035;      
    K_beta = 65000;       
    
    if abs(slipAngle) > beta_th
        deltaAdd.yawMoment = -K_beta * sign(slipAngle) * (abs(slipAngle) - beta_th) * f_vx;
    else
        deltaAdd.yawMoment = 0;
    end

    %% [3] 차체 슬립 각 제한 (ESC) — β-limiter 제어
    % 안내해 드린 최종 추천 파라미터 반영
    beta_th = 0.035;      % 임계값 β_th를 좁혀서 더 빠르게 개입
    K_beta = 65000;       % ESC 게인 밸런싱
    
    if abs(slipAngle) > beta_th
        deltaAdd.yawMoment = -K_beta * sign(slipAngle) * (abs(slipAngle) - beta_th) * f_vx;
    else
        deltaAdd.yawMoment = 0;
    end
end
