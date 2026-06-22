function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL 종방향 제어기 기본 뼈대 복구
%   (ABS 핵심 로직은 ctrl_coordinator에서 원격 제어하므로 여기서는 패스스루만 수행합니다.)

% 구조체 기본 필드 생성 및 초기화
forceCmd.Fx_total   = 0;
forceCmd.brakeRatio = 0;

% 속도 추종 PI 루프
if ~isfield(ctrlState, 'intError')
    ctrlState.intError = 0;
end

speedError = vxRef - vx;
ctrlState.intError = ctrlState.intError + speedError * dt;
ctrlState.intError = max(min(ctrlState.intError, CTRL.LON.intMax), -CTRL.LON.intMax);

% 계산된 힘 출력
forceCmd.Fx_total = (CTRL.LON.Kp * speedError) + (CTRL.LON.Ki * ctrlState.intError);
if forceCmd.Fx_total < 0
    forceCmd.brakeRatio = min(abs(forceCmd.Fx_total) / LIM.MAX_BRAKE_TRQ, 1.0);
end
end