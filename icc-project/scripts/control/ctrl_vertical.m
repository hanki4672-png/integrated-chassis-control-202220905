function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL [학생 작성] CDC (Continuous Damping Control) — per-wheel 감쇠 명령
%   Skyhook 반능동(Semi-active) 제어 알고리즘을 활용한 4륜 독립 서스펜션 제어

    % 4바퀴 배열 초기화
    c_final = zeros(4,1);
    
    % 구조체 누락 방지 예외 처리 (안전 장치)
    if isfield(CTRL, 'VER')
        cMin    = CTRL.VER.cMin;
        cMax    = CTRL.VER.cMax;
        skyGain = CTRL.VER.skyGain;
    else
        cMin    = 500;
        cMax    = 5000;
        skyGain = 2500;
    end

    %% [핵심] Semi-active On-Off Skyhook 알고리즘 4륜 독립 구현
    for i = 1:4
        % 1. 바퀴별 물리 변수 추출
        zs_dot = suspState.zs_dot(i); % Sprung mass (차체) 수직 속도
        zu_dot = suspState.zu_dot(i); % Unsprung mass (바퀴) 수직 속도
        
        v_rel = zs_dot - zu_dot; % 차체와 바퀴 사이의 상대 속도
        
        % 2. Skyhook On-Off 제어 판별식 (힌트 반영)
        % 차체의 절대 속도 방향과 서스펜션의 상대 운동 방향이 일치할 때
        % 제동력을 극대화(cMax)하여 차체의 거동(Bounce/Roll/Pitch)을 강하게 억제합니다.
        if (zs_dot * v_rel) > 0
            c_final(i) = skyGain; % 댐핑 압력 상승
        else
            c_final(i) = cMin;    % 최소 감쇠력으로 편안한 승차감 확보
        end
        
        % 3. 물리적 하드웨어 벨브 한계 제한 (Saturation 규격 만족)
        c_final(i) = max(min(c_final(i), cMax), cMin);
    end

    %% (2) 출력 프로토콜 매핑 (구조체 필드명 일치)
    dampingCmd.dampingCoeff = c_final;
end