function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL [학생 작성] 종방향 제어기 (속도 추종 + ABS)
%
%   설계 기법:
%       - 속도 추종: PI (cruise/decel)
%       - ABS: slip-limit. 감속 중(ax<0) 휠 슬립비 |κ|>κ_target 이면 brake force 감쇠.
%              bang-bang 보다 연속 감쇠가 absSlipRMS 에 유리.
%       - jerk 제한: force 변화율을 LIM.MAX_JERK·m 으로 cap
%       - anti-windup: 적분 클램프
%
%   Inputs:
%       vxRef     - 목표 종방향 속도 [m/s]
%       vx        - 실제 종방향 속도 [m/s]
%       ax        - 종가속도 [m/s^2]
%       ctrlState - 내부 상태 (.intError, .prevForce, .wheelSlip(4) 등)
%       CTRL      - .LON.Kp/Ki/intMax (선택적 .LON.kappaTarget/.absGain)
%       LIM       - .MAX_AX, .MAX_JERK, .MAX_BRAKE_TRQ
%       dt        - sample time
%
%   Outputs:
%       forceCmd.Fx_total   - 총 종방향 힘 요구 [N], 양수 가속 / 음수 제동
%       forceCmd.brakeRatio - 제동 비율 (0~1)
%       ctrlState           - 업데이트

    %% ---- 내부 상태 초기화 ----
    if ~isfield(ctrlState, 'intError');  ctrlState.intError  = 0; end
    if ~isfield(ctrlState, 'prevForce'); ctrlState.prevForce = 0; end

    %% ---- 파라미터 ----
    Kp = CTRL.LON.Kp;
    Ki = CTRL.LON.Ki;
    intLim = CTRL.LON.intMax;
    if isfield(CTRL.LON, 'kappaTarget'); kappaTarget = CTRL.LON.kappaTarget; else; kappaTarget = 0.12; end
    if isfield(CTRL.LON, 'absGain');     absGain     = CTRL.LON.absGain;     else; absGain     = 1.0;  end

    m = local_get_mass(ctrlState);

    %% ---- (1) 속도 추종 PI ----
    vErr = vxRef - vx;
    ctrlState.intError = ctrlState.intError + vErr * dt;
    ctrlState.intError = max(min(ctrlState.intError, intLim), -intLim);  % anti-windup
    % PI 출력은 가속도 요구로 본 뒤 force 로 환산 (F = m·a)
    accCmd = Kp * vErr + Ki * ctrlState.intError;
    accCmd = max(min(accCmd, LIM.MAX_AX), -LIM.MAX_AX);
    Fx = m * accCmd;

    %% ---- (2) ABS: slip-limit ----
    % runner 가 매 step 휠 슬립비를 ctrlState 에 캐시한다고 가정 (헤더 명시).
    % 필드명이 환경마다 다를 수 있어 여러 후보를 방어적으로 탐색.
    kappaMax = local_get_max_slip(ctrlState);
    absActive = false;
    if Fx < 0 && ax < 0 && ~isnan(kappaMax) && kappaMax > kappaTarget
        % 초과량에 비례해 제동력 연속 감쇠 (0.3~1.0 배로 제한)
        excess = kappaMax - kappaTarget;
        reduction = 1 - absGain * (excess / max(kappaTarget, 1e-3));
        reduction = max(min(reduction, 1.0), 0.3);
        Fx = Fx * reduction;
        absActive = true;
    end
    ctrlState.absActive = absActive;

    %% ---- (3) jerk 제한 ----
    maxdF = LIM.MAX_JERK * m * dt;               % 허용 force 변화량 / step
    dF = Fx - ctrlState.prevForce;
    dF = max(min(dF, maxdF), -maxdF);
    Fx = ctrlState.prevForce + dF;
    ctrlState.prevForce = Fx;

    %% ---- 출력 ----
    forceCmd.Fx_total = Fx;
    if Fx < 0
        forceCmd.brakeRatio = 1;                 % 제동 모드
    else
        forceCmd.brakeRatio = 0;                 % 가속 모드
    end

    % per-wheel ABS 를 coordinator 가 수행할 수 있도록 휠 슬립과 타겟을 전달.
    % (총량 제동은 여기서, 휠별 락업 감쇠는 coordinator 에서 — 분배 단계라 적절)
    forceCmd.wheelSlip   = local_get_wheelslip_vec(ctrlState);  % 4×1 [FL;FR;RL;RR]
    forceCmd.kappaTarget = kappaTarget;
end

% =====================================================================
function sv = local_get_wheelslip_vec(ctrlState)
% per-wheel 슬립 4-vector [FL;FR;RL;RR] 를 안전하게 추출. 없으면 0벡터.
    sv = zeros(4,1);
    if isfield(ctrlState, 'wheelSlip') && numel(ctrlState.wheelSlip) >= 4
        w = ctrlState.wheelSlip(:);
        sv = w(1:4);
        return;
    end
    if isfield(ctrlState, 'slipRatio') && numel(ctrlState.slipRatio) >= 4
        w = ctrlState.slipRatio(:);
        sv = w(1:4);
        return;
    end
    if isfield(ctrlState, 'tire') && isstruct(ctrlState.tire)
        wheels = {'FL','FR','RL','RR'};
        for i = 1:4
            w = wheels{i};
            if isfield(ctrlState.tire, w) && isfield(ctrlState.tire.(w), 'slipRatio')
                sv(i) = ctrlState.tire.(w).slipRatio;
            end
        end
    end
end

% =====================================================================
function m = local_get_mass(ctrlState)
    if isfield(ctrlState, 'VEH') && isstruct(ctrlState.VEH) && isfield(ctrlState.VEH, 'mass')
        m = ctrlState.VEH.mass;
    else
        m = 1500;
    end
end

% =====================================================================
function kappaMax = local_get_max_slip(ctrlState)
% runner 가 주입했을 법한 휠 슬립 필드를 방어적으로 탐색.
% 후보: ctrlState.wheelSlip (4-vec), ctrlState.slipRatio,
%       ctrlState.tire.{FL,FR,RL,RR}.slipRatio
    kappaMax = NaN;
    if isfield(ctrlState, 'wheelSlip') && ~isempty(ctrlState.wheelSlip)
        kappaMax = max(abs(ctrlState.wheelSlip(:)));
        return;
    end
    if isfield(ctrlState, 'slipRatio') && ~isempty(ctrlState.slipRatio)
        kappaMax = max(abs(ctrlState.slipRatio(:)));
        return;
    end
    if isfield(ctrlState, 'tire') && isstruct(ctrlState.tire)
        wheels = {'FL','FR','RL','RR'};
        vals = [];
        for i = 1:numel(wheels)
            w = wheels{i};
            if isfield(ctrlState.tire, w) && isfield(ctrlState.tire.(w), 'slipRatio')
                vals(end+1) = abs(ctrlState.tire.(w).slipRatio); %#ok<AGROW>
            end
        end
        if ~isempty(vals); kappaMax = max(vals); end
    end
end
