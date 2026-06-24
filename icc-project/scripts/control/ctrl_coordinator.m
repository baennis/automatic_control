function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation
%
%   설계:
%       (1) lonCmd.Fx_total < 0 → 4륜 제동, 전후 60:40 분배
%       (2) latCmd.yawMoment → 좌/우 차동 brake (track 반거리/lever arm)
%       (3) latCmd.steerAngle → actuatorCmd.steerAngle (saturation)
%       (4) verCmd → dampingCoeff (pass-through)
%       (5) 마찰원 제한(가산점): per-wheel brake torque ≤ μ·Fz 상당 토크
%       (6) 최종 [0, MAX_BRAKE_TRQ] 클리핑
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad]
%       actuatorCmd.brakeTorque   - 4×1 [FL; FR; RL; RR] [Nm]
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]

    %% ---- 파라미터 ----
    rw = local_field(VEH, 'rw', 0.31);
    tf = local_field(VEH, 'track_f', 1.55);
    tr = local_field(VEH, 'track_r', 1.55);
    mass = local_field(VEH, 'mass', 1500);
    if isfield(CTRL, 'COORD') && isfield(CTRL.COORD, 'frontBrakeRatio')
        ratioF = CTRL.COORD.frontBrakeRatio;
    else
        ratioF = 0.60;                          % 전후 60:40
    end
    g = 9.81;
    mu = local_field(VEH, 'mu_peak', 1.0);

    %% ---- (1) 종방향 제동 → 4륜 균등(전후 비율) ----
    brakeTorque = zeros(4, 1);                   % [FL; FR; RL; RR]
    if lonCmd.Fx_total < 0
        F_brake = abs(lonCmd.Fx_total);          % 총 제동력 [N]
        T_total = F_brake * rw;                  % 총 제동 토크 [Nm]
        T_front = ratioF * T_total;              % 전축 몫
        T_rear  = (1 - ratioF) * T_total;        % 후축 몫
        brakeTorque(1) = T_front / 2;            % FL
        brakeTorque(2) = T_front / 2;            % FR
        brakeTorque(3) = T_rear  / 2;            % RL
        brakeTorque(4) = T_rear  / 2;            % RR
    end

    %% ---- (2) ESC yaw moment → 4-wheel brake (WLS allocation) ----
    % 가중 최소자승 control allocation:
    %   목표: brake 차동으로 요청 yaw moment Mz 를 생성
    %   제어 변수 u = [dFL; dFR; dRL; dRR] (각 휠 추가 제동력 [N])
    %   yaw moment 매핑:  Mz = B_alloc * u
    %     B_alloc = [-t_f/2, +t_f/2, -t_r/2, +t_r/2]   (좌측 음, 우측 양)
    %   비용:  min  ||W_u * u||^2   s.t.  B_alloc*u = Mz   (effort 최소화)
    %   해 (가중 pseudo-inverse, equality-constrained LS):
    %     u* = Winv*B' * (B*Winv*B')^-1 * Mz,   Winv = W_u^-1
    %   → actuator effort 를 최소화하면서 목표 moment 정확 달성 (가산점: WLS allocation)
    Mz = latCmd.yawMoment;
    if abs(Mz) > 1e-6
        htf = max(tf, 1e-3) / 2;
        htr = max(tr, 1e-3) / 2;
        B_alloc = [-htf, htf, -htr, htr];        % 1×4 effectiveness (force→yaw moment)

        % 휠별 effort 가중 (대각). 전축이 조향축이라 차동에 더 쓰이도록 전축 가중 ↓
        % (가중이 작을수록 그 액추에이터를 더 많이 사용)
        if isfield(CTRL, 'COORD') && isfield(CTRL.COORD, 'wlsFrontWeight')
            wF = CTRL.COORD.wlsFrontWeight;
        else
            wF = 1.0;                            % 전축 effort 가중
        end
        if isfield(CTRL, 'COORD') && isfield(CTRL.COORD, 'wlsRearWeight')
            wR = CTRL.COORD.wlsRearWeight;
        else
            wR = 1.5;                            % 후축 effort 가중 (>전축 → 후축 덜 씀)
        end
        Winv = diag([1/wF, 1/wF, 1/wR, 1/wR]);   % W^-1

        % equality-constrained WLS 해: u = Winv*B' * inv(B*Winv*B') * Mz
        BWi   = B_alloc * Winv;                  % 1×4
        denom = BWi * B_alloc';                  % 스칼라 (1×1)
        if abs(denom) < 1e-9; denom = 1e-9; end
        dF    = (Winv * B_alloc') * (Mz / denom);% 4×1 추가 제동력 [N]

        % ESC 강도 게인: WLS 는 effort-최소 해라 보수적이므로, 실제 ESC 개입
        % 권한을 확보하기 위해 스케일. (CTRL 로 조정 가능)
        if isfield(CTRL, 'COORD') && isfield(CTRL.COORD, 'escGain')
            escGain = CTRL.COORD.escGain;
        else
            escGain = 3.0;
        end
        dF = dF * escGain;

        % 제동력 → 토크 (T = F · rw), 4륜에 가산
        brakeTorque = brakeTorque + dF * rw;
    end

    % 음수 토크는 물리적으로 불가 → 0 하한
    brakeTorque = max(brakeTorque, 0);

    %% ---- (5) 마찰원 제한 (가산점) ----
    % 각 휠 수직하중 근사 (정적 분배): 전축이 lr/L, 후축이 lf/L
    lf = local_field(VEH, 'lf', 1.2);
    lr = local_field(VEH, 'lr', 1.4);
    L  = lf + lr;
    Fz_f = mass * g * (lr / L) / 2;              % 전륜 1개
    Fz_r = mass * g * (lf / L) / 2;              % 후륜 1개
    Tmax_f = mu * Fz_f * rw;                     % 마찰원 상당 최대 제동토크 (전륜)
    Tmax_r = mu * Fz_r * rw;                     % (후륜)
    brakeTorque(1) = min(brakeTorque(1), Tmax_f);
    brakeTorque(2) = min(brakeTorque(2), Tmax_f);
    brakeTorque(3) = min(brakeTorque(3), Tmax_r);
    brakeTorque(4) = min(brakeTorque(4), Tmax_r);

    %% ---- (5b) per-wheel ABS: 락업 휠 제동 감쇠 ----
    % ctrl_longitudinal 이 forceCmd.wheelSlip 으로 per-wheel 슬립을 넘겨준다.
    % 슬립비 |κ| 가 타겟을 넘으면(락업 진행) 그 휠의 brake torque 를 연속 감쇠.
    % 시나리오가 강제하는 제동(특히 후륜 락업)을 휠 단위로 풀어 ABS 동작 구현.
    if isfield(lonCmd, 'wheelSlip') && numel(lonCmd.wheelSlip) >= 4
        slip = lonCmd.wheelSlip(:);
        if isfield(lonCmd, 'kappaTarget'); kTgt = abs(lonCmd.kappaTarget); else; kTgt = 0.12; end
        for i = 1:4
            kappa = abs(slip(i));
            if kappa > kTgt
                % 초과량에 비례해 감쇠. 락업(κ→1)일수록 강하게 줄임.
                % reduction = kTgt/kappa 면 슬립을 타겟 근처로 되돌리는 효과.
                reduction = kTgt / max(kappa, kTgt);
                reduction = max(min(reduction, 1.0), 0.1);
                brakeTorque(i) = brakeTorque(i) * reduction;
            end
        end
    end

    %% ---- (6) 최종 brake 클리핑 ----
    brakeTorque = max(min(brakeTorque, LIM.MAX_BRAKE_TRQ), 0);

    %% ---- (3) 조향 saturation ----
    steer = max(min(latCmd.steerAngle, LIM.MAX_STEER_ANGLE), -LIM.MAX_STEER_ANGLE);

    %% ---- (4) damping pass-through (4×1 보장) ----
    d = verCmd(:);
    if numel(d) < 4; d = repmat(d(1), 4, 1); end
    dampingCoeff = d(1:4);

    %% ---- 출력 ----
    actuatorCmd.steerAngle   = steer;
    actuatorCmd.brakeTorque  = brakeTorque;
    actuatorCmd.dampingCoeff = dampingCoeff;
end

% =====================================================================
function v = local_field(S, name, default)
    if isstruct(S) && isfield(S, name) && ~isempty(S.(name))
        v = S.(name);
    else
        v = default;
    end
end
