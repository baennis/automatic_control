function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL [학생 작성] 횡방향 통합 제어기 (AFS + ESC)
%
%   설계 기법:
%       - AFS: LQR (bicycle model state-space, vx 적응) 로 yaw rate 추종 보조 조향
%              + integral term (정상상태 오차 제거) + anti-windup
%       - ESC: |β| > β_threshold 일 때 driver intent 반대 방향 yaw moment
%       - speed scheduling: vx 별로 LQR 게인을 on-the-fly 재계산 (LPV/gain scheduling)
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s]
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 β [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태 (.intError 등)
%       CTRL       - 게인 (.LAT.Kp/Ki/Kd/intMax, 선택적으로 .LAT.Q/.R/.Ki_lqr/.betaThreshold/.betaGain)
%       LIM        - 한계값 (.MAX_STEER_ANGLE, .MAX_SLIP_ANGLE)
%       dt         - sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad]
%       deltaAdd.yawMoment  - ESC 요청 yaw moment [Nm]
%       ctrlState           - 업데이트된 내부 상태

    %% ---- 내부 상태 초기화 (필드 없으면 안전 생성) ----
    if ~isfield(ctrlState, 'intError');  ctrlState.intError  = 0; end
    if ~isfield(ctrlState, 'prevError'); ctrlState.prevError = 0; end

    %% ---- 설계 파라미터 (CTRL.LAT 에서 읽되, 없으면 합리적 기본값) ----
    % LQR 가중치: state = [vy; yawRate] 에 대한 Q, 입력 δ 에 대한 R
    %   yawRate 추종이 목적이므로 yawRate 상태에 큰 가중을 둔다.
    if isfield(CTRL.LAT, 'Q'); Qw = CTRL.LAT.Q; else; Qw = [1, 0; 0, 500]; end
    if isfield(CTRL.LAT, 'R'); Rw = CTRL.LAT.R; else; Rw = 1.0; end
    % yaw rate 추종 오차에 대한 적분 게인 (정상상태 오차 제거)
    if isfield(CTRL.LAT, 'Ki_lqr'); Ki = CTRL.LAT.Ki_lqr; else; Ki = CTRL.LAT.Ki; end
    % β-limiter
    if isfield(CTRL.LAT, 'betaThreshold'); betaTh = CTRL.LAT.betaThreshold; else; betaTh = deg2rad(3); end
    if isfield(CTRL.LAT, 'betaGain');      Kbeta  = CTRL.LAT.betaGain;      else; Kbeta  = 6e4;          end

    % 안전: β 임계가 물리 한계를 넘지 않도록
    betaTh = min(betaTh, 0.8 * LIM.MAX_SLIP_ANGLE);

    %% ---- (1) AFS: LQR 보조 조향 (yaw rate 추종) ----
    % bicycle model state-space 를 현재 vx 에서 계산 — speed scheduling 핵심
    % VEH 파라미터는 CTRL 경유로 넘어오지 않으므로 ctrlState 에 캐시되어 있으면 사용,
    % 아니면 표준 C-segment 값으로 build (sim_params.m 기준값과 정합)
    VEH = local_get_veh(ctrlState);

    yawErr = yawRateRef - yawRate;

    % LQR 게인: 매 step vx 로 (A,B) 갱신 후 lqr 계산은 비용이 크므로,
    % vx 가 이전과 충분히 가까우면 캐시된 게인 재사용 (계산량 절감 + 동일 거동)
    Klqr = local_lqr_gain(vx, VEH, Qw, Rw, ctrlState);
    ctrlState.lastVx   = vx;
    ctrlState.lastKlqr = Klqr;

    % LQR 상태 피드백: u = -K x, 단 우리는 "추종" 이므로 오차 기반으로 사용.
    % 상태 [vy; r] 중 yawRate 오차를 줄이는 방향으로 보조 조향을 만든다.
    % vy 는 직접 측정 어려우므로 β·vx 로 근사 (vy ≈ vx·tan(β) ≈ vx·β).
    vy_est = vx * slipAngle;
    xErr   = [0 - vy_est; yawErr];      % vy 는 0 으로 수렴 희망, yawRate 는 ref 추종
    deltaLQR = Klqr * xErr;             % 부호: 오차 양수(부족) → 조향 증가

    % 적분 보정 (정상상태 yaw rate 오차 제거)
    ctrlState.intError = ctrlState.intError + yawErr * dt;
    % anti-windup: 적분 클램프
    intLim = CTRL.LAT.intMax;
    ctrlState.intError = max(min(ctrlState.intError, intLim), -intLim);
    deltaInt = Ki * ctrlState.intError;

    deltaAFS = deltaLQR + deltaInt;

    %% ---- AFS 권한 제한 (경로추종 간섭 방지) ----
    % AFS 의 역할은 운전자가 못 잡는 미세 yaw 보정이지, 운전자의 경로추종을
    % 덮어쓰는 큰 조향이 아니다. 보조 조향을 작은 한계로 묶어 closed-loop
    % 운전자(Stanley) 의 경로추종과 충돌하는 과조향을 방지한다.
    % (ESC yaw moment 는 제한하지 않음 — A7 스핀아웃 방어의 핵심)
    if isfield(CTRL.LAT, 'afsAuthority'); afsLim = CTRL.LAT.afsAuthority; else; afsLim = deg2rad(2); end
    deltaAFS = max(min(deltaAFS, afsLim), -afsLim);

    %% ---- (2) ESC: slip angle limiter (yaw moment) ----
    % |β| 가 임계를 넘으면 β 를 줄이는 방향(=driver intent 반대)으로 모멘트 인가
    % 크기는 초과량에 비례, 속도 의존 스케일 f(vx) 적용
    yawMoment = 0;
    if abs(slipAngle) > betaTh
        fv = min(vx / 20, 2);                       % 속도 스케줄 (저속 약, 고속 강)
        excess = abs(slipAngle) - betaTh;
        yawMoment = -Kbeta * sign(slipAngle) * excess * fv;
    end

    %% ---- (3) saturation ----
    deltaAFS = max(min(deltaAFS, LIM.MAX_STEER_ANGLE), -LIM.MAX_STEER_ANGLE);

    %% ---- 출력 ----
    deltaAdd.steerAngle = deltaAFS;
    deltaAdd.yawMoment  = yawMoment;

    ctrlState.prevError = yawErr;
end

% =====================================================================
function VEH = local_get_veh(ctrlState)
% bicycle model 용 차량 파라미터. runner 가 ctrlState.VEH 로 주입하면 그걸 쓰고,
% 아니면 sim_params.m 의 C-segment 기준값으로 build (정합 유지).
    if isfield(ctrlState, 'VEH') && isstruct(ctrlState.VEH) ...
            && isfield(ctrlState.VEH, 'Cf')
        VEH = ctrlState.VEH;
        return;
    end
    VEH.mass = 1500; VEH.Iz = 2500;
    VEH.lf = 1.2;    VEH.lr = 1.4;
    VEH.Cf = 80000;  VEH.Cr = 85000;
end

% =====================================================================
function K = local_lqr_gain(vx, VEH, Q, R, ctrlState)
% vx 에서 bicycle model (A,B) 를 만들고 LQR 게인 계산.
% vx 변화가 작으면 캐시 재사용 (gain scheduling 의 보간 효과 + 연산 절감).
    if isfield(ctrlState, 'lastVx') && isfield(ctrlState, 'lastKlqr') ...
            && abs(vx - ctrlState.lastVx) < 0.5
        K = ctrlState.lastKlqr;
        return;
    end
    [A, B] = calc_bicycle_model(vx, VEH);
    K = local_lqr(A, B, Q, R);
end

% =====================================================================
function K = local_lqr(A, B, Q, R)
% 연속시간 LQR. Control System Toolbox 의 lqr 이 있으면 사용,
% 없으면 Hamiltonian 고유분해로 CARE 를 직접 푼다 (toolbox 무의존).
    if exist('lqr', 'file') == 2 || exist('lqr', 'builtin') == 5
        try
            K = lqr(A, B, Q, R);
            return;
        catch
            % fall through to manual solver
        end
    end
    P = local_care(A, B, Q, R);
    K = R \ (B' * P);
end

% =====================================================================
function P = local_care(A, B, Q, R)
% Continuous Algebraic Riccati Equation 을 Hamiltonian 행렬의
% stable invariant subspace 로 푼다. (Arnold-Laub 방식)
    n = size(A, 1);
    H = [A, -B*(R\B'); -Q, -A'];
    [V, D] = eig(H);
    d = diag(D);
    % 안정(실수부<0) 고유값에 해당하는 고유벡터 선택
    idx = real(d) < 0;
    if sum(idx) ~= n
        % 수치적으로 0 근방이 섞이면 가장 작은 n 개 실수부로 보정
        [~, order] = sort(real(d), 'ascend');
        idx = false(2*n, 1); idx(order(1:n)) = true;
    end
    U = V(:, idx);
    U1 = U(1:n, :);
    U2 = U(n+1:end, :);
    P = real(U2 / U1);
    P = (P + P') / 2;   % 대칭화
end
