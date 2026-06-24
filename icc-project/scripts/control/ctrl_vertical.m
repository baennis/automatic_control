function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL [학생 작성] CDC (Continuous Damping Control)
%
%   설계 기법:
%       - Hybrid skyhook + groundhook (continuous form)
%         · skyhook 성분: sprung mass 절대속도를 줄임 → ride comfort
%         · groundhook 성분: unsprung mass 절대속도를 줄임 → road holding
%       - per-wheel 독립 적용
%       - cMin ≤ c ≤ cMax 제한
%
%   Inputs:
%       suspState - .zs_dot(4), .zu_dot(4), .zs(4), .zu(4)
%       ctrlState - 내부 상태
%       CTRL      - .VER.cMin/cMax/skyGain (선택적 .VER.groundGain/.alpha)
%       dt        - sample time
%
%   Output:
%       dampingCmd - 4×1 damping coefficient [Ns/m]

    cMin = CTRL.VER.cMin;
    cMax = CTRL.VER.cMax;
    cSky = CTRL.VER.skyGain;
    if isfield(CTRL.VER, 'groundGain'); cGround = CTRL.VER.groundGain; else; cGround = 0.3 * cSky; end
    if isfield(CTRL.VER, 'alpha');      alpha   = CTRL.VER.alpha;      else; alpha   = 0.8;          end  % skyhook 비중

    cNom = 0.5 * (cMin + cMax);                 % 중립 감쇠

    % suspState 필드 방어적 확보 (없으면 passive 로 안전 복귀)
    [zs_dot, zu_dot, ok] = local_get_velocities(suspState);
    if ~ok
        dampingCmd = cNom * ones(4, 1);
        return;
    end

    dampingCmd = zeros(4, 1);
    for i = 1:4
        vRel = zs_dot(i) - zu_dot(i);           % 서스펜션 상대속도

        % --- skyhook (continuous semi-active) ---
        % sprung 속도와 상대속도가 같은 방향이면 강한 감쇠로 sprung 억제, 아니면 약하게
        if zs_dot(i) * vRel > 0
            cSkyhook = cSky;
        else
            cSkyhook = cMin;
        end

        % --- groundhook ---
        if (-zu_dot(i)) * (-vRel) > 0
            cGroundhook = cGround;
        else
            cGroundhook = cMin;
        end

        % --- hybrid blend ---
        c = alpha * cSkyhook + (1 - alpha) * cGroundhook;

        % --- saturation ---
        c = max(min(c, cMax), cMin);
        dampingCmd(i) = c;
    end
end

% =====================================================================
function [zs_dot, zu_dot, ok] = local_get_velocities(suspState)
% suspState 에서 sprung/unsprung 속도 4-vector 를 안전하게 추출.
    ok = false;
    zs_dot = zeros(4,1); zu_dot = zeros(4,1);
    if ~isstruct(suspState); return; end
    if isfield(suspState, 'zs_dot') && isfield(suspState, 'zu_dot')
        zs = suspState.zs_dot(:);
        zu = suspState.zu_dot(:);
        if numel(zs) >= 4 && numel(zu) >= 4
            zs_dot = zs(1:4);
            zu_dot = zu(1:4);
            ok = true;
        end
    end
end
