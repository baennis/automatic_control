function info = student_info()
%STUDENT_INFO 학생 정보 — 채점 시 매칭에 사용. **반드시 수정해서 제출.**
%
%   info struct fields:
%       .student_id   - 202021314
%       .name         - 배준원
%       .team_members - 
%       .course       - 2026 자동제어
%       .ai_usage     - claude(코드 방향성, 문법 수정)
%
%   본 파일을 수정하지 않으면 -5점 감점 + 채점 시트 매칭 불가.

    info.student_id   = '202021314';
    info.name         = '배준원';
    info.team_members = {};

    info.course = '자동제어 - 2026 봄';

    % AI 도구 사용 사실 (정직 신고) — 사용 안 했으면 'none'
    %   예: 'ChatGPT used for PID gain tuning suggestion'
    %       'Claude used to debug LQR design'
    info.ai_usage = 'claude(코드 방향성, 문법 수정)';

    %% 검증 (수정 금지)
    if contains(info.student_id, 'TODO_FILL')
        warning('[student_info] 학번이 기입되지 않았습니다 — 채점 시 감점 + 매칭 불가');
    end
    if contains(info.name, 'TODO_FILL')
        warning('[student_info] 이름이 기입되지 않았습니다');
    end
end
