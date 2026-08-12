# 문서 안내

| 문서 | 용도 | 상태 |
|---|---|---|
| [**code_review_script.md**](code_review_script.md) | **코드 리뷰 발표 대본** — 디렉토리 → 설계 근거 → 임계값 근거 → 이론 적용 → 세부 코드 | ✅ **최신 (2026-08-09)** |
| [references.md](references.md) | 참고 논문, 어떻게 썼고 어디서 벗어났는지 | ✅ 최신 |
| [notion_summary.md](notion_summary.md) | 개발 기록 요약 (노션 붙여넣기용) | ✅ 최신 |
| [run_guide.md](run_guide.md) | 다른 컴퓨터에서 실행하는 법 | ✅ 최신 |
| [robot_team_handoff.md](robot_team_handoff.md) | 로봇 파트 연동 전달사항 | 진행 중 |
| [operator_protocol.md](operator_protocol.md) | RC카 조작 매뉴얼 (시연 담당자 필독) | ✅ 최신 |

## 삭제된 문서 (2026-08-09)

`code_review_master.md`, `code_review_study_guide.md`, `code_walkthrough.md`

2026-08-06 기준으로 작성되어 **엔드게임 협공과 재-SLAM 맵 좌표 이전이 반영되지
않았다.** 성공률·좌표·알고리즘 설명이 모두 현재와 달라, 리뷰 때 열면 발표 내용과
어긋난다. 내용이 필요하면 git 히스토리에서 볼 수 있다:

```bash
git log --diff-filter=D --oneline -- docs/code_review_master.md
git show <커밋>^:intelligence1_algorithm/herding_controller_dual/docs/code_review_master.md
```

대체 문서는 [`code_review_script.md`](code_review_script.md)다.
