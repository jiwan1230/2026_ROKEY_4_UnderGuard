# 코드 리뷰 준비 폴더 (herding_controller_dual)

발표용으로 필요한 자료를 한 곳에 모았다. 순서대로 보면 된다.

## 한 페이지로 전부 — `full_review_page.html`

**[`full_review_page.html`](full_review_page.html)** — 더블클릭해서 브라우저로 열면 된다.
아래 1~7번 전부(설계·개발과정·코드·테스트·Q&A·리스크·참고문헌)를 GIF까지 전부 박아넣어
한 페이지로 합친 것 — 폴더를 오가며 파일을 열 필요 없이 스크롤만 하면 된다.
아티팩트로도 발행되어 있다: https://claude.ai/code/artifact/f6af1341-2131-447a-ae78-cf11d8d6f1b5
(3D 리플레이 아티팩트로 가는 링크도 "7 참고문헌" 맨 아래에 있음 — 그 페이지 자체는 내용을
확인 못 해서 이동 링크만 걸어뒀다).

아래 1~4번은 그 안에 들어간 원본 자료들의 개별 파일 위치다 (참고용, 위 한 페이지짜리로도 충분함).

## 0. 개발 과정 8단계 타임라인 (강사 피드백 대응)

**[`timeline_8_stages.md`](timeline_8_stages.md)** — "왜 엔드게임 협공을
만들게 됐는가"를 1단계(Driver-Blocker 92%)부터 8단계(재-SLAM 좌표 이전,
현재 96/98%)까지 시간순으로 정리. 3단계("16가지 시도 전부 실패")는 표로
전부 정리했고, 1·2·3·4·5단계는 지금 코드로 새로 뽑은 GIF가 딸려 있다
(6단계 협공 GIF는 기존 것 재사용). 3단계의 "게이트" GIF만 원본 코드가
안 남아있어 재구성판임을 명시했다.

## 1. 설계도

**[`design_diagram.html`](design_diagram.html)** — 더블클릭해서 브라우저로 열면 된다
(인터넷 연결 불필요, 완전히 자체 포함된 파일).

- 메시지 인터페이스: `herding_node.py`의 구독/발행 토픽과 노드 경계
- 6+1 상태 FSM (`state_machine.py`)
- `HerdingCore.step()` 전체 제어 플로우 — FSM 분기, 동적 역할 배정,
  일반 몰이(Driving/Blocking Point) vs **엔드게임 협공**(`compute_endgame_pincer`)
  분기까지 코드 파일:라인 번호와 함께 그렸다.
- 각 박스 아래에 실제 코드 위치(`파일.py:줄번호`)를 달아서, 설계도 → 코드가
  바로 대응되도록 했다.

## 2. 코드

이 폴더에는 코드를 복사해두지 않았다 — **패키지 디렉토리를 그대로 열어서 보여준다.**
리뷰 때는 `herding_controller_dual/`(이 폴더의 상위 폴더)를 열어두면 된다.

```
herding_controller_dual/
├── herding_controller_dual/   ← 알고리즘 코어 (rclpy 의존성 0, herding_node.py 제외)
│   ├── herding_core.py          HerdingCore.step() — 설계도 그림 3
│   ├── herding_planner.py       Driving/Blocking Point, 엔드게임 협공
│   ├── state_machine.py         FSM — 설계도 그림 2
│   ├── role_assigner.py         동적 Driver/Blocker 배정
│   ├── escape_model.py          마르코프 도주 방향 예측
│   ├── target_estimator.py      칼만 필터
│   ├── geodesic_field.py        벽 고려 목표 방향 필드
│   ├── occlusion_grid.py        LOST 재탐색 belief 그리드
│   ├── grid_map.py               격자 좌표 변환
│   └── herding_node.py          ← rclpy를 import하는 유일한 파일, 설계도 그림 1
├── config/herding_params.yaml   ROS 파라미터 기본값
├── test/                        오프라인 검증 하네스 (아래 3번 항목)
└── README.md                     패키지 전체 설명 (`readme_copies/package_README.md`에 사본)
```

## 3. README

- [`readme_copies/package_README.md`](readme_copies/package_README.md) — 패키지 루트 README 사본
  (디렉토리 구조, 검증 결과, 빌드/실행법, 메시지 인터페이스 요약)
- [`readme_copies/docs_README.md`](readme_copies/docs_README.md) — 문서 색인
- [`readme_copies/code_review_script.md`](readme_copies/code_review_script.md) — **코드 리뷰 발표 대본**
  (디렉토리 → 설계 근거 → 임계값 근거 → 이론 적용 → 세부 코드 순서)

원본은 항상 `herding_controller_dual/README.md`, `herding_controller_dual/docs/`에 있다 —
여기 있는 건 발표 중 폴더를 옮겨다니지 않기 위한 사본이니, 최신 내용 확인은 원본 기준.

## 4. 테스트 자료

모두 2026-08-10, 이 브랜치(`algorithm/intelligence1-herding-blocker-redesign`) 기준으로
직접 재실행해서 새로 뽑은 로그다.

**GIF는 전부 2배속이다** (원본 시뮬레이션 시간의 절반으로 재생됨 — 아래 "n초 지속" 등의
수치는 시뮬레이션 상의 실제 경과 시간이지 영상 재생 시간이 아니다). 화면 캡처 자체를
다시 뜬 게 아니라 프레임 표시 시간만 절반으로 줄인 것이라 내용은 원본과 동일하다.

| 파일 | 내용 | 실행 명령 |
|---|---|---|
| [`tests/pytest_log.txt`](tests/pytest_log.txt) | 단위 테스트 요약 — **177 passed** | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q` |
| [`tests/pytest_log_verbose.txt`](tests/pytest_log_verbose.txt) | 위와 동일하되 테스트 177개 각각의 이름 전부 | 위 명령 + `-v` |
| [`tests/run_validation_log.txt`](tests/run_validation_log.txt) | **정식 통계 검증** 전체 로그 — 추상 아레나 ALGO-001~008 8/8 PASS, 실제맵 성공률 reactive_flee 96.0% / noisy_human 98.0% (트랩당 100회×3트랩) | `python3 -m test.run_validation` |
| [`tests/pincer_demo_log.txt`](tests/pincer_demo_log.txt) | 엔드게임 협공 36회 시행 요약 (발동률 100%, 성공 75%, 사잇각·지속시간) | `python3 -m experiments.pincer_demo --trials 36 --render 2` |
| [`tests/media/endgame_pincer_demo_1.gif`](tests/media/endgame_pincer_demo_1.gif) | 협공 시연 GIF #1 — 시드 2000000, top 트랩, 사잇각 최대 141°, 23.8초 지속 | 위 명령이 자동 생성 |
| [`tests/media/endgame_pincer_demo_2.gif`](tests/media/endgame_pincer_demo_2.gif) | 협공 시연 GIF #2 — 시드 2000030, top 트랩, 사잇각 최대 132°, 11.0초 지속 | 위 명령이 자동 생성 |
| [`tests/media/stage1_driver_blocker_baseline.gif`](tests/media/stage1_driver_blocker_baseline.gif) | 1단계: 플랜A(협공 없음) 구조 그대로 몰이 | 새 스크립트(`herding_controller`, ReactiveFlee) |
| [`tests/media/stage2_robot_b_zero_contribution.gif`](tests/media/stage2_robot_b_zero_contribution.gif) | 2단계: 같은 시드로 막는 로봇 정상/고정 나란히 비교 — 결과 동일 | `blocker_contribution_ablation.py`의 frozen-blocker 방식 재사용 |
| [`tests/media/stage3_sealing_line_impossible.gif`](tests/media/stage3_sealing_line_impossible.gif) | 3단계: 봉인 선분이 실제 경로 내내 성립 안 함(0%) | `compute_sealing_pair()`를 실제 궤적에 매 순간 적용 |
| [`tests/media/stage3_gate_reconstruction.gif`](tests/media/stage3_gate_reconstruction.gif) | 3단계: 게이트 설치 [재구성판] — 있음 0/20 vs 없음 19/20 | 트랩 앞 칸막이를 새로 재구성 (원본 좌표 없음) |
| [`tests/media/stage4_realistic_target_collapse.gif`](tests/media/stage4_realistic_target_collapse.gif) | 4단계 [재구성]: 협공 이전 + 구석회피 표적 → 실패 | 협공 끈 dual 설정 + CorneringAwareFlee |
| [`tests/media/stage5_last_7cm_failure.gif`](tests/media/stage5_last_7cm_failure.gif) | 5단계 [재구성]: 트랩 0.40m 앞까지 왔다가 옆으로 빠짐 | 위와 같은 조건, 최근접 실패 시행 |

8단계 전체 이야기와 각 GIF의 맥락은 [`timeline_8_stages.md`](timeline_8_stages.md) 참고.

리뷰 매뉴얼 기준 "기능 테스트까지만" 리뷰 대상이므로, 여기 담긴 것도 전부 오프라인
시뮬레이션 검증(`test/`)이다 — 로봇 파트와의 실제 토픽 통신 테스트는 이 파트 담당이 아니다.

### 주의 — 전부 시뮬레이션이다

모든 수치는 시뮬레이션 결과다. **실물 로봇 검증은 아직 하지 않았다.** 시뮬레이터에
반영된 것: 실제 SLAM 맵, 로봇 몸통 반경, RC카 크기, 로봇 간 충돌, 덫의 물리적 포획.
반영 안 된 것: 인식 지연·오차, Nav2 경로 추종 오차, 바퀴 미끄러짐. 완전히 무작위로
움직이는 표적은 못 잡는다 (성공률 13%까지 하락) — 이 알고리즘은 "표적이 다가오는
로봇에 반응해서 도망친다"는 전제 위에 서 있다. 자세한 내용은
`readme_copies/code_review_script.md`의 한계 설명 참고.
