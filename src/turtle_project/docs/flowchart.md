# Under-Guard 데이터 흐름도

```mermaid
flowchart TB
    oak["센서 원본"]:::ext
    nav2["Nav2"]:::ext

    subgraph robotpc["로봇 PC — robot4·robot6 각각 실행"]
        cam["카메라 sync"]:::done
        det["감지"]:::partial
        trap["trap 점검"]:::todo
        agent["로봇 주행"]:::partial
    end

    subgraph centralpc["중앙 PC"]
        central["중앙 조율"]:::partial
        herd["쥐몰이"]:::partial
        db["구멍 DB"]:::todo
        webcam["웹캠 감시"]:::todo
    end

    oak -->|"rgb·depth 원본"| cam
    cam -->|"synced 3종"| det
    cam -->|"synced 영상"| trap
    det -->|"target_pose"| agent
    det -->|"event"| central
    det -->|"구멍 조회"| db
    det -->|"rat 위치"| herd
    trap -->|"trap_ok"| central
    webcam -->|"rat_detected"| central
    agent -->|"status"| central
    agent -->|"goal 주행"| nav2
    central -->|"command"| agent
    central -->|"TRACK/PATROL"| det
    central -->|"HERD"| herd
    herd -->|"target_pose"| agent

    subgraph legend["범례"]
        L1["완성 done"]:::done
        L2["부분 partial"]:::partial
        L3["뼈대 todo"]:::todo
    end

    classDef done fill:#c6f6d5,stroke:#22863a,color:#000
    classDef partial fill:#fff3c4,stroke:#b7791f,color:#000
    classDef todo fill:#fed7d7,stroke:#c53030,color:#000
    classDef ext fill:#e2e8f0,stroke:#4a5568,color:#000
```

## 통신 규칙

- 노드 간 통신은 String 토픽 3개: `/fleet/status`, `/fleet/command`, `/fleet/event`
- `target_pose`(PoseStamped): **detector·쥐몰이가 발행 → robot_agent만 구독** (로봇당 Nav2 주인 1개)
- robot A(추적)/B(몰이)는 고정 아님 — central이 `TRACK`/`HERD` 명령으로 런타임 배정
