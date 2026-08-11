-- db_node MySQL 초기화. PC1에서 한 번만 실행.
--   mysql -u root -p < config/mysql_init.sql
-- 전부 CREATE ... IF NOT EXISTS라 재실행해도 기존 데이터는 지워지지 않는다.

CREATE DATABASE IF NOT EXISTS underguard;
USE underguard;

-- Robot 런타임의 옛 기준 테이블. opening으로 이전된 뒤로는 db_node가 더 이상
-- 쓰지 않는 동결 스냅숏이다 — 호환성/이력 조회용으로만 남겨둔다. DROP하지 않는다.
CREATE TABLE IF NOT EXISTS holes (
    id INTEGER PRIMARY KEY AUTO_INCREMENT,
    x DOUBLE,
    y DOUBLE,
    trap_installed INTEGER
);

-- ── 아래부터 확장 테이블 ─────────────────────────────────────

CREATE TABLE IF NOT EXISTS incident (
    incident_id INTEGER PRIMARY KEY AUTO_INCREMENT,
    first_detected_at DATETIME,
    first_x DOUBLE,
    first_y DOUBLE,
    severity VARCHAR(20),
    status VARCHAR(20),
    captured_at DATETIME,
    response_seconds INTEGER,
    result VARCHAR(20),
    image_path VARCHAR(255)
);

-- object_type: RAT / DROPPING / OPENING
-- (DROPPING을 감지하는 파이프라인이 아직 없어 db_node는 RAT/OPENING만 씀)
-- incident_id는 NULL 허용 — OPENING 탐지처럼 사건과 무관한 탐지도 있다.
CREATE TABLE IF NOT EXISTS detection_event (
    detection_id INTEGER PRIMARY KEY AUTO_INCREMENT,
    incident_id INTEGER,
    detected_at DATETIME,
    object_type VARCHAR(20),
    x DOUBLE,
    y DOUBLE,
    confidence DOUBLE,
    robot_id VARCHAR(20),
    image_path VARCHAR(255),
    FOREIGN KEY (incident_id) REFERENCES incident(incident_id)
);

CREATE TABLE IF NOT EXISTS rat_track (
    track_id INTEGER PRIMARY KEY AUTO_INCREMENT,
    incident_id INTEGER NOT NULL,
    `timestamp` DATETIME,
    x DOUBLE,
    y DOUBLE,
    confidence DOUBLE,
    FOREIGN KEY (incident_id) REFERENCES incident(incident_id)
);

-- status: CANDIDATE / VERIFIED / MISSING / REVERIFY / SEALED
-- holes를 대체하는 Robot 런타임 기준 테이블 (db_node가 QueryHole/ListHoles/
-- opening_confirmed을 이제 여기로 처리한다).
CREATE TABLE IF NOT EXISTS opening (
    opening_id INTEGER PRIMARY KEY AUTO_INCREMENT,
    x DOUBLE,
    y DOUBLE,
    first_seen_at DATETIME,
    last_seen_at DATETIME,
    status VARCHAR(20),
    last_verified_at DATETIME,
    verification_count INTEGER DEFAULT 0
);

-- holes -> opening 1회성 이전. x,y가 이미 opening에 있으면 건너뛰어 재실행해도
-- 중복 이전되지 않는다. 최초 발견 시각은 holes에 없어 모르므로(억지 값 대신)
-- first_seen_at/last_seen_at/last_verified_at은 NULL로 둔다.
INSERT INTO opening (x, y, status, verification_count)
SELECT h.x, h.y, 'VERIFIED', 0
FROM holes h
WHERE NOT EXISTS (
    SELECT 1 FROM opening o WHERE o.x = h.x AND o.y = h.y
);

-- status: ACTIVE / MOVED / MISSING / CAPTURED
-- (db_node는 현재 ACTIVE/MOVED만 실제로 씀 — MISSING/CAPTURED는 그 상태를
--  구분해 알려주는 이벤트가 아직 없어 스키마만 준비, 3단계에서도 미사용)
CREATE TABLE IF NOT EXISTS trap (
    trap_id INTEGER PRIMARY KEY AUTO_INCREMENT,
    opening_id INTEGER NOT NULL,
    x DOUBLE,
    y DOUBLE,
    installed_at DATETIME,
    status VARCHAR(20),
    last_checked_at DATETIME,
    total_capture_count INTEGER DEFAULT 0,
    FOREIGN KEY (opening_id) REFERENCES opening(opening_id)
);

CREATE TABLE IF NOT EXISTS trap_inspection (
    inspection_id INTEGER PRIMARY KEY AUTO_INCREMENT,
    trap_id INTEGER NOT NULL,
    robot_id VARCHAR(20),
    checked_at DATETIME,
    result VARCHAR(20),
    detected_x DOUBLE,
    detected_y DOUBLE,
    distance_from_opening DOUBLE,
    FOREIGN KEY (trap_id) REFERENCES trap(trap_id)
);

-- role: PATROL / TRACK / HERD / SWEEP / RETURN
-- (robot_agent 상태머신엔 DOCK만의 지속 상태가 따로 없다 — 도킹은 RETURN
--  상태 안에서 함께 일어나므로 DOCK은 db_node가 별도로 기록하지 않는다)
-- incident_id는 NULL 허용 — PATROL/RETURN은 특정 사건과 무관하다.
CREATE TABLE IF NOT EXISTS robot_mission (
    mission_id INTEGER PRIMARY KEY AUTO_INCREMENT,
    incident_id INTEGER,
    robot_id VARCHAR(20),
    role VARCHAR(20),
    started_at DATETIME,
    ended_at DATETIME,
    result VARCHAR(20),
    duration_sec INTEGER,
    failure_reason VARCHAR(255),
    FOREIGN KEY (incident_id) REFERENCES incident(incident_id)
);

-- db_node 전용 계정 (root 대신 사용). 비밀번호는 여기 적지 말고 실행 시 바꿔서 넣는다.
-- CREATE USER IF NOT EXISTS 'underguard'@'localhost' IDENTIFIED BY 'CHANGE_ME';
-- GRANT ALL ON underguard.* TO 'underguard'@'localhost';
-- FLUSH PRIVILEGES;
