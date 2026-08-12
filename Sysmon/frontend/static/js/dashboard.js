const cfg = window.MONITOR_CONFIG || {pollInterval : 1000, mode : 'mock'};
let selectedRobot = null;
let lastSnapshot = null;
let showTrails = true;
let showDetectionMarkers = true;
let mapMetadata = null;
let mapImage = null;
let mapMarkerHits = [];
const robotTrails = new Map();

const $ = selector => document.querySelector(selector);
const formatTime = unix => {
  // 브라우저 언어와 관계없이 발표 화면의 시간을 짧은 24시간제로 표시한다.
  const date = new Date(Number(unix) * 1000);
  const pad = value => String(value).padStart(2, '0');
  return `${pad(date.getHours())}:${pad(date.getMinutes())}:${
      pad(date.getSeconds())}`;
};
const n = (value, digits = 1) =>
    value == null ? '—' : Number(value).toFixed(digits);
const escapeHtml = value =>
    String(value ?? '').replace(/[&<>'"]/g, char => ({
                                              '&' : '&amp;',
                                              '<' : '&lt;',
                                              '>' : '&gt;',
                                              '\'' : '&#39;',
                                              '"' : '&quot;'
                                            }[char]));
const stateLabels = {
  OFFLINE : '연결 끊김',
  IDLE : '대기',
  SEARCHING : '탐색 중',
  APPROACHING : '접근 중',
  TRACKING : '추적 중',
  TARGET_LOST : '대상 유실',
  NAVIGATING : '이동 중',
  INSTALLING_TRAP : '트랩 상태 확인 중',
  RETURNING : '복귀 중',
  COMPLETED : '임무 완료 · 대기 중',
  PAUSED : '일시정지',
  ERROR : '오류'
};
// PAUSED는 명령 API가 삭제되며 도달 불가능해져 제거했다(mock_manager.py 참고).
// 전체 임무 상태는 이제 로봇 state 우선순위로 서버가 매번 계산한다(state_manager.py
// _derive_mission_status 참고). 코드는 우선순위 그대로: ERROR > TARGET_LOST >
// TRACKING > VERIFYING > RETURNING > IDLE.
const missionLabels = {
  ERROR : '오류 확인 필요',
  TARGET_LOST : '설치류 대응 중 · 대상 유실',
  TRACKING : '설치류 대응 중',
  VERIFYING : '주변 위험요소 확인 중',
  RETURNING : '복귀 중',
  IDLE : '대기 중'
};
const roleLabels = {
  SCOUT : '공동 탐색·역할 대기',
  RAT_TRACKER : '설치류 관찰·추적',
  SURVEY_TRAP : '침입구·트랩 상태 확인',
  UNASSIGNED : '역할 미지정'
};
// 좁은 로봇 선택 탭에는 역할의 핵심만 표시하고 상세 카드에는 전체명을 쓴다.
const shortRoleLabels = {
  SCOUT : '공동 탐색',
  RAT_TRACKER : '설치류 추적',
  SURVEY_TRAP : '위험요소 확인',
  UNASSIGNED : '역할 대기'
};
const objectLabels = {
  LIVE_RODENT : '쥐',
  ENTRY_POINT : '쥐구멍',
  DROPPINGS : '배설물',
  rc_car : '쥐',
  rat_hole : '쥐구멍',
  droppings : '배설물'
};
const detectionColors = {
  LIVE_RODENT : '#ff453a',
  ENTRY_POINT : '#bf5af2',
  DROPPINGS : '#b9975b',
  rc_car : '#ff453a',
  rat_hole : '#bf5af2',
  droppings : '#b9975b'
};
const detectionClassNames = {
  LIVE_RODENT : 'target-rodent',
  rc_car : 'target-rodent',
  ENTRY_POINT : 'target-entry-point',
  rat_hole : 'target-entry-point',
  DROPPINGS : 'target-droppings',
  droppings : 'target-droppings'
};
const localizeObjectText = value => String(value ?? '')
                                        .replaceAll('LIVE_RODENT', '쥐')
                                        .replaceAll('ENTRY_POINT', '쥐구멍')
                                        .replaceAll('DROPPINGS', '배설물')
                                        .replaceAll('rc_car', '쥐')
                                        .replaceAll('rat_hole', '쥐구멍')
                                        .replaceAll('droppings', '배설물')
                                        .replaceAll('RC Car', '쥐');

/**
 * 위험신호 종류를 서로 다른 모양과 색상으로 Canvas에 그린다.
 * 입력: Canvas 문맥, 객체 종류, 화면 좌표, 크기와 투명도다.
 * 출력: 없음. 쥐·쥐구멍·배설물 마커를 현재 Canvas에 추가한다.
 * 사용: 현재 대상과 이번 실행 중 수신한 탐지를 같은 규칙으로 표시한다.
 */
function drawDetectionMarker(ctx, objectType, point, size = 5, alpha = 1) {
  const color = detectionColors[objectType] || '#ff453a';
  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.globalAlpha = alpha;
  ctx.fillStyle = color;
  ctx.strokeStyle = 'rgba(245,245,247,.85)';
  ctx.lineWidth = 1;
  if ([ 'ENTRY_POINT', 'rat_hole' ].includes(objectType)) {
    ctx.rotate(Math.PI / 4);
    ctx.fillRect(-size * .72, -size * .72, size * 1.44, size * 1.44);
    ctx.strokeRect(-size * .72, -size * .72, size * 1.44, size * 1.44);
  } else if ([ 'DROPPINGS', 'droppings' ].includes(objectType)) {
    ctx.beginPath();
    ctx.moveTo(0, -size);
    ctx.lineTo(size * .9, size * .75);
    ctx.lineTo(-size * .9, size * .75);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
  } else {
    ctx.beginPath();
    ctx.arc(0, 0, size, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = 'rgba(255,255,255,.75)';
    ctx.beginPath();
    ctx.arc(0, 0, Math.max(1, size * .22), 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.restore();
}

/**
 * 마커 주변에서 겹치지 않는 후보를 골라 어두운 배경 라벨을 그린다.
 * 입력: Canvas 문맥, 라벨 문자열·색상·기준점, 사용 중인 영역과 화면 크기다.
 * 출력: 없음. 선택한 라벨 영역을 occupied에 추가한다.
 * 사용: 수신 탐지, 덫, 로봇, 현재 대상 라벨을 같은 규칙으로 표시한다.
 */
function drawMapLabel(ctx, text, point, color, occupied, viewport) {
  ctx.save();
  ctx.font = 'bold 11px Arial';
  ctx.textBaseline = 'middle';
  const width = Math.ceil(ctx.measureText(text).width) + 12;
  const height = 22;
  const candidates = [
    {x : point.x + 12, y : point.y - height - 10},
    {x : point.x + 12, y : point.y + 10},
    {x : point.x - width - 12, y : point.y - height - 10},
    {x : point.x - width - 12, y : point.y + 10},
    {x : point.x - width / 2, y : point.y - height - 18},
    {x : point.x - width / 2, y : point.y + 18},
    {x : point.x + 34, y : point.y - height / 2},
    {x : point.x - width - 34, y : point.y - height / 2},
    {x : point.x + 38, y : point.y - height - 28},
    {x : point.x + 38, y : point.y + 28},
    {x : point.x - width - 38, y : point.y - height - 28},
    {x : point.x - width - 38, y : point.y + 28}
  ];
  const intersectionArea = (left, right) => {
    const overlapWidth =
        Math.max(0, Math.min(left.x + left.width, right.x + right.width) -
                        Math.max(left.x, right.x));
    const overlapHeight =
        Math.max(0, Math.min(left.y + left.height, right.y + right.height) -
                        Math.max(left.y, right.y));
    return overlapWidth * overlapHeight;
  };
  const rectangles = candidates
                         .map(candidate => ({
                                x : Math.max(
                                    6, Math.min(candidate.x,
                                                viewport.width - width - 6)),
                                y : Math.max(
                                    6, Math.min(candidate.y,
                                                viewport.height - height - 6)),
                                width,
                                height
                              }))
                         .filter((candidate, index, all) =>
                                     all.findIndex(item =>
                                                       item.x === candidate.x &&
                                                       item.y === candidate.y) ===
                                     index);
  // 빈 위치가 없을 때도 첫 후보로 되돌아가지 않고 겹치는 면적이 가장 작은
  // 위치를 택한다. 밀집된 탐지에서도 라벨이 한곳에 포개지는 현상을 줄인다.
  const box = rectangles
                  .map(candidate => ({
                         ...candidate,
                         overlap : occupied.reduce(
                             (sum, item) =>
                                 sum + intersectionArea(candidate, item),
                             0),
                         distance : Math.hypot(
                             candidate.x + candidate.width / 2 - point.x,
                             candidate.y + candidate.height / 2 - point.y)
                       }))
                  .sort((left, right) =>
                            left.overlap - right.overlap ||
                            left.distance - right.distance)[0];
  occupied.push(box);
  const labelAnchor = {
    x : Math.max(box.x, Math.min(point.x, box.x + box.width)),
    y : Math.max(box.y, Math.min(point.y, box.y + box.height))
  };
  if (Math.hypot(labelAnchor.x - point.x, labelAnchor.y - point.y) > 4) {
    ctx.strokeStyle = color;
    ctx.globalAlpha = .5;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(point.x, point.y);
    ctx.lineTo(labelAnchor.x, labelAnchor.y);
    ctx.stroke();
    ctx.globalAlpha = 1;
  }
  ctx.fillStyle = 'rgba(0,0,0,.88)';
  ctx.strokeStyle = color;
  ctx.lineWidth = 1;
  ctx.fillRect(box.x, box.y, box.width, box.height);
  ctx.strokeRect(box.x, box.y, box.width, box.height);
  ctx.fillStyle = color;
  ctx.fillText(text, box.x + 6, box.y + height / 2 + .5);
  ctx.restore();
}

/**
 * 라벨이 로봇·탐지 마커를 가리지 않도록 화면상의 마커 영역을 예약한다.
 * 입력: 사용 중인 영역 배열, Canvas 좌표와 마커 주위 여백이다.
 * 출력: 없음. drawMapLabel이 피해야 할 사각형을 occupied에 추가한다.
 * 사용: 한 프레임의 라벨을 그리기 전에 표시 가능한 모든 마커를 등록한다.
 */
function reserveMapMarkerArea(occupied, point, radius = 8) {
  occupied.push({
    x : point.x - radius,
    y : point.y - radius,
    width : radius * 2,
    height : radius * 2
  });
}

/**
 * Canvas 마커의 클릭 판정 정보를 현재 프레임 목록에 등록한다.
 * 입력: 화면 좌표, 마커 종류, 원본 데이터와 클릭 반경이다.
 * 출력: 없음. `mapMarkerHits`에 클릭 가능한 영역을 추가한다.
 * 사용: 지도 렌더링 후 `markerAtPointer()`가 가장 가까운 마커를 찾는다.
 */
function addMapMarkerHit(point, kind, data, radius = 17) {
  mapMarkerHits.push({point, kind, data, radius});
}

/**
 * 탐지 마커와 구분되는 초록 사각형으로 설치된 덫을 그린다.
 * 입력: Canvas 문맥, 덫 데이터와 화면 좌표다.
 * 출력: 없음. 현재 Canvas에 덫 아이콘을 추가한다.
 * 사용: `drawMap()`이 `/api/snapshot`의 traps를 순회할 때 호출한다.
 */
function drawTrapMarker(ctx, trap, point) {
  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.fillStyle = '#32d74b';
  ctx.strokeStyle = '#f5f5f7';
  ctx.lineWidth = 1;
  ctx.fillRect(-6, -6, 12, 12);
  ctx.strokeRect(-6, -6, 12, 12);
  ctx.strokeStyle = '#1c1c1e';
  ctx.beginPath();
  ctx.moveTo(-3, 0);
  ctx.lineTo(3, 0);
  ctx.moveTo(0, -3);
  ctx.lineTo(0, 3);
  ctx.stroke();
  ctx.restore();
}

/**
 * 클릭한 지도 마커를 상세 카드에 표시한다.
 * 입력: 종류(kind), 화면 좌표, 탐지/덫 데이터를 포함한 마커 객체다.
 * 출력: 없음. 시간·로봇·신뢰도/상태·좌표 DOM을 갱신한다.
 * 사용: map canvas 클릭 이벤트에서 선택된 마커를 전달한다.
 */
function showMapMarkerDetail(marker) {
  const detail = $('#map-marker-detail');
  const data = marker.data;
  const isTrap = marker.kind === 'trap';
  const label =
      isTrap ? `설치된 덫 #${data.id}`
             : (objectLabels[data.object_type] || data.object_type || '대상');
  const count = Number(data.count || 1);
  const confidence = data.confidence == null
                         ? null
                         : `${Math.round(Number(data.confidence) * 100)}%`;
  let info = isTrap ? (data.status === 'INSTALLED' ? '설치 완료' : data.status)
                    : (confidence ? `신뢰도 ${confidence}` : '신뢰도 —');
  if (!isTrap && count > 1)
    info += ` · ${count}건`;

  const titlePrefix = marker.kind === 'current' ? '현재 ' : '';
  const titleSuffix = marker.kind === 'detection' ? ' 감지' : '';
  $('#map-marker-title').textContent =
      isTrap ? label : `${titlePrefix}${label}${titleSuffix}`;
  const timestamp = Number(data.timestamp);
  $('#map-marker-time').textContent =
      Number.isFinite(timestamp) && timestamp > 0 ? formatTime(timestamp) : '—';
  $('#map-marker-robot').textContent = data.robot_id || '—';
  $('#map-marker-info').textContent = info;
  $('#map-marker-position').textContent =
      data.map_x == null || data.map_y == null
          ? '—'
          : `${n(data.map_x, 2)}, ${n(data.map_y, 2)}`;
  // 탐지 시점과 정확히 동기화된 프레임은 아니고, 그 로봇의 최근 캐시
  // 프레임을 가리키는 링크다(카메라 미연결/Mock이면 비어 있음).
  const imageCell = $('#map-marker-image');
  imageCell.innerHTML = data.image_url
      ? `<a href="${escapeHtml(data.image_url)}" target="_blank" rel="noopener">보기</a>`
      : '—';
  detail.classList.remove('hidden');
}

/**
 * 같은 종류의 근접 탐지를 지도 마커 한 개와 건수로 묶는다.
 * 입력: 최신 탐지 배열과 같은 위치로 판단할 반경(m)이다.
 * 출력: 대표 탐지 데이터에 count가 추가된 클러스터 배열이다.
 * 사용: `drawMap()`이 같은 위치의 반복 탐지를 `×N`으로 표시할 때 호출한다.
 */
function clusterReceivedDetections(detections, radius = .15) {
  const clusters = [];
  detections.forEach(detection => {
    if (detection.map_x == null || detection.map_y == null)
      return;
    const existing = clusters.find(
        item => item.object_type === detection.object_type &&
                Math.hypot(item.map_x - detection.map_x,
                           item.map_y - detection.map_y) <= radius);
    if (existing)
      existing.count += 1;
    else
      clusters.push({...detection, count : 1});
  });
  return clusters;
}

function toast(message, type = 'success') {
  const item = document.createElement('div');
  item.className = `toast ${type}`;
  item.textContent = message;
  $('#toast-region').append(item);
  setTimeout(() => item.remove(), 3200);
}

/**
 * JSON API를 공통 방식으로 호출한다.
 * 입력: API URL과 선택적인 fetch 옵션이다.
 * 출력: 파싱된 JSON Promise이며, 실패 응답은 Error로 변환한다.
 * 사용: `await request('/api/snapshot')`처럼 호출한다.
 */
async function request(url, options = {}) {
  const response = await fetch(url, options);
  const data = await response.json();
  if (!response.ok)
    throw new Error(data.error || data.reason || '요청 실패');
  return data;
}

/**
 * 서버 스냅샷 한 건을 대시보드 전체 영역에 반영한다.
 * 입력: `/api/snapshot`의 요약·임무·로봇·사건·탐지 객체다.
 * 출력: 없음. DOM, 이동 궤적, 지도 canvas를 갱신한다.
 * 사용: 최초 로딩과 주기적 `poll()` 성공 시 호출한다.
 */
function render(snapshot) {
  lastSnapshot = snapshot;
  renderOperationsStatus(snapshot);

  snapshot.robots.forEach(robot => {
    const trail = robotTrails.get(robot.robot_id) || [];
    const point = {
      x : robot.position.x,
      y : robot.position.y,
      frame : robot.position_frame || 'unknown'
    };
    const previous = trail[trail.length - 1];
    if (previous && previous.frame !== point.frame)
      trail.length = 0;
    if (!previous ||
        Math.hypot(point.x - previous.x, point.y - previous.y) > .02)
      trail.push(point);
    robotTrails.set(robot.robot_id, trail.slice(-60));
  });

  if (!selectedRobot && snapshot.robots.length)
    selectedRobot = snapshot.robots[0].robot_id;
  renderRobots(snapshot.robots, snapshot.runtime, snapshot.mission);
  renderCameras(snapshot.robots, snapshot.server_time);
  renderEvents(snapshot.events);
  drawMap(snapshot);
}

/**
 * 정상 상태는 연결 요약으로 축약하고 이상 상태만 상단 배너에 표시한다.
 * 입력: 로봇·요약·런타임을 포함한 `/api/snapshot` 응답이다.
 * 출력: 없음. 제목 옆 연결 문구와 예외 배너를 갱신한다.
 * 사용: 스냅샷을 받을 때마다 `render()`가 호출한다.
 */
function renderOperationsStatus(snapshot) {
  const online = Number(snapshot.summary.robots_online || 0);
  const total = Number(snapshot.summary.robots_total || 0);
  const dot = $('#fleet-connection-dot');
  const state = $('#fleet-connection-state');
  const banner = $('#operations-alert-banner');
  const message = $('#operations-alert-message');
  const allConnected = total > 0 && online === total;
  // Mock 모드는 실제 네트워크 연결이 아니라 시뮬레이션이므로 "연결됨" 대신
  // "활성"으로 표현해 평가자가 실연동과 시뮬레이션을 혼동하지 않게 한다.
  const isMock = cfg.mode === 'mock';
  const unit = isMock ? '활성' : '연결';
  const noneText = isMock ? '활성 Mock 로봇 없음' : '연결된 로봇 없음';

  dot.className = allConnected ? 'online' : (online > 0 ? 'warning' : 'danger');
  state.textContent =
      allConnected
          ? (isMock ? `Mock 로봇 ${total}대 활성` : `로봇 ${total}대 연결됨`)
          : (online > 0 ? `로봇 ${online}/${total}대 ${unit}` : noneText);

  const warnings = [];
  const offline = snapshot.robots
                      .filter(robot => robot.connection === 'OFFLINE')
                      .map(robot => robot.robot_id);
  if (offline.length)
    warnings.push(`${offline.join(', ')} 연결이 끊어졌습니다.`);

  const lowBatteryThreshold = Number(snapshot.runtime?.low_battery_threshold || 15);
  const lowBattery = snapshot.robots
                         .filter(robot => robot.connection === 'ONLINE' &&
                             robot.battery != null &&
                             Number(robot.battery) < lowBatteryThreshold)
                         .map(robot => robot.robot_id);
  if (lowBattery.length)
    warnings.push(`${lowBattery.join(', ')} 배터리가 부족합니다.`);

  const lostTargets = snapshot.robots
                          .filter(robot => robot.state === 'TARGET_LOST')
                          .map(robot => robot.robot_id);
  if (lostTargets.length)
    warnings.push(`${lostTargets.join(', ')}이(가) 추적 대상을 유실했습니다.`);

  const errors = snapshot.robots
                     .filter(robot => robot.state === 'ERROR')
                     .map(robot => robot.robot_id);
  if (errors.length)
    warnings.push(`${errors.join(', ')} 오류 상태를 확인해 주세요.`);

  message.textContent = warnings.join(' · ');
  banner.classList.toggle('hidden', warnings.length === 0);
}

/**
 * 선택 로봇의 읽기 전용 상태 카드를 렌더링한다.
 * 입력: 로봇 배열, 경고 기준을 담은 런타임 상태, 전체 임무 상태다.
 * 출력: 없음. 로봇 선택, 전체 임무 요약, 현재 상태 DOM을 다시 만든다.
 * 사용: 최초 스냅샷과 로봇 선택 후 `render()`에서 호출한다.
 */
function renderRobots(robots, runtime = {}, mission = {}) {
  const lowBatteryThreshold = Number(runtime.low_battery_threshold || 15);
  const robot =
      robots.find(item => item.robot_id === selectedRobot) || robots[0];
  if (!robot) {
    $('#robot-list').innerHTML =
        '<div class="empty">등록된 로봇이 없습니다.</div>';
    return;
  }
  const selector =
      `<div class="robot-selector" role="tablist" aria-label="로봇 선택">${
          robots
              .map(item => `
    <button type="button" role="tab" aria-selected="${
                       item.robot_id === robot.robot_id}" class="robot-tab ${
                       item.robot_id === robot.robot_id
                           ? 'active'
                           : ''}" data-select-robot="${
                       escapeHtml(item.robot_id)}">
      <span><b>${escapeHtml(item.robot_id)}</b><small>${
                       escapeHtml(shortRoleLabels[item.role] ||
                                  item.role)}</small></span>
      <i class="${item.connection === 'ONLINE' ? '' : 'offline'}"></i>
    </button>`)
              .join('')}</div>`;
  // 전체 임무 상태는 선택 탭 바로 아래에 두어 로봇 상태와 함께 읽는다. 역할
  // 배정 전(WAITING)에는 계산된 상태 대신 "역할 배정 전"을 우선 보여준다.
  // role_assignment_status는 한 번 ASSIGNED가 되면 되돌아가지 않으므로 status
  // 값과 별도로만 확인하면 된다.
  const waitingForRoleAssignment = mission.role_assignment_status === 'WAITING';
  const missionText = waitingForRoleAssignment
      ? '역할 배정 전'
      : (missionLabels[mission.status] || mission.status || '대기 중');
  const missionSummary = `
    <div class="fleet-mission-summary">
      <span>전체 임무</span>
      <strong id="mission-status" class="mission-summary-state state-text state-${
          String(mission.status || 'IDLE').toLowerCase()}">${
          escapeHtml(missionText)}</strong>
    </div>`;
  const detail =
      [ robot ]
          .map(robot => {
            const battery =
                robot.battery == null
                    ? 0
                    : Math.max(0, Math.min(100, Number(robot.battery)));
            const isLowBattery = battery < lowBatteryThreshold;
            return `
    <article class="robot-card selected selected-robot-card" data-robot="${
                escapeHtml(robot.robot_id)}">
      <div class="robot-card-head"><div><div class="robot-name">${
                escapeHtml(robot.robot_id)}</div><div class="robot-role">${
                escapeHtml(roleLabels[robot.role] ||
                           robot.role)}</div></div><span class="online-dot ${
                robot.connection === 'ONLINE' ? '' : 'offline'}">${
                robot.connection === 'ONLINE' ? '온라인'
                                              : '오프라인'}</span></div>
      <div class="robot-task state-text state-${
                String(robot.state).toLowerCase()}">${
                escapeHtml(localizeObjectText(robot.current_task))}</div>${
                isLowBattery
                    ? `<div class="battery-advisory">배터리 ${
                          n(robot.battery,
                            0)}% · 복귀 권장 · 신규 확인 임무 제한</div>`
                    : ''}
      <div class="robot-metrics robot-metrics-compact">
        <div class="metric battery-metric"><small>배터리</small><strong>${
                n(robot.battery, 0)}%</strong><span class="battery-mini ${
                isLowBattery
                    ? 'low'
                    : ''}"><i style="width:${battery}%"></i></span></div>
        <div class="metric"><small>속도</small><strong>${
                n(robot.speed, 2)} m/s</strong></div>
        <div class="metric"><small>좌표</small><strong>${
                n(robot.position.x,
                  1)}, ${n(robot.position.y, 1)}</strong></div>
      </div>
    </article>`;
          })
          .join('');
  $('#robot-list').innerHTML = selector + missionSummary + detail;
  document.querySelectorAll('[data-select-robot]')
      .forEach(button => button.addEventListener('click', () => {
        selectedRobot = button.dataset.selectRobot;
        render(lastSnapshot);
      }));
}

/**
 * 로봇별 카메라 카드에 탐지 대상과 장치 상태를 렌더링한다.
 * 입력: 로봇 배열과 연결 갱신 시간을 계산할 서버 시각이다.
 * 출력: 없음. 로봇 식별색과 대상 탐지색을 분리한 카드 DOM을 다시 만든다.
 * 사용: `/api/snapshot`을 반영하는 `render()`에서 호출한다.
 */
function renderCameras(robots, serverTime) {
  const cameraStack = $('#camera-stack');
  const visibleRobots = robots.slice(0, 2);
  if (!visibleRobots.length) {
    cameraStack.innerHTML =
        '<div class="empty">표시할 로봇 카메라가 없습니다.</div>';
    return;
  }

  cameraStack.innerHTML =
      visibleRobots
          .map((robot, index) => {
            const target = robot.target || {};
            const hasTarget = Boolean(target.object_type);
            const targetName = objectLabels[target.object_type] ||
                               target.object_type || '대상';
            const targetClass = hasTarget
                                    ? (detectionClassNames[
                                           target.object_type] ||
                                       'target-unknown')
                                    : 'waiting';
            const confidence =
                hasTarget && target.confidence != null
                    ? `${Math.round(Number(target.confidence) * 100)}%`
                    : '—';
            const cameraError = robot.connection !== 'ONLINE' ||
                                robot.state === 'ERROR';
            const cameraNormal = !cameraError &&
                                 robot.camera_status === 'NORMAL';
            const updateAge = robot.last_update
                                  ? Math.max(0, Number(serverTime || 0) -
                                                    Number(robot.last_update))
                                  : null;
            const source = target.source || '—';
            const isSelected = robot.robot_id === selectedRobot;
            return `
      <article class="panel camera-card camera-${
                index === 0 ? 'robot-primary' : 'robot-secondary'} ${
                isSelected ? 'selected' : ''}" data-camera-robot="${
                escapeHtml(
                    robot
                        .robot_id)}" tabindex="0" role="button" aria-pressed="${
                isSelected}" aria-label="${
                escapeHtml(robot.robot_id)} 카메라 선택">
        <header class="camera-card-header">
          <div class="camera-identity">
            <span class="camera-number">CAM ${index + 1}</span>
            <strong>${escapeHtml(robot.robot_id)}</strong>
            <small>${escapeHtml(roleLabels[robot.role] || robot.role)}</small>
          </div>
          <span class="camera-connection ${
                cameraNormal ? 'normal'
                             : (cameraError ? 'error' : 'warning')}"><i></i>${
                cameraNormal
                    ? '정상'
                    : (robot.connection !== 'ONLINE' ? '연결 끊김'
                                                     : '확인 필요')}${
                updateAge == null ? '' : ` · ${updateAge.toFixed(1)}초`}</span>
        </header>
        <div class="camera-placeholder camera-frame">${
                cfg.mode === 'mock'
                    ? ''
                    // ROS 모드: 실제 프레임을 시도한다. 아직 아무도 안
                    // 보냈으면(Mock이거나 카메라 미연결) 404라 onerror로
                    // 조용히 숨고, 아래 크로스헤어/라벨만 남는다.
                    : `<img class="camera-live-feed" alt="${
                          escapeHtml(robot.robot_id)} 카메라" loading="lazy"
                          src="/api/camera/${
                          encodeURIComponent(robot.robot_id)}/frame?t=${
                          Date.now()}"
                          onerror="this.style.display='none'">`}
          ${cfg.mode === 'mock' ? '<div class="scan-line"></div>' : ''}
          <div class="camera-crosshair"></div>
          <span class="camera-label">${
                cfg.mode === 'mock'
                    ? 'MOCK VIDEO · 실제 영상 아님'
                    : 'ROS CAMERA DATA · 영상 스트림 연결 시도 중'}</span>
          <div class="detection-box ${targetClass}"><span><b>${
                hasTarget ? `${escapeHtml(targetName)} 감지`
                          : '대상 대기 중'}</b>${
                hasTarget ? `<em>${confidence}</em>` : ''}</span></div>
          <div class="camera-metadata camera-metadata-compact">
            <div><small>대상 거리</small><strong>${
                target.distance == null
                    ? '—'
                    : `${n(target.distance, 2)} m`}</strong></div>
            <div><small>입력 장치</small><strong>${
                escapeHtml(source)}</strong></div>
          </div>
        </div>
      </article>`;
          })
          .join('');

  cameraStack.querySelectorAll('[data-camera-robot]').forEach(card => {
    const selectCamera = () => {
      if (selectedRobot === card.dataset.cameraRobot)
        return;
      selectedRobot = card.dataset.cameraRobot;
      render(lastSnapshot);
    };
    card.addEventListener('click', selectCamera);
    card.addEventListener('keydown', event => {
      if (event.key === 'Enter' || event.key === ' ') {
        event.preventDefault();
        selectCamera();
      }
    });
  });
}

function renderEvents(events) {
  // 최근 사건은 패널에, 전체 목록은 대화상자에 렌더링한다.
  const markup = event => `<div class="event-item"><span class="event-time">${
      formatTime(event.timestamp)}</span><span class="event-robot">${
      escapeHtml(
          event.robot_id ||
          '시스템')}</span><span class="event-message"><i class="severity ${
      escapeHtml(event.severity.toLowerCase())}"></i><b class="sr-only">${
      event.severity === 'INFO' ? '정보' : '경고'}: </b>${
      escapeHtml(localizeObjectText(event.message))}</span></div>`;
  $('#event-list').innerHTML = events.slice(0, 4).map(markup).join('') ||
                               '<div class="empty">이벤트가 없습니다.</div>';
  $('#event-dialog-list').innerHTML =
      events.map(markup).join('') ||
      '<div class="empty">이벤트가 없습니다.</div>';
  $('#event-count').textContent = events.length;
}

/**
 * 정적 ROS 맵 메타데이터와 서버가 변환한 PNG 이미지를 한 번 불러온다.
 * 입력: 없음. `/api/map`과 응답의 image_url을 사용한다.
 * 출력: 없음. 맵 캐시와 상태 문구를 갱신하고 필요하면 지도를 다시 그린다.
 * 사용: 대시보드 초기화 마지막 단계에서 한 번 호출한다.
 */
async function loadMap() {
  const label = $('#map-grid-label');
  try {
    const metadata = await request('/api/map');
    if (!metadata.available)
      throw new Error(metadata.error || '맵을 사용할 수 없습니다.');

    const image = new Image();
    await new Promise((resolve, reject) => {
      image.addEventListener('load', resolve, {once : true});
      image.addEventListener('error',
                             () => reject(new Error('맵 이미지 로드 실패')),
                             {once : true});
      image.src = metadata.image_url;
    });
    mapMetadata = metadata;
    mapImage = image;
    canvasMapWrap().classList.add('has-static-map');
    label.textContent = '';
    label.classList.add('hidden');
    if (lastSnapshot)
      drawMap(lastSnapshot);
  } catch (error) {
    mapMetadata = null;
    mapImage = null;
    label.textContent = cfg.mode === 'mock' ? 'MOCK · 좌표 격자'
                                           : 'ROS · 맵 미연결';
    label.classList.remove('hidden');
    console.warn(error);
  }
}

const canvasMapWrap = () => $('#map-canvas').closest('.map-wrap');

/**
 * ROS map 좌표(m)를 현재 Canvas 화면 좌표(px)로 바꾸는 투영기를 만든다.
 * 입력: Canvas의 CSS 픽셀 너비와 높이다.
 * 출력: 맵 이미지 영역과 `toCanvas(x, y)` 변환 함수를 포함한 객체다.
 * 사용: `drawMap()`이 로봇·탐지·덫 좌표를 같은 기준으로 그릴 때 호출한다.
 */
function createMapProjection(width, height) {
  if (!mapMetadata || !mapImage) {
    return {
      imageRect : null,
      toCanvas : (x, y) => ({
        x : 35 + (x / 6) * (width - 70),
        y : height - 35 - (y / 5) * (height - 70)
      })
    };
  }

  const padding = 18;
  const imageScale = Math.min((width - padding * 2) / mapMetadata.width,
                              (height - padding * 2) / mapMetadata.height);
  const drawWidth = mapMetadata.width * imageScale;
  const drawHeight = mapMetadata.height * imageScale;
  const left = (width - drawWidth) / 2;
  const top = (height - drawHeight) / 2;
  const [originX, originY, originYaw] = mapMetadata.origin;
  const cosYaw = Math.cos(originYaw);
  const sinYaw = Math.sin(originYaw);

  return {
    imageRect : {x : left, y : top, width : drawWidth, height : drawHeight},
    toCanvas : (x, y) => {
      const dx = Number(x) - originX;
      const dy = Number(y) - originY;
      const localX = cosYaw * dx + sinYaw * dy;
      const localY = -sinYaw * dx + cosYaw * dy;
      // 백엔드가 PNG를 90도 반시계방향으로 돌려서 내려주므로(map_service.py
      // _load의 ROTATE_90 참고) localY가 가로(픽셀 x), localX가 세로(픽셀 y)
      // 방향이 되는 것까지는 시계방향과 같지만, 반시계는 그 거울상이라 두
      // 축 모두 폭/높이에서 빼야 한다 — 두 함수는 항상 같이 바꿔야 한다.
      const pixelX = mapMetadata.width - localY / mapMetadata.resolution;
      const pixelY = mapMetadata.height - localX / mapMetadata.resolution;
      return {x : left + pixelX * imageScale, y : top + pixelY * imageScale};
    }
  };
}

/**
 * 로봇 위치·방향·이동 궤적과 현재 세션의 탐지를 지도에 그린다.
 * 입력: 로봇 위치와 최근 탐지를 포함한 서버 스냅샷이다.
 * 출력: 없음. 화면 크기에 맞춘 canvas 픽셀을 다시 그린다.
 * 사용: `render()`와 지도 표시 옵션 변경 핸들러에서 호출한다.
 */
function drawMap(snapshot) {
  const canvas = $('#map-canvas');
  const rect = canvas.getBoundingClientRect();
  const scale = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, rect.width * scale);
  canvas.height = Math.max(1, rect.height * scale);
  const ctx = canvas.getContext('2d');
  ctx.scale(scale, scale);
  const w = rect.width, h = rect.height;
  const projection = createMapProjection(w, h);
  const toCanvas = projection.toCanvas;
  const mapFrame = mapMetadata?.frame_id || 'map';
  const occupiedLabels = [];
  mapMarkerHits = [];

  if (projection.imageRect) {
    const area = projection.imageRect;
    ctx.save();
    ctx.globalAlpha = .62;
    ctx.filter = 'brightness(.72) contrast(1.22)';
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(mapImage, area.x, area.y, area.width, area.height);
    ctx.restore();
    ctx.strokeStyle = 'rgba(152,152,157,.45)';
    ctx.lineWidth = 1;
    ctx.strokeRect(area.x, area.y, area.width, area.height);
  } else {
    ctx.strokeStyle = 'rgba(152,152,157,.22)';
    ctx.lineWidth = 2;
    const zoneA = toCanvas(.4, 4.5), zoneB = toCanvas(5.6, .5);
    ctx.strokeRect(zoneA.x, zoneA.y, zoneB.x - zoneA.x, zoneB.y - zoneA.y);
    ctx.setLineDash([ 10, 12 ]);
    ctx.lineWidth = 1;
    ctx.strokeStyle = 'rgba(41,151,255,.12)';
    [1.55, 3, 4.45].forEach(x => {
      const a = toCanvas(x, .6), b = toCanvas(x, 4.4);
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
    });
    ctx.setLineDash([]);
    ctx.fillStyle = 'rgba(152,152,157,.42)';
    ctx.font = '12px Arial';
    ctx.fillText('입구', toCanvas(.45, .4).x, toCanvas(.45, .4).y);
  }

  if (showTrails) {
    snapshot.robots.forEach((robot, index) => {
      if (projection.imageRect && robot.position_frame !== mapFrame)
        return;
      const trail = robotTrails.get(robot.robot_id) || [];
      if (trail.length < 2)
        return;
      ctx.strokeStyle =
          index === 0 ? 'rgba(41,151,255,.42)' : 'rgba(255,159,10,.42)';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      trail.forEach((point, i) => {
        const p = toCanvas(point.x, point.y);
        i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y);
      });
      ctx.stroke();
    });
  }

  const targetStates = new Set([
    'TRACKING', 'SEARCHING', 'APPROACHING', 'NAVIGATING', 'INSTALLING_TRAP',
    'TARGET_LOST'
  ]);
  const currentTargets =
      snapshot.robots.filter(robot => targetStates.has(robot.state))
          .map(robot => robot.target || {})
          .filter(target => target.map_x != null && target.map_y != null);
  const receivedDetections =
      clusterReceivedDetections(snapshot.detections || [])
          .filter(det => !currentTargets.some(
                      target => target.object_type === det.object_type &&
                                Math.hypot(target.map_x - det.map_x,
                                           target.map_y - det.map_y) <= .15))
          .slice(0, 5)
          .reverse();

  // 라벨을 그리는 순서와 관계없이 모든 실제 마커 위치를 먼저 보호한다.
  if (showDetectionMarkers)
    receivedDetections.forEach(det => {
      if (det.map_x != null && det.map_y != null)
        reserveMapMarkerArea(occupiedLabels,
                             toCanvas(det.map_x, det.map_y));
    });
  (snapshot.traps || []).forEach(trap => {
    if (trap.map_x != null && trap.map_y != null)
      reserveMapMarkerArea(occupiedLabels,
                           toCanvas(trap.map_x, trap.map_y));
  });
  snapshot.robots.forEach(robot => {
    if (projection.imageRect && robot.position_frame !== mapFrame)
      return;
    reserveMapMarkerArea(
        occupiedLabels, toCanvas(robot.position.x, robot.position.y),
        robot.robot_id === selectedRobot ? 24 : 13);
    const target = robot.target || {};
    if (targetStates.has(robot.state) && target.map_x != null &&
        target.map_y != null)
      reserveMapMarkerArea(occupiedLabels,
                           toCanvas(target.map_x, target.map_y), 11);
  });

  if (showDetectionMarkers)
    receivedDetections.forEach((det, index, array) => {
      if (det.map_x == null || det.map_y == null)
        return;
      const p = toCanvas(det.map_x, det.map_y);
      const newest = index === array.length - 1;
      drawDetectionMarker(ctx, det.object_type, p, newest ? 5 : 4,
                          .28 + index / Math.max(1, array.length) * .32);
      addMapMarkerHit(p, 'detection', det);
      // 수신 마커는 모두 클릭 가능하지만 최근 두 묶음만 라벨을 표시한다.
      if (index >= array.length - 2) {
        const count = det.count > 1 ? ` ×${det.count}` : '';
        const text = `${
            objectLabels[det.object_type] || det.object_type ||
            '대상'} 감지${count}`;
        drawMapLabel(ctx, text, p,
                     detectionColors[det.object_type] || '#ffb4ab',
                     occupiedLabels, {width : w, height : h});
      }
    });

  (snapshot.traps || []).forEach(trap => {
    if (trap.map_x == null || trap.map_y == null)
      return;
    const p = toCanvas(trap.map_x, trap.map_y);
    drawTrapMarker(ctx, trap, p);
    drawMapLabel(ctx, `덫 #${trap.id}`, p, '#b7f0c1', occupiedLabels,
                 {width : w, height : h});
    addMapMarkerHit(p, 'trap', trap);
  });

  snapshot.robots.forEach((robot, index) => {
    if (projection.imageRect && robot.position_frame !== mapFrame)
      return;
    const p = toCanvas(robot.position.x, robot.position.y);
    const color = index === 0 ? '#2997ff' : '#ff9f0a';
    const isSelected = robot.robot_id === selectedRobot;
    if (isSelected) {
      ctx.strokeStyle = '#f5f5f7';
      ctx.lineWidth = 2.5;
      ctx.globalAlpha = .8;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 23, 0, Math.PI * 2);
      ctx.stroke();
      ctx.globalAlpha = 1;
    }
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(-robot.position.yaw);
    ctx.scale(isSelected ? 1.3 : 1, isSelected ? 1.3 : 1);
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(13, 0);
    ctx.lineTo(-9, -8);
    ctx.lineTo(-5, 0);
    ctx.lineTo(-9, 8);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
    drawMapLabel(ctx, robot.robot_id, p, color, occupiedLabels,
                 {width : w, height : h});
    const t = robot.target || {};
    if (targetStates.has(robot.state) && t.map_x != null && t.map_y != null) {
      const tp = toCanvas(t.map_x, t.map_y);
      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth = 2.2;
      ctx.lineCap = 'round';
      ctx.globalAlpha = .9;
      ctx.setLineDash([ 1, 7 ]);
      ctx.beginPath();
      ctx.moveTo(p.x, p.y);
      ctx.lineTo(tp.x, tp.y);
      ctx.stroke();
      ctx.restore();
      drawDetectionMarker(ctx, t.object_type, tp, 5, .98);
      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.globalAlpha = .75;
      ctx.beginPath();
      ctx.arc(tp.x, tp.y, 10, 0, Math.PI * 2);
      ctx.stroke();
      ctx.restore();
      const targetLabel =
          `${robot.state === 'TARGET_LOST' ? '마지막' : '현재'} ${
              objectLabels[t.object_type] || t.object_type || '대상'}`;
      if (isSelected)
        drawMapLabel(ctx, targetLabel, tp,
                     detectionColors[t.object_type] || '#f5f5f7',
                     occupiedLabels, {width : w, height : h});
      const recentDetection =
          (snapshot.detections || [])
              .find(det => det.robot_id === robot.robot_id &&
                           det.object_type === t.object_type &&
                           det.map_x != null && det.map_y != null &&
                           Math.hypot(det.map_x - t.map_x,
                                      det.map_y - t.map_y) <= .2);
      addMapMarkerHit(tp, 'current', {
        ...t,
        robot_id : robot.robot_id,
        timestamp : recentDetection?.timestamp || robot.last_update
      },
                      19);
    }
  });

  if (projection.imageRect) {
    const unmapped =
        snapshot.robots.filter(robot => robot.position_frame !== mapFrame)
            .map(robot => robot.robot_id);
    if (unmapped.length) {
      ctx.fillStyle = '#ffd60a';
      ctx.font = '12px Arial';
      ctx.fillText(`TF 미연동: ${unmapped.join(', ')}`, 18, h - 12);
    }
  }
}

// ---------------------------------------------------------------------------
// 기록 조회 탭 — 실시간 폴링과 분리된 별도 화면이다. 탐지·임무·트랩 기록은
// 로봇 DB(db_node/MySQL)가 원본이고, 이동 경로만 관제 서버가 amcl_pose를 받아
// 직접 쌓는다(로봇 DB에 위치 이력 테이블이 없다). 사용자가 "기록 조회" 탭을
// 열 때만 불러온다(실시간 탭 폴링에는 관여하지 않는다).
// ---------------------------------------------------------------------------
const HISTORY_TRAIL_COLORS = [ '#2997ff', '#ff9f0a', '#bf5af2', '#32d74b' ];
let historyDetections = [];
let selectedHistoryDetectionId = null;

/**
 * 기록 자체가(필터와 무관하게) 하나도 없을 때 보여줄 안내 문구다.
 * 필터 때문에 잠깐 안 보이는 것과 구분해서, 이 경우에만 어디를 봐야 하는지
 * 알려준다 — 기록은 로봇 DB에 쌓이므로 db_node가 떠 있어야 남는다.
 */
const HISTORY_SEED_HINT_HTML =
    '아직 저장된 기록이 없습니다.<br>' +
    '로봇 DB(db_node)가 실행 중이어야 탐지 기록이 쌓입니다.';

/** db_node에 못 붙었을 때 각 영역에 공통으로 보여줄 문구다. */
const HISTORY_DB_ERROR_HTML =
    '로봇 DB에 연결하지 못했습니다.<br>' +
    'db_node가 실행 중인지, 관제 서버가 ROS 모드인지 확인해 주세요.';

/**
 * 로봇 id를 일관된 색으로 매핑한다(트레일 선·범례가 매번 같은 색을 쓰도록).
 * 입력: 로봇 id 문자열이다. 출력: HEX 색상 문자열이다.
 * 사용: 기록 지도의 이동 경로 선, 카드 표시 등에서 재사용한다.
 */
const historyRobotColor = (() => {
  const assigned = new Map();
  return robotId => {
    if (!assigned.has(robotId))
      assigned.set(robotId,
                   HISTORY_TRAIL_COLORS[assigned.size % HISTORY_TRAIL_COLORS.length]);
    return assigned.get(robotId);
  };
})();

/**
 * 기록 지도(정적 맵 + 누적 이동 경로 + 탐지 위치)를 그린다.
 * 입력: `/api/history/detections`, `/api/history/trail` 응답 배열이다.
 * 출력: 없음. `#history-map-canvas`를 다시 그린다.
 * 사용: 필터가 바뀌거나 새로고침할 때마다 `loadHistory()`가 호출한다.
 */
function drawHistoryMap(detections, trail, totalEmpty = false) {
  const canvas = $('#history-map-canvas');
  const rect = canvas.getBoundingClientRect();
  const scale = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, rect.width * scale);
  canvas.height = Math.max(1, rect.height * scale);
  const ctx = canvas.getContext('2d');
  ctx.scale(scale, scale);
  const w = rect.width, h = rect.height;
  const projection = createMapProjection(w, h);
  const toCanvas = projection.toCanvas;

  if (projection.imageRect) {
    const area = projection.imageRect;
    ctx.save();
    ctx.globalAlpha = .55;
    ctx.filter = 'brightness(.65) contrast(1.2)';
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(mapImage, area.x, area.y, area.width, area.height);
    ctx.restore();
    ctx.strokeStyle = 'rgba(152,152,157,.45)';
    ctx.lineWidth = 1;
    ctx.strokeRect(area.x, area.y, area.width, area.height);
  }

  const byRobot = new Map();
  trail.forEach(point => {
    if (point.map_x == null || point.map_y == null)
      return;
    if (!byRobot.has(point.robot_id))
      byRobot.set(point.robot_id, []);
    byRobot.get(point.robot_id).push(point);
  });
  byRobot.forEach((points, robotId) => {
    if (points.length < 2)
      return;
    ctx.strokeStyle = historyRobotColor(robotId);
    ctx.globalAlpha = .55;
    ctx.lineWidth = 2;
    ctx.beginPath();
    points.forEach((point, i) => {
      const p = toCanvas(point.map_x, point.map_y);
      i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y);
    });
    ctx.stroke();
    ctx.globalAlpha = 1;
  });

  detections.forEach(det => {
    if (det.map_x == null || det.map_y == null)
      return;
    drawDetectionMarker(ctx, det.object_type, toCanvas(det.map_x, det.map_y), 5, .85);
  });

  const hasData = detections.length > 0 || trail.length > 0;
  const mapEmpty = $('#history-map-empty');
  mapEmpty.classList.toggle('hidden', hasData);
  if (!hasData)
    mapEmpty.innerHTML = totalEmpty
        ? HISTORY_SEED_HINT_HTML
        : '이 필터 조건에 해당하는 경로/탐지가 없습니다.';
}

/**
 * 탐지 기록 카드 목록을 그린다(썸네일·종류·로봇·시각·좌표).
 * 입력: `/api/history/detections` 응답 배열이다.
 * 출력: 없음. `#history-detection-list`를 다시 채운다.
 * 사용: `loadHistory()`가 지도 렌더링과 함께 호출한다.
 */
function renderHistoryDetectionList(detections, totalEmpty = false) {
  const list = $('#history-detection-list');
  if (!detections.length) {
    list.innerHTML = `<div class="empty">${
        totalEmpty ? HISTORY_SEED_HINT_HTML : '이 필터 조건에 해당하는 탐지 기록이 없습니다.'
    }</div>`;
    return;
  }
  list.innerHTML = detections
                        .map(det => {
                          const label = objectLabels[det.object_type] ||
                              det.object_type || '대상';
                          const thumb = det.image_url
                              ? `<img src="${det.image_url}" alt="${label} 증거 이미지" loading="lazy">`
                              : '<div class="no-image">사진<br>없음</div>';
                          const coords = (det.map_x != null && det.map_y != null)
                              ? `(${n(det.map_x, 2)}, ${n(det.map_y, 2)})`
                              : '좌표 없음';
                          const confidence = det.confidence != null
                              ? ` · 신뢰도 ${Math.round(det.confidence * 100)}%`
                              : '';
                          return `<button type="button" class="history-card${
                              det.id === selectedHistoryDetectionId ? ' selected' : ''}"
                              data-detection-id="${det.id}">
                              ${thumb}
                              <div class="history-card-body">
                                <div class="history-card-title">${escapeHtml(label)}
                                  ${det.is_dummy ? '<span class="dummy-badge">DUMMY</span>' : ''}
                                </div>
                                <div class="history-card-meta">
                                  ${formatTime(det.timestamp)} · ${escapeHtml(det.robot_id || '—')}<br>
                                  ${coords}${confidence}
                                </div>
                              </div>
                            </button>`;
                        })
                        .join('');
}

/**
 * 필터된 시간 범위 안에서 탐지가 언제 일어났는지 점으로 찍어 보여준다.
 * 입력: 탐지 배열과 타임라인 시작 시각(초, epoch)이다 — 끝은 항상 지금이다.
 * 출력: 없음. `#history-timeline`을 다시 채운다.
 * 사용: `loadHistory()`가 목록·지도와 함께 호출한다.
 */
function renderHistoryTimeline(detections, windowStart, totalEmpty = false) {
  const container = $('#history-timeline');
  const empty = $('#history-timeline-empty');
  container.querySelectorAll('.timeline-tick').forEach(tick => tick.remove());
  empty.classList.toggle('hidden', detections.length > 0);
  if (!detections.length) {
    empty.innerHTML = totalEmpty
        ? HISTORY_SEED_HINT_HTML
        : '이 필터 조건에 해당하는 기록이 없습니다.';
    return;
  }

  // 시각을 못 읽은 기록은 어디에 찍을지 정할 수 없다 — 빼지 않으면 start가
  // NaN이 되어 타임라인 전체가 깨진다.
  const dated = detections.filter(det => det.timestamp != null);
  const now = Date.now() / 1000;
  const start = windowStart != null
      ? windowStart
      : Math.min(...dated.map(det => det.timestamp));
  const span = Math.max(1, now - start);

  dated.forEach(det => {
    const ratio = Math.min(1, Math.max(0, (det.timestamp - start) / span));
    const tick = document.createElement('button');
    tick.type = 'button';
    tick.className = `timeline-tick type-${(det.object_type || '').toLowerCase()}${
        det.id === selectedHistoryDetectionId ? ' selected' : ''}`;
    tick.style.left = `${ratio * 100}%`;
    tick.dataset.detectionId = det.id;
    const label = objectLabels[det.object_type] || det.object_type || '대상';
    tick.title = `${formatTime(det.timestamp)} · ${label} · ${det.robot_id || '—'}`;
    container.appendChild(tick);
  });
}

/**
 * 탐지 하나를 "선택 상태"로 만들어 큰 사진·상세 정보를 보여준다.
 * 입력: 선택할 탐지의 id다(목록/타임라인 어느 쪽에서 눌렀든 동일하게 처리).
 * 출력: 없음. 상세 패널, 목록 카드, 타임라인 점의 선택 표시를 모두 갱신한다.
 * 사용: 타임라인 점 클릭, 목록 카드 클릭 핸들러에서 호출한다.
 */
function selectHistoryDetection(detectionId) {
  const detection = historyDetections.find(det => det.id === detectionId);
  if (!detection)
    return;
  selectedHistoryDetectionId = detectionId;

  document.querySelectorAll('.history-card').forEach(card => {
    card.classList.toggle('selected', Number(card.dataset.detectionId) === detectionId);
  });
  document.querySelectorAll('.timeline-tick').forEach(tick => {
    tick.classList.toggle('selected', Number(tick.dataset.detectionId) === detectionId);
  });

  const label = objectLabels[detection.object_type] || detection.object_type || '대상';
  $('#history-detail-photo').innerHTML = detection.image_url
      ? `<img src="${detection.image_url}" alt="${escapeHtml(label)} 증거 이미지">`
      : '<div class="no-image large">이 기록에는<br>저장된 사진이 없습니다</div>';

  const coords = (detection.map_x != null && detection.map_y != null)
      ? `(${n(detection.map_x, 2)}, ${n(detection.map_y, 2)})`
      : '—';
  const confidence = detection.confidence != null
      ? `${Math.round(detection.confidence * 100)}%`
      : '—';
  $('#history-detail-meta').innerHTML = `
    <dt>시각</dt><dd>${formatTime(detection.timestamp)}${
        detection.is_dummy ? ' <span class="dummy-badge">DUMMY</span>' : ''}</dd>
    <dt>종류</dt><dd>${escapeHtml(label)}</dd>
    <dt>로봇</dt><dd>${escapeHtml(detection.robot_id || '—')}</dd>
    <dt>좌표</dt><dd>${coords}</dd>
    <dt>신뢰도</dt><dd>${confidence}</dd>
  `;
}

/**
 * 로봇 DB 조회 결과(행 배열)를 표 하나로 그린다.
 * 입력: 대상 요소 선택자, 컬럼 정의 [{key, label, format}], 행 배열이다.
 * 출력: 없음. 해당 영역을 표로 다시 채운다.
 * 사용: 임무 기록·트랩 현황 패널이 공유한다(모양이 같아 함수 하나로 충분).
 */
function renderHistoryTable(selector, columns, rows) {
  const container = $(selector);
  if (!container)
    return;
  if (!rows.length) {
    container.innerHTML = '<div class="empty">기록이 없습니다.</div>';
    return;
  }
  const head = columns.map(col => `<th>${escapeHtml(col.label)}</th>`).join('');
  const body =
      rows.map(row => `<tr>${
                   columns
                       .map(col => {
                         const value = col.format ? col.format(row[col.key], row)
                                                  : row[col.key];
                         return `<td>${value == null || value === ''
                                           ? '—'
                                           : escapeHtml(value)}</td>`;
                       })
                       .join('')}</tr>`)
          .join('');
  container.innerHTML =
      `<table class="history-table"><thead><tr>${head}</tr></thead><tbody>${
          body}</tbody></table>`;
}

/** DB의 DATETIME 문자열("2026-08-12 09:07:11")에서 시각 부분만 짧게 보여준다. */
const dbTime = value => value ? String(value).replace(/^\d{4}-/, '') : null;

const MISSION_COLUMNS = [
  {key : 'robot_id', label : '로봇'},
  {key : 'role', label : '역할'},
  {key : 'started_at', label : '시작', format : dbTime},
  {key : 'ended_at', label : '종료', format : dbTime},
  {
    key : 'duration_sec',
    label : '소요',
    format : value => value == null ? null : `${value}초`
  },
  {key : 'result', label : '결과'},
  {key : 'failure_reason', label : '비고'}
];

const TRAP_COLUMNS = [
  {key : 'trap_id', label : '트랩'},
  {
    key : 'opening_id',
    label : '구멍',
    format : (value, row) => value == null
        ? null
        : `#${value} (${n(row.opening_x, 2)}, ${n(row.opening_y, 2)})`
  },
  {key : 'status', label : '상태'},
  {key : 'installed_at', label : '설치', format : dbTime},
  {key : 'last_checked_at', label : '최근 점검', format : dbTime},
  {
    key : 'inspections',
    label : '점검',
    format : (value, row) => `${value ?? 0}회${
        row.moved_count ? ` · 이탈 ${row.moved_count}` : ''}`
  },
  {
    key : 'avg_drift_m',
    label : '평균 이탈',
    format : value => value == null ? null : `${n(value, 2)}m`
  }
];

/**
 * 임무 기록·트랩 현황 표를 로봇 DB에서 불러와 그린다.
 * 입력: 없음. `/api/db/missions`, `/api/db/traps`를 사용한다.
 * 출력: 완료 시 두 표가 갱신되는 Promise다(실패해도 예외를 던지지 않는다).
 * 사용: `loadHistory()`가 탐지 기록과 함께 호출한다.
 */
async function loadHistoryTables() {
  const panels = [
    {selector : '#history-missions', url : '/api/db/missions', columns : MISSION_COLUMNS},
    {selector : '#history-traps', url : '/api/db/traps', columns : TRAP_COLUMNS}
  ];
  await Promise.all(panels.map(async panel => {
    try {
      const data = await request(`${panel.url}?limit=100`);
      renderHistoryTable(panel.selector, panel.columns, data.rows || []);
    } catch (error) {
      // 표만 비고 나머지 기록 화면은 그대로 둔다 — DB 장애를 여기서 격리한다.
      $(panel.selector).innerHTML =
          `<div class="empty">${HISTORY_DB_ERROR_HTML}</div>`;
    }
  }));
}

/**
 * 필터 값(종류·로봇·기간)을 읽어 기록을 다시 조회하고 지도·목록을 갱신한다.
 * 입력: 없음. `#history-filter-*` select 값을 직접 읽는다.
 * 출력: 완료 시 화면이 갱신되는 Promise다.
 * 사용: 탭을 열 때, 새로고침 버튼, 필터 변경 시 호출한다.
 */
async function loadHistory() {
  const objectType = $('#history-filter-object-type').value;
  const robotId = $('#history-filter-robot').value;
  const windowSec = Number($('#history-filter-window').value) || 0;
  const since = windowSec > 0 ? (Date.now() / 1000 - windowSec) : null;

  const params = new URLSearchParams();
  if (objectType)
    params.set('object_type', objectType);
  if (robotId)
    params.set('robot_id', robotId);
  if (since != null)
    params.set('since', String(since));

  const trailParams = new URLSearchParams(params);
  loadHistoryTables();      // 표는 탐지 기록과 독립 — 서로 기다리지 않는다
  try {
    const [ summary, detections, trail ] = await Promise.all([
      request('/api/history/summary'),
      request(`/api/history/detections?${params.toString()}`),
      request(`/api/history/trail?${trailParams.toString()}`)
    ]);
    $('#history-summary').querySelector('strong').textContent =
        `탐지 ${summary.detections}${summary.capped ? '건+' : '건'} · 경로 ${
            summary.trail_points}개 기록됨`;

    const robotSelect = $('#history-filter-robot');
    if (robotSelect.options.length <= 1) {
      const robotIds = [...new Set(trail.map(point => point.robot_id))].sort();
      robotIds.forEach(id => {
        const option = document.createElement('option');
        option.value = id;
        option.textContent = id;
        robotSelect.appendChild(option);
      });
    }

    const totalEmpty = summary.detections === 0 && summary.trail_points === 0;
    historyDetections = detections;
    if (!detections.some(det => det.id === selectedHistoryDetectionId))
      selectedHistoryDetectionId = detections[0]?.id ?? null;

    renderHistoryDetectionList(detections, totalEmpty);
    renderHistoryTimeline(detections, since, totalEmpty);
    drawHistoryMap(detections, trail, totalEmpty);
    if (selectedHistoryDetectionId != null) {
      selectHistoryDetection(selectedHistoryDetectionId);
    } else {
      $('#history-detail-photo').innerHTML = `<div class="no-image large">${
          totalEmpty ? HISTORY_SEED_HINT_HTML : '이 필터 조건에 해당하는 기록이 없습니다.'
      }</div>`;
      $('#history-detail-meta').innerHTML = '';
    }
  } catch (error) {
    // db_node가 없으면 여기로 온다 — 빈 화면만 남기지 말고 이유를 화면에 적는다.
    historyDetections = [];
    $('#history-summary').querySelector('strong').textContent =
        '기록을 불러오지 못함';
    $('#history-detection-list').innerHTML =
        `<div class="empty">${HISTORY_DB_ERROR_HTML}</div>`;
    $('#history-detail-photo').innerHTML =
        `<div class="no-image large">${HISTORY_DB_ERROR_HTML}</div>`;
    $('#history-detail-meta').innerHTML = '';
    renderHistoryTimeline([], since, false);
    $('#history-timeline-empty').innerHTML = HISTORY_DB_ERROR_HTML;
    drawHistoryMap([], []);
    $('#history-map-empty').innerHTML = HISTORY_DB_ERROR_HTML;
    toast('기록을 불러오지 못했습니다: ' + error.message, 'error');
  }
}

/**
 * 서버 스냅샷을 한 번 조회해 대시보드 전체를 갱신한다.
 * 입력: 없음. `/api/snapshot`을 사용한다.
 * 출력: 완료 시 DOM이 갱신되는 Promise이며 오류는 연결 실패 상태로 표시한다.
 * 사용: 최초 로딩과 주기 타이머에서 호출한다.
 */
async function poll() {
  try {
    render(await request('/api/snapshot'));
  } catch (error) {
    $('#fleet-connection-dot').className = 'danger';
    $('#fleet-connection-state').textContent = '관제 서버 수신 실패';
    $('#operations-alert-message').textContent =
        '관제 서버 상태와 네트워크 연결을 확인해 주세요.';
    $('#operations-alert-banner').classList.remove('hidden');
    console.error(error);
  }
}

const mockPanelToggle = $('#mock-panel-toggle');
if (mockPanelToggle)
  mockPanelToggle.addEventListener(
      'click', () => $('#mock-controls').classList.toggle('hidden'));
document.querySelectorAll('[data-event]')
    .forEach(button => button.addEventListener('click', async () => {
      try {
        await request('/api/mock/events', {
          method : 'POST',
          headers : {'Content-Type' : 'application/json'},
          body : JSON.stringify(
              {event_type : button.dataset.event, robot_id : selectedRobot})
        });
        $('#mock-controls').classList.add('hidden');
        toast('테스트 이벤트를 생성했습니다.');
        await poll();
      } catch (error) {
        toast(error.message, 'error');
      }
    }));

// 시스템 시작/정지 — central의 대기 모드를 풀거나(START: 순찰 개시)
// 전 로봇 STOP + 대기 복귀(STOP)를 지시한다. ROS 모드에서만 동작한다.
document.querySelectorAll('.system-controls [data-action]')
    .forEach(button => button.addEventListener('click', async () => {
      button.disabled = true;
      try {
        await request(`/api/system/${button.dataset.action}`, {method : 'POST'});
        toast(button.dataset.action === 'start'
                  ? '시스템 시작 — 순찰을 개시합니다.'
                  : '시스템 정지 — 전 로봇을 정지시킵니다.');
      } catch (error) {
        toast(error.message, 'error');
      } finally {
        button.disabled = false;
      }
    }));

document.querySelectorAll('[data-mobile-target]')
    .forEach(button => button.addEventListener('click', () => {
      document.querySelectorAll('[data-mobile-target]')
          .forEach(item => item.classList.toggle('active', item === button));
      document.querySelectorAll('.dashboard-grid > .panel')
          .forEach(panel => panel.classList.toggle(
                       'mobile-active',
                       panel.classList.contains(button.dataset.mobileTarget)));
    }));

const mapCanvas = $('#map-canvas');
/**
 * 포인터 좌표와 클릭 반경을 비교해 가장 가까운 지도 마커를 찾는다.
 * 입력: map canvas에서 발생한 마우스 이벤트다.
 * 출력: 클릭 가능한 마커 객체이며 반경 안에 없으면 undefined다.
 * 사용: 마우스 커서 변경과 마커 상세 카드 열기에 공통으로 사용한다.
 */
const markerAtPointer = event => {
  const rect = mapCanvas.getBoundingClientRect();
  const point = {x : event.clientX - rect.left, y : event.clientY - rect.top};
  return mapMarkerHits
      .map(marker => ({
             marker,
             distance :
                 Math.hypot(point.x - marker.point.x, point.y - marker.point.y)
           }))
      .filter(item => item.distance <= item.marker.radius)
      .sort((a, b) => a.distance - b.distance)[0]
      ?.marker;
};
mapCanvas.addEventListener('mousemove', event => {
  mapCanvas.classList.toggle('marker-hover', Boolean(markerAtPointer(event)));
});
mapCanvas.addEventListener(
    'mouseleave', () => { mapCanvas.classList.remove('marker-hover'); });
mapCanvas.addEventListener('click', event => {
  const marker = markerAtPointer(event);
  if (marker)
    showMapMarkerDetail(marker);
  else
    $('#map-marker-detail').classList.add('hidden');
});
$('#close-map-marker-detail')
    .addEventListener('click',
                      () => $('#map-marker-detail').classList.add('hidden'));

$('#toggle-trails').addEventListener('click', event => {
  showTrails = !showTrails;
  event.currentTarget.classList.toggle('active', showTrails);
  if (lastSnapshot)
    drawMap(lastSnapshot);
});
$('#toggle-detections').addEventListener('click', event => {
  showDetectionMarkers = !showDetectionMarkers;
  event.currentTarget.classList.toggle('active', showDetectionMarkers);
  if (lastSnapshot)
    drawMap(lastSnapshot);
});
$('#toggle-map-legend').addEventListener('click', event => {
  const card = $('#map-legend-card');
  const visible = !card.classList.toggle('hidden');
  event.currentTarget.classList.toggle('active', visible);
  event.currentTarget.setAttribute('aria-expanded', String(visible));
});
$('#open-event-dialog')
    .addEventListener('click', () => $('#event-dialog').showModal());
document.querySelectorAll('[data-open-event-dialog]')
    .forEach(button => button.addEventListener(
                 'click', () => $('#event-dialog').showModal()));
$('#close-event-dialog')
    .addEventListener('click', () => $('#event-dialog').close());
$('#event-dialog').addEventListener('click', event => {
  if (event.target === event.currentTarget)
    event.currentTarget.close();
});

document.querySelectorAll('.view-tab').forEach(tab => tab.addEventListener('click', () => {
  const view = tab.dataset.view;
  document.querySelectorAll('.view-tab').forEach(item => {
    const active = item === tab;
    item.classList.toggle('active', active);
    if (active)
      item.setAttribute('aria-current', 'page');
    else
      item.removeAttribute('aria-current');
  });
  $('#view-live').classList.toggle('hidden', view !== 'live');
  $('#view-history').classList.toggle('hidden', view !== 'history');
  // 실시간 화면 전용 "한 화면 고정" CSS(html,body{overflow:hidden} 등,
  // dashboard.css의 min-height 미디어쿼리)를 기록 조회 화면이 물려받으면
  // 안 된다. body 클래스만으로는 부족했다 — 그 규칙이 <html>에도 똑같이
  // 걸려있어서, <html>은 인라인 스타일로 직접 덮어써야 확실히 이긴다.
  const isHistory = view === 'history';
  document.body.classList.toggle('history-mode', isHistory);
  document.documentElement.style.overflow = isHistory ? 'auto' : '';
  document.documentElement.style.height = isHistory ? 'auto' : '';
  document.body.style.overflow = isHistory ? 'auto' : '';
  document.body.style.height = isHistory ? 'auto' : '';
  if (isHistory)
    loadHistory();
}));
[ 'history-filter-object-type', 'history-filter-robot', 'history-filter-window' ]
    .forEach(id => $(`#${id}`).addEventListener('change', loadHistory));
$('#history-refresh').addEventListener('click', loadHistory);

// 목록 카드/타임라인 점은 loadHistory()가 매번 innerHTML을 새로 채우므로
// 위임(delegation)으로 한 번만 걸어 둔다.
$('#history-detection-list').addEventListener('click', event => {
  const card = event.target.closest('.history-card');
  if (card)
    selectHistoryDetection(Number(card.dataset.detectionId));
});
$('#history-timeline').addEventListener('click', event => {
  const tick = event.target.closest('.timeline-tick');
  if (tick)
    selectHistoryDetection(Number(tick.dataset.detectionId));
});

loadMap();
poll();
setInterval(poll, cfg.pollInterval);
