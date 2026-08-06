const cfg = window.MONITOR_CONFIG || {pollInterval : 1000, mode : 'mock'};
let selectedRobot = null;
let lastSnapshot = null;
let showTrails = true;
let showDetectionHistory = true;
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
  INSTALLING_TRAP : '덫 설치 중',
  RETURNING : '복귀 중',
  COMPLETED : '임무 완료 · 대기 중',
  PAUSED : '일시정지',
  ERROR : '오류'
};
const missionLabels = {
  READY : '임무 대기',
  RUNNING : '진행 중',
  PAUSED : '일시정지',
  COMPLETED : '전체 임무 완료 · 대기 중'
};
const roleLabels = {
  SCOUT : '공동 탐색·역할 대기',
  RAT_TRACKER : '쥐 추적',
  SURVEY_TRAP : '쥐구멍 탐색·트랩 설치',
  UNASSIGNED : '역할 미지정'
};
// 좁은 로봇 선택 탭에는 역할의 핵심만 표시하고 상세 카드에는 전체명을 쓴다.
const shortRoleLabels = {
  SCOUT : '공동 탐색',
  RAT_TRACKER : '쥐 추적',
  SURVEY_TRAP : '쥐구멍 탐색',
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
  LIVE_RODENT : '#ff6b6b',
  ENTRY_POINT : '#b58cff',
  DROPPINGS : '#e2ad4f',
  rc_car : '#ff6b6b',
  rat_hole : '#b58cff',
  droppings : '#e2ad4f'
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
 * 사용: 현재 대상과 과거 탐지를 같은 시각 규칙으로 표시할 때 호출한다.
 */
function drawDetectionMarker(ctx, objectType, point, size = 5, alpha = 1) {
  const color = detectionColors[objectType] || '#ff6b6b';
  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.globalAlpha = alpha;
  ctx.fillStyle = color;
  ctx.strokeStyle = 'rgba(237,243,248,.85)';
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
 * 사용: 과거 탐지, 덫, 로봇, 현재 대상 라벨을 같은 규칙으로 표시한다.
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
  ctx.fillStyle = 'rgba(8,13,18,.88)';
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
  ctx.fillStyle = '#66d18f';
  ctx.strokeStyle = '#edf3f8';
  ctx.lineWidth = 1;
  ctx.fillRect(-6, -6, 12, 12);
  ctx.strokeRect(-6, -6, 12, 12);
  ctx.strokeStyle = '#173222';
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
  const titleSuffix = marker.kind === 'history' ? ' 감지' : '';
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
  detail.classList.remove('hidden');
}

/**
 * 같은 종류의 근접 탐지를 지도 마커 한 개와 건수로 묶는다.
 * 입력: 최신 탐지 배열과 같은 위치로 판단할 반경(m)이다.
 * 출력: 대표 탐지 데이터에 count가 추가된 클러스터 배열이다.
 * 사용: `drawMap()`이 같은 위치의 반복 탐지를 `×N`으로 표시할 때 호출한다.
 */
function clusterDetectionHistory(detections, radius = .15) {
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
 * 영향이 큰 로봇 명령을 설명한 뒤 사용자의 결정을 기다린다.
 * 입력: 제목, 질문, 적용 결과, 확인 버튼 문구와 위험 동작 여부다.
 * 출력: 확인이면 true, 취소·ESC이면 false인 Promise다.
 * 사용: 새 임무, 복귀, 임무 중단, 전체 이동 정지 직전에 호출한다.
 */
function requestCommandConfirmation(
    {title, message, impact, confirmLabel, danger = false}) {
  const dialog = $('#command-confirm-dialog');
  if (!dialog) {
    toast('새 제어 화면 적용을 위해 Mock 서버를 재시작해 주세요.', 'error');
    return Promise.resolve(false);
  }
  $('#command-confirm-title').textContent = title;
  $('#command-confirm-message').textContent = message;
  $('#command-confirm-impact').textContent = impact;
  const approve = $('#approve-command-confirm');
  approve.textContent = confirmLabel;
  approve.classList.toggle('danger', danger);
  dialog.returnValue = 'cancel';
  dialog.showModal();
  return new Promise(resolve => {
    dialog.addEventListener('close',
                            () => resolve(dialog.returnValue === 'confirm'),
                            {once : true});
  });
}

/**
 * 인증이 필요한 JSON API를 공통 방식으로 호출한다.
 * 입력: API URL과 선택적인 fetch 옵션이다.
 * 출력: 파싱된 JSON Promise이며, 실패 응답은 Error로 변환한다.
 * 사용: `await request('/api/snapshot')`처럼 호출한다.
 */
async function request(url, options = {}) {
  const response = await fetch(url, options);
  if (response.status === 401) {
    location.href = '/login';
    throw new Error('인증 필요');
  }
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
  renderFleetStopControl(snapshot.robots, snapshot.runtime);
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

  dot.className = allConnected ? 'online' : (online > 0 ? 'warning' : 'danger');
  state.textContent =
      allConnected ? `로봇 ${total}대 연결됨`
                   : (online > 0 ? `로봇 ${online}/${total}대 연결`
                                 : '연결된 로봇 없음');

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
 * 현재 상태에서 각 제어 버튼을 사용할 수 있는지와 안내 문구를 계산한다.
 * 입력: 선택 로봇 상태와 런타임 명령 지원 여부다.
 * 출력: 버튼별 disabled/reason 값과 카드 하단 안내 문구다.
 * 사용: `renderRobots()`가 버튼 상태, title, 도움말을 같은 기준으로 만든다.
 */
function getCommandControls(robot, commandsEnabled) {
  const movingStates = [
    'TRACKING', 'SEARCHING', 'APPROACHING', 'NAVIGATING', 'INSTALLING_TRAP'
  ];
  const unavailable =
      !commandsEnabled
          ? 'ROS 이동 명령 연결 전에는 사용할 수 없습니다.'
          : (robot.state === 'OFFLINE' ? '로봇 통신이 끊겨 사용할 수 없습니다.'
                                       : '');
  const startBlocked =
      [...movingStates, 'RETURNING', 'PAUSED', 'ERROR' ].includes(robot.state);
  const pauseBlocked =
      robot.state !== 'PAUSED' && !movingStates.includes(robot.state);
  const returnBlocked =
      [ 'OFFLINE', 'RETURNING', 'IDLE' ].includes(robot.state);
  const stopBlocked = [ 'OFFLINE', 'IDLE', 'COMPLETED' ].includes(robot.state);
  const reason = (blocked, detail) => unavailable || (blocked ? detail : '');

  let guidance = '대기 중 · 임무 시작을 선택할 수 있습니다.';
  if (!commandsEnabled)
    guidance = '상태 조회 전용 · ROS 이동 명령 연결 후 제어할 수 있습니다.';
  else if (robot.state === 'OFFLINE')
    guidance = '통신 끊김 · 연결이 복구될 때까지 명령을 보낼 수 없습니다.';
  else if (movingStates.includes(robot.state))
    guidance = '임무 수행 중 · 일시정지, 복귀 또는 임무 중단이 가능합니다.';
  else if (robot.state === 'PAUSED')
    guidance = '일시정지 상태 · 임무 재개 또는 복귀를 선택하세요.';
  else if (robot.state === 'RETURNING')
    guidance = '복귀 중 · 필요하면 임무 중단으로 이동을 정지할 수 있습니다.';
  else if (robot.state === 'COMPLETED')
    guidance = '임무 완료 · 새 임무 시작 시 진행률과 화면 경로가 초기화됩니다.';
  else if (robot.state === 'TARGET_LOST')
    guidance = '대상 유실 · 임무 재시작, 복귀 또는 중단을 선택하세요.';
  else if (robot.state === 'ERROR')
    guidance = '오류 상태 · 원인을 확인한 뒤 임무 중단을 선택하세요.';

  return {
    start : {
      disabled : Boolean(unavailable || startBlocked),
      reason : reason(startBlocked,
                      robot.state === 'PAUSED'
                          ? '일시정지 상태에서는 임무 재개를 사용하세요.'
                          : '현재 상태에서는 새 임무를 시작할 수 없습니다.')
    },
    pause : {
      disabled : Boolean(unavailable || pauseBlocked),
      reason : reason(pauseBlocked,
                      '진행 중인 이동 임무가 있을 때 사용할 수 있습니다.')
    },
    returnHome : {
      disabled : Boolean(unavailable || returnBlocked),
      reason : reason(returnBlocked,
                      '대기·복귀·오프라인 상태에서는 사용할 수 없습니다.')
    },
    stop : {
      disabled : Boolean(unavailable || stopBlocked),
      reason : reason(stopBlocked, '진행 중이거나 일시정지된 임무가 없습니다.')
    },
    guidance
  };
}

/**
 * 전체 이동 정지 버튼의 활성 여부와 비활성 이유를 계산해 반영한다.
 * 입력: 전체 로봇 배열과 `commands_enabled`를 포함한 런타임 상태다.
 * 출력: 없음. 버튼의 disabled와 title 속성을 갱신한다.
 * 사용: 스냅샷을 받을 때마다 `render()`가 호출한다.
 */
function renderFleetStopControl(robots, runtime = {}) {
  const button = $('#stop-all-robots');
  if (!button)
    return;
  const commandsEnabled = runtime.commands_enabled !== false;
  const targets = robots.filter(
      robot => robot.connection === 'ONLINE' &&
               !['IDLE', 'COMPLETED', 'OFFLINE'].includes(robot.state));
  button.disabled = !commandsEnabled || !targets.length;
  button.title =
      !commandsEnabled
          ? 'ROS 이동 명령 연결 전에는 사용할 수 없습니다.'
          : (!targets.length ? '정지할 이동 임무가 없습니다.'
                             : `${targets.length}대의 이동 임무를 정지합니다.`);
}

/**
 * 선택 로봇의 상태 카드와 역할별 제어 버튼을 렌더링한다.
 * 입력: 로봇 배열, 명령 지원 여부를 담은 런타임 상태, 전체 임무 상태다.
 * 출력: 없음. 로봇 선택, 전체 임무 요약, 상태·명령 DOM을 다시 만든다.
 * 사용: 최초 스냅샷과 로봇 선택·명령 처리 후 `render()`에서 호출한다.
 */
function renderRobots(robots, runtime = {}, mission = {}) {
  const commandsEnabled = runtime.commands_enabled !== false;
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
  // 전체 임무 상태는 안전 버튼과 분리해 로봇 선택 탭 바로 아래에서 읽는다.
  const waitingForRoleAssignment =
      mission.role_assignment_status === 'WAITING' &&
      [ 'READY', 'RUNNING' ].includes(mission.status);
  const missionText = waitingForRoleAssignment
      ? '역할 배정 전'
      : (missionLabels[mission.status] || mission.status || '임무 대기');
  const missionSummary = `
    <div class="fleet-mission-summary">
      <span>전체 임무</span>
      <strong id="mission-status" class="mission-summary-state state-${
          String(mission.status || 'READY').toLowerCase()}">${
          escapeHtml(missionText)}</strong>
    </div>`;
  const detail =
      [ robot ]
          .map(robot => {
            const startCommand =
                robot.role === 'RAT_TRACKER'
                    ? 'START_TRACKING'
                    : (robot.role === 'SURVEY_TRAP' ? 'START_SEARCH'
                                                    : 'START_SCOUTING');
            const controls = getCommandControls(robot, commandsEnabled);
            const pauseCommand = robot.state === 'PAUSED' ? 'RESUME' : 'PAUSE';
            const primaryCommand =
                robot.state === 'PAUSED'
                    ? 'RESUME'
                    : ([
                        'TRACKING', 'SEARCHING', 'APPROACHING', 'NAVIGATING',
                        'INSTALLING_TRAP'
                      ].includes(robot.state)
                           ? 'PAUSE'
                           : startCommand);
            const battery =
                robot.battery == null
                    ? 0
                    : Math.max(0, Math.min(100, Number(robot.battery)));
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
                escapeHtml(localizeObjectText(robot.current_task))}</div>
      <div class="robot-metrics robot-metrics-compact">
        <div class="metric battery-metric"><small>배터리</small><strong>${
                n(robot.battery, 0)}%</strong><span class="battery-mini ${
                battery < lowBatteryThreshold
                    ? 'low'
                    : ''}"><i style="width:${battery}%"></i></span></div>
        <div class="metric"><small>속도</small><strong>${
                n(robot.speed, 2)} m/s</strong></div>
        <div class="metric"><small>좌표</small><strong>${
                n(robot.position.x,
                  1)}, ${n(robot.position.y, 1)}</strong></div>
      </div>
      <div class="command-row">
        <button class="main-command ${
                primaryCommand === startCommand
                    ? 'primary-command'
                    : ''}" data-command="${startCommand}" title="${
                escapeHtml(controls.start.reason ||
                           '새 임무를 시작합니다.')}" ${
                controls.start.disabled ? 'disabled' : ''}>${
                robot.state === 'COMPLETED' ? '새 임무 시작'
                                            : '임무 시작'}</button>
        <button class="secondary-command ${
                primaryCommand === pauseCommand
                    ? 'primary-command'
                    : ''}" data-command="${pauseCommand}" title="${
                escapeHtml(controls.pause.reason ||
                           '현재 임무를 일시정지하거나 재개합니다.')}" ${
                controls.pause.disabled ? 'disabled' : ''}>${
                robot.state === 'PAUSED' ? '임무 재개' : '일시정지'}</button>
        <button class="secondary-command" data-command="RETURN_HOME" title="${
                escapeHtml(
                    controls.returnHome.reason ||
                    '현재 목표 이동을 취소하고 시작 위치로 복귀합니다.')}" ${
                controls.returnHome.disabled ? 'disabled' : ''}>복귀</button>
        <button class="danger-command" data-command="STOP" title="${
                escapeHtml(
                    controls.stop.reason ||
                    '현재 이동과 임무를 중단하고 대기 상태로 전환합니다.')}" ${
                controls.stop.disabled ? 'disabled' : ''}>임무 중단</button>
      </div>
      <p id="selected-command-guidance" class="command-guidance"><i></i>${
                escapeHtml(controls.guidance)}</p>
    </article>`;
          })
          .join('');
  $('#robot-list').innerHTML = selector + missionSummary + detail;
  document.querySelectorAll('[data-select-robot]')
      .forEach(button => button.addEventListener('click', () => {
        selectedRobot = button.dataset.selectRobot;
        render(lastSnapshot);
      }));
  document.querySelectorAll('[data-command]')
      .forEach(button => button.addEventListener('click', async event => {
        event.stopPropagation();
        const robotId = button.closest('.robot-card').dataset.robot;
        const command = button.dataset.command;
        const labels = {
          START_SCOUTING : '공동 탐색 시작',
          START_TRACKING : '추적 시작',
          START_SEARCH : '탐색 시작',
          PAUSE : '일시정지',
          RESUME : '임무 재개',
          RETURN_HOME : '복귀',
          STOP : '임무 중단'
        };
        const currentRobot =
            lastSnapshot?.robots.find(item => item.robot_id === robotId);
        // 상태를 크게 바꾸는 명령만 영향 범위를 설명하고 한 번 더 확인한다.
        let confirmation = null;
        if (command === 'RETURN_HOME') {
          confirmation = {
            title : `${robotId} 복귀`,
            message : `${robotId}를 시작 위치로 복귀시키겠습니까?`,
            impact :
                '현재 목표 이동을 취소하고 복귀 경로로 전환합니다. 저장된 탐지 기록은 유지됩니다.',
            confirmLabel : '복귀 실행'
          };
        } else if (command === 'STOP') {
          confirmation = {
            title : `${robotId} 임무 중단`,
            message : `${robotId}의 현재 임무를 중단하시겠습니까?`,
            impact :
                '현재 이동을 정지하고 로봇을 대기 상태로 전환합니다. 저장된 탐지 기록은 유지됩니다.',
            confirmLabel : '임무 중단',
            danger : true
          };
        } else if (command.startsWith('START_') &&
                   currentRobot?.state === 'COMPLETED') {
          confirmation = {
            title : `${robotId} 새 임무 시작`,
            message : '완료된 임무에 이어 새 임무를 시작하시겠습니까?',
            impact :
                '전체 임무 진행률이 0%부터 다시 시작되고 화면의 이동 경로가 초기화됩니다. 저장된 탐지 기록은 유지됩니다.',
            confirmLabel : '새 임무 시작'
          };
        }
        if (confirmation && !await requestCommandConfirmation(confirmation))
          return;
        const previousText = button.textContent;
        button.disabled = true;
        button.textContent = '전송 중…';
        try {
          await request('/api/commands', {
            method : 'POST',
            headers : {'Content-Type' : 'application/json'},
            body : JSON.stringify({robot_id : robotId, command})
          });
          if (command.startsWith('START_') &&
              currentRobot?.state === 'COMPLETED') {
            // 새 임무 진행률과 일치하도록 이전 임무의 화면 경로도 비운다.
            robotTrails.clear();
          }
          toast(`${robotId} · ${labels[command]} 명령이 반영되었습니다.`);
          await poll();
        } catch (error) {
          toast(error.message, 'error');
          button.disabled = false;
          button.textContent = previousText;
        }
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
        <div class="camera-placeholder camera-frame">
          <div class="scan-line"></div>
          <div class="camera-crosshair"></div>
          <span class="camera-label">${
                cfg.mode === 'mock'
                    ? 'MOCK VIDEO · 실제 영상 아님'
                    : 'ROS CAMERA DATA · 영상 스트림 미연동'}</span>
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
 * 입력: 없음. 인증된 `/api/map`과 응답의 image_url을 사용한다.
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
    // map_service.py의 world_to_pixel()과 동일한 3단계 변환이다:
    // origin만큼 평행이동 → origin_yaw만큼 반대 회전 → m을 픽셀로 바꾸고
    // y축을 뒤집는다(이미지는 위→아래, ROS map은 아래→위로 증가).
    // 서버가 만든 PNG와 같은 좌표계를 써야 로봇/탐지 마커가 지도와
    // 어긋나지 않는다.
    toCanvas : (x, y) => {
      const dx = Number(x) - originX;
      const dy = Number(y) - originY;
      const localX = cosYaw * dx + sinYaw * dy;
      const localY = -sinYaw * dx + cosYaw * dy;
      const pixelX = localX / mapMetadata.resolution;
      const pixelY = mapMetadata.height - localY / mapMetadata.resolution;
      // 마지막으로 원본 PGM 픽셀 → 화면에 그려진 배율(imageScale)로 맞추고
      // 지도 이미지가 canvas 중앙에 오도록 left/top 오프셋을 더한다.
      return {x : left + pixelX * imageScale, y : top + pixelY * imageScale};
    }
  };
}

/**
 * 로봇 위치·방향·이동 궤적과 현재/과거 탐지를 실시간 지도에 그린다.
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

  // 1) 배경: 실제 ROS 맵 PNG가 있으면 어둡게 깔고, 없으면(Mock) 좌표 격자를
  // 대신 그린다.
  if (projection.imageRect) {
    const area = projection.imageRect;
    ctx.save();
    ctx.globalAlpha = .62;
    ctx.filter = 'brightness(.72) contrast(1.22)';
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(mapImage, area.x, area.y, area.width, area.height);
    ctx.restore();
    ctx.strokeStyle = 'rgba(142,160,178,.45)';
    ctx.lineWidth = 1;
    ctx.strokeRect(area.x, area.y, area.width, area.height);
  } else {
    ctx.strokeStyle = 'rgba(142,160,178,.22)';
    ctx.lineWidth = 2;
    const zoneA = toCanvas(.4, 4.5), zoneB = toCanvas(5.6, .5);
    ctx.strokeRect(zoneA.x, zoneA.y, zoneB.x - zoneA.x, zoneB.y - zoneA.y);
    ctx.setLineDash([ 10, 12 ]);
    ctx.lineWidth = 1;
    ctx.strokeStyle = 'rgba(86,194,182,.12)';
    [1.55, 3, 4.45].forEach(x => {
      const a = toCanvas(x, .6), b = toCanvas(x, 4.4);
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
    });
    ctx.setLineDash([]);
    ctx.fillStyle = 'rgba(142,160,178,.42)';
    ctx.font = '12px Arial';
    ctx.fillText('입구', toCanvas(.45, .4).x, toCanvas(.45, .4).y);
  }

  // 2) 로봇별 이동 궤적(옵션): render()가 매 스냅샷마다 쌓아 둔
  // robotTrails를 선으로 잇는다.
  if (showTrails) {
    snapshot.robots.forEach((robot, index) => {
      if (projection.imageRect && robot.position_frame !== mapFrame)
        return;
      const trail = robotTrails.get(robot.robot_id) || [];
      if (trail.length < 2)
        return;
      ctx.strokeStyle =
          index === 0 ? 'rgba(86,194,182,.42)' : 'rgba(242,166,90,.42)';
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
  const history =
      clusterDetectionHistory(snapshot.detections || [])
          .filter(det => !currentTargets.some(
                      target => target.object_type === det.object_type &&
                                Math.hypot(target.map_x - det.map_x,
                                           target.map_y - det.map_y) <= .15))
          .slice(0, 5)
          .reverse();

  // 3) 라벨 겹침 방지 1단계: 실제로 그려질 모든 마커(과거 탐지·덫·로봇·현재
  // 대상)의 위치를 먼저 occupiedLabels에 예약해 둔다. 이렇게 해야 이후
  // drawMapLabel()이 "글자 라벨이 다른 마커를 가리지 않는" 위치를 고를 수
  // 있다. 라벨을 그리는 순서와 관계없이 모든 실제 마커 위치를 먼저 보호한다.
  if (showDetectionHistory)
    history.forEach(det => {
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

  // 4) 과거 탐지 마커를 그린다(위치 예약이 끝난 뒤). 최근 것일수록 더
  // 진하게(alpha 증가), 최근 2개 묶음만 텍스트 라벨을 붙인다.
  if (showDetectionHistory)
    history.forEach((det, index, array) => {
      if (det.map_x == null || det.map_y == null)
        return;
      const p = toCanvas(det.map_x, det.map_y);
      const newest = index === array.length - 1;
      drawDetectionMarker(ctx, det.object_type, p, newest ? 5 : 4,
                          .28 + index / Math.max(1, array.length) * .32);
      addMapMarkerHit(p, 'history', det);
      // 과거 마커는 모두 클릭 가능하지만 최근 두 묶음만 라벨을 표시한다.
      if (index >= array.length - 2) {
        const count = det.count > 1 ? ` ×${det.count}` : '';
        const text = `${
            objectLabels[det.object_type] || det.object_type ||
            '대상'} 감지${count}`;
        drawMapLabel(ctx, text, p,
                     detectionColors[det.object_type] || '#ffb0b0',
                     occupiedLabels, {width : w, height : h});
      }
    });

  // 5) 설치된 덫 마커.
  (snapshot.traps || []).forEach(trap => {
    if (trap.map_x == null || trap.map_y == null)
      return;
    const p = toCanvas(trap.map_x, trap.map_y);
    drawTrapMarker(ctx, trap, p);
    drawMapLabel(ctx, `덫 #${trap.id}`, p, '#a9e8c0', occupiedLabels,
                 {width : w, height : h});
    addMapMarkerHit(p, 'trap', trap);
  });

  // 6) 로봇 아이콘(방향이 있는 삼각형)과, 추적/탐색 중이면 현재 대상까지의
  // 점선 + 대상 마커를 그린다. 가장 마지막에 그려서 다른 마커 위에 보이게
  // 한다.
  snapshot.robots.forEach((robot, index) => {
    if (projection.imageRect && robot.position_frame !== mapFrame)
      return;
    const p = toCanvas(robot.position.x, robot.position.y);
    const color = index === 0 ? '#56c2b6' : '#f2a65a';
    const isSelected = robot.robot_id === selectedRobot;
    if (isSelected) {
      ctx.strokeStyle = '#edf3f8';
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
                     detectionColors[t.object_type] || '#edf3f8',
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
      ctx.fillStyle = '#f6c77d';
      ctx.font = '12px Arial';
      ctx.fillText(`TF 미연동: ${unmapped.join(', ')}`, 18, h - 12);
    }
  }
}

/**
 * 서버 스냅샷을 한 번 조회해 대시보드 전체를 갱신한다.
 * 입력: 없음. 인증된 `/api/snapshot`을 사용한다.
 * 출력: 완료 시 DOM이 갱신되는 Promise이며 오류는 연결 실패 상태로 표시한다.
 * 사용: 최초 로딩, 주기 타이머, 명령 처리 직후 호출한다.
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

/**
 * 현재 제어 가능한 모든 로봇에 STOP 명령을 각각 전송한다.
 * 입력: `전체 이동 정지` 버튼의 click 이벤트다.
 * 출력: 없음. 사용자 확인 후 결과 토스트와 최신 스냅샷을 반영한다.
 * 사용: 물리 E-Stop이 아닌 관제 명령 기반의 일괄 임무 중단에만 사용한다.
 */
async function stopAllRobotMissions(event) {
  const button = event.currentTarget;
  const targets =
      (lastSnapshot?.robots || [])
          .filter(robot =>
                      robot.connection === 'ONLINE' &&
                      !['IDLE', 'COMPLETED', 'OFFLINE'].includes(robot.state));
  if (!targets.length)
    return;
  const confirmed = await requestCommandConfirmation({
    title : '전체 이동 정지',
    message : `${
        targets.map(robot => robot.robot_id)
            .join(', ')}의 이동 임무를 정지하시겠습니까?`,
    impact :
        '관제 STOP 명령을 각 로봇에 전송하고 임무를 대기 상태로 전환합니다. 이 기능은 물리 긴급 정지 장치를 대체하지 않습니다.',
    confirmLabel : '전체 정지',
    danger : true
  });
  if (!confirmed)
    return;

  const previousText = button.textContent;
  button.disabled = true;
  button.textContent = '정지 명령 중…';
  // 한 로봇의 실패가 다른 로봇의 STOP 전송을 취소하지 않도록 모두 기다린다.
  const results = await Promise.allSettled(targets.map(
      robot => request('/api/commands', {
        method : 'POST',
        headers : {'Content-Type' : 'application/json'},
        body : JSON.stringify({robot_id : robot.robot_id, command : 'STOP'})
      })));
  const failed = results.filter(result => result.status === 'rejected');
  if (failed.length)
    toast(`${targets.length - failed.length}/${
              targets.length}대만 정지 명령이 반영되었습니다.`,
          'error');
  else
    toast(`${targets.length}대의 이동 정지 명령이 반영되었습니다.`);
  button.textContent = previousText;
  await poll();
}

const stopAllRobotsButton = $('#stop-all-robots');
if (stopAllRobotsButton)
  stopAllRobotsButton.addEventListener('click', stopAllRobotMissions);

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
  showDetectionHistory = !showDetectionHistory;
  event.currentTarget.classList.toggle('active', showDetectionHistory);
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

loadMap();
poll();
setInterval(poll, cfg.pollInterval);
