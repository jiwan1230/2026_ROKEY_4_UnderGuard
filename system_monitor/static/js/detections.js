const tbody = document.querySelector('#detection-tbody');
const form = document.querySelector('#filter-form');
const pageSummary = document.querySelector('#page-summary');
const previousPage = document.querySelector('#prev-page');
const nextPage = document.querySelector('#next-page');
const openDataReset = document.querySelector('#open-data-reset');
const dataResetDialog = document.querySelector('#data-reset-dialog');
const confirmDataReset = document.querySelector('#confirm-data-reset');

const pageSize = 10;
let currentPage = 1;
let allRows = [];

const statusLabels = {
  UNREVIEWED : '미검토',
  REVIEWED : '검토 완료',
  ACTIONED : '조치 완료',
  FALSE_POSITIVE : '오탐',
};
const objectLabels = {
  LIVE_RODENT : '쥐',
  ENTRY_POINT : '쥐구멍',
  DROPPINGS : '배설물',
  rc_car : '쥐',
  rat_hole : '쥐구멍',
  droppings : '배설물',
};

function escapeHtml(value) {
  // DB 문자열이 HTML로 실행되지 않도록 표에 넣기 전에 특수 문자를
  // 이스케이프한다.
  const entities = {
    '&' : '&amp;',
    '<' : '&lt;',
    '>' : '&gt;',
    '\'' : '&#39;',
    '"' : '&quot;',
  };
  return String(value ?? '')
      .replace(/[&<>'"]/g, (character) => entities[character]);
}

function toast(message, type = 'success') {
  const item = document.createElement('div');
  item.className = `toast ${type}`;
  item.textContent = message;
  document.querySelector('#toast-region').append(item);
  setTimeout(() => item.remove(), 3200);
}

function formatDbTime(value) {
  // SQLite의 UTC 문자열을 평가 현장의 Asia/Seoul 시간으로 표시한다.
  if (!value)
    return '—';
  const date = new Date(`${value.replace(' ', 'T')}Z`);
  return new Intl
      .DateTimeFormat('ko-KR', {
        timeZone : 'Asia/Seoul',
        year : 'numeric',
        month : '2-digit',
        day : '2-digit',
        hour : '2-digit',
        minute : '2-digit',
        second : '2-digit',
        hour12 : false,
      })
      .format(date);
}

function reviewOptions(selectedStatus) {
  return Object.entries(statusLabels)
      .map(([ value, label ]) => {
        const selected = value === selectedStatus ? 'selected' : '';
        return `<option value="${value}" ${selected}>${label}</option>`;
      })
      .join('');
}

/** 탐지 API 항목 하나를 검토 입력란이 있는 안전한 HTML 행으로 변환한다. */
function renderDetectionRow(row) {
  const confidence =
      row.confidence == null ? '—' : `${Math.round(row.confidence * 100)}%`;
  const distance =
      row.distance == null ? '—' : `${Number(row.distance).toFixed(2)}m`;
  const coordinates =
      row.map_x == null
          ? '—'
          : `${Number(row.map_x).toFixed(2)}, ${Number(row.map_y).toFixed(2)}`;

  return `
    <tr data-id="${row.id}">
      <td>${row.id}</td>
      <td title="DB UTC: ${escapeHtml(row.detected_at)}">
        ${escapeHtml(formatDbTime(row.detected_at))}
      </td>
      <td>${escapeHtml(row.robot_id)}</td>
      <td><b>${
      escapeHtml(objectLabels[row.object_type] || row.object_type)}</b></td>
      <td>${confidence}</td>
      <td>${distance}</td>
      <td>${coordinates}</td>
      <td>${escapeHtml(row.source || '—')}</td>
      <td>
        <select class="review-select" aria-label="검토 상태">
          ${reviewOptions(row.review_status)}
        </select>
      </td>
      <td>
        <input
          class="memo-input"
          value="${escapeHtml(row.memo || '')}"
          maxlength="200"
          placeholder="검토 메모"
        >
      </td>
      <td><button class="table-button save-review">저장</button></td>
    </tr>
  `;
}

function renderRows() {
  // 서버 결과에서 현재 페이지에 해당하는 10건만 DOM에 렌더링한다.
  const pages = Math.max(1, Math.ceil(allRows.length / pageSize));
  currentPage = Math.min(currentPage, pages);
  const start = (currentPage - 1) * pageSize;
  const items = allRows.slice(start, start + pageSize);

  tbody.innerHTML =
      items.length
          ? items.map(renderDetectionRow).join('')
          : '<tr><td colspan="11" class="empty">검색 결과가 없습니다.</td></tr>';
  pageSummary.textContent =
      `총 ${allRows.length}건 · ${currentPage}/${pages} 페이지`;
  previousPage.disabled = currentPage <= 1;
  nextPage.disabled = currentPage >= pages;
}

/**
 * 검색 폼 조건으로 탐지 목록을 다시 조회한다.
 * 입력: 현재 `#filter-form` 값이다.
 * 출력: 없음. `allRows`와 페이지 UI를 갱신한다.
 * 사용: 페이지 최초 진입 및 검색 폼 제출 시 호출한다.
 */
async function loadRows() {
  try {
    const query = new URLSearchParams(new FormData(form));
    const response = await fetch(`/api/detections?${query}`);
    if (response.status === 401) {
      location.href = '/login';
      return;
    }

    const data = await response.json();
    if (!response.ok)
      throw new Error(data.error || '조회 실패');
    allRows = data.items;
    currentPage = 1;
    renderRows();
  } catch (error) {
    toast(error.message, 'error');
  }
}

/**
 * 선택한 표 행의 검토 상태와 메모를 서버에 저장한다.
 * 입력: `.save-review` 버튼 DOM 요소다.
 * 출력: 저장 완료까지 기다릴 수 있는 Promise다.
 * 사용: 탐지 행의 저장 버튼 클릭 핸들러에서 호출한다.
 */
async function saveReview(button) {
  const row = button.closest('tr');
  const payload = {
    review_status : row.querySelector('.review-select').value,
    memo : row.querySelector('.memo-input').value,
  };

  button.disabled = true;
  button.textContent = '저장 중';
  try {
    const response = await fetch(`/api/detections/${row.dataset.id}`, {
      method : 'PATCH',
      headers : {'Content-Type' : 'application/json'},
      body : JSON.stringify(payload),
    });
    const data = await response.json();
    if (!response.ok)
      throw new Error(data.error || '저장 실패');

    toast(`탐지 #${row.dataset.id} 검토 결과를 저장했습니다.`);
    button.textContent = '저장됨';
    row.classList.remove('dirty');
    row.classList.add('saved');
    setTimeout(() => row.classList.remove('saved'), 1500);
  } catch (error) {
    toast(error.message, 'error');
    button.textContent = '다시 시도';
  } finally {
    button.disabled = false;
  }
}

/**
 * 관리자 확인 후 현재 모드의 DB 운영 데이터와 화면 이력을 초기화한다.
 * 입력: 없음. 출력: 서버 초기화 완료까지 기다릴 수 있는 Promise다.
 * 사용: Mock/ROS 탐지 DB 화면의 관리자 `초기화 실행` 버튼에서 호출한다.
 */
async function resetOperationalData() {
  confirmDataReset.disabled = true;
  confirmDataReset.textContent = '초기화 중';
  try {
    const response = await fetch('/api/admin/reset-operational-data', {
      method : 'POST',
      headers : {'Content-Type' : 'application/json'},
      body : JSON.stringify({confirmation : 'RESET_OPERATIONAL_DATA'}),
    });
    if (response.status === 401) {
      location.href = '/login';
      return;
    }
    const data = await response.json();
    if (!response.ok)
      throw new Error(data.error || '초기화 실패');

    dataResetDialog.close();
    await loadRows();
    const total =
        Object.values(data.deleted).reduce((sum, count) => sum + count, 0);
    const resultMessage =
        data.mode === 'mock'
            ? `운영 데이터 ${total}건을 삭제했습니다. 로봇은 임무 대기 상태입니다.`
            : `ROS 운영 데이터 ${total}건을 삭제했습니다. 새 토픽 수집은 계속됩니다.`;
    toast(resultMessage);
  } catch (error) {
    toast(error.message, 'error');
  } finally {
    confirmDataReset.disabled = false;
    confirmDataReset.textContent = '초기화 실행';
  }
}

tbody.addEventListener('click', (event) => {
  const button = event.target.closest('.save-review');
  if (button)
    saveReview(button);
});

tbody.addEventListener(
    'input',
    (event) => { event.target.closest('tr')?.classList.add('dirty'); });

tbody.addEventListener(
    'change',
    (event) => { event.target.closest('tr')?.classList.add('dirty'); });

previousPage.addEventListener('click', () => {
  if (currentPage <= 1)
    return;
  currentPage -= 1;
  renderRows();
  document.querySelector('.table-wrap').scrollTop = 0;
});

nextPage.addEventListener('click', () => {
  if (currentPage * pageSize >= allRows.length)
    return;
  currentPage += 1;
  renderRows();
  document.querySelector('.table-wrap').scrollTop = 0;
});

form.addEventListener('submit', (event) => {
  event.preventDefault();
  loadRows();
});

openDataReset?.addEventListener('click', () => dataResetDialog.showModal());
confirmDataReset?.addEventListener('click', resetOperationalData);

loadRows();
