'use strict';

const $ = (selector) => document.querySelector(selector);
const portSelect = $('#port-select');
const pollingSelect = $('#polling-select');
const refreshButton = $('#refresh-button');
const connectButton = $('#connect-button');
const disconnectButton = $('#disconnect-button');
const estopButton = $('#estop-button');
const controlFieldset = $('#control-fieldset');
const stateElement = $('#connection-state');
const messageElement = $('#message');
const logOutput = $('#log-output');
const eventOutput = $('#event-output');
const axisSelect = $('#axis-select');

const ERROR_MESSAGES = {
  E001: '不明なコマンドです。', E002: '引数が不正です。', E003: '軸番号が範囲外です。',
  E004: '動作中は設定を変更できません。', E005: 'フォルト状態です。', E006: 'ソフトリミットに到達しました。',
  E007: 'ホーミングが完了していません。', E008: '別のモーションを実行中です。',
  E010: 'フォルト状態ではありません。', E011: '軸番号が重複しています。', E012: '基板IDを取得できません。'
};

let busy = false;
let connected = false;
let jogActive = false;
const trendDashboard = new window.TrendCharts.TrendDashboard({
  uPlotClass: window.uPlot,
  containers: {
    velocity: $('#velocity-chart'), position: $('#position-chart'),
    deviation: $('#deviation-chart'), power: $('#power-chart')
  },
  thresholds: {
    stall: () => $('#stall-threshold').value,
    current: () => $('#current-threshold').value
  }
});

async function refreshPorts() {
  setBusy(true);
  showMessage('COMポートを検索しています…');
  try {
    const selected = portSelect.value;
    const ports = await window.motorMonitor.listPorts();
    portSelect.replaceChildren();
    if (ports.length === 0) portSelect.add(new Option('利用可能なCOMポートがありません', ''));
    for (const port of ports) {
      const details = [port.manufacturer, port.serialNumber && `ID: ${port.serialNumber}`].filter(Boolean).join(' / ');
      portSelect.add(new Option(`${port.path}${details ? ` — ${details}` : ''}`, port.path));
    }
    if (ports.some((port) => port.path === selected)) portSelect.value = selected;
    showMessage(`${ports.length}件のポートが見つかりました。`);
  } catch (error) {
    showMessage(`ポート一覧の取得に失敗しました: ${error.message}`, true);
  } finally { setBusy(false); }
}

async function connect() {
  if (!portSelect.value) return;
  setBusy(true);
  showMessage(`${portSelect.value}に接続しています…`);
  try {
    await window.motorMonitor.setPollingInterval(Number(pollingSelect.value));
    renderStatus(await window.motorMonitor.connect(portSelect.value));
    connected = true;
    showMessage('疎通確認に成功し、監視を開始しました。');
  } catch (error) {
    connected = false;
    showMessage(`接続に失敗しました: ${error.message}`, true);
  } finally { setBusy(false); updateControls(); }
}

async function disconnect() {
  await stopJog();
  setBusy(true);
  try {
    await window.motorMonitor.disconnect();
    connected = false;
    showMessage('切断しました。');
  } catch (error) { showMessage(`切断処理に失敗しました: ${error.message}`, true); }
  finally { setBusy(false); updateControls(); }
}

async function sendCommand(command, successMessage = '') {
  try {
    const response = await window.motorMonitor.sendCommand(command);
    if (response.startsWith('ERR ')) {
      const code = response.split(/\s+/)[1];
      showMessage(`${command}: ${ERROR_MESSAGES[code] || response}`, true);
      return false;
    }
    showMessage(successMessage || `${command}: ${response}`);
    return true;
  } catch (error) {
    showMessage(`${command}の送信に失敗しました: ${error.message}`, true);
    return false;
  }
}

function selectedAxis() { return axisSelect.value; }

async function startJog(direction) {
  const speed = Number($('#jog-speed').value);
  if (!Number.isInteger(speed) || speed < 1 || speed > 100000 || jogActive) return;
  jogActive = true;
  const ok = await sendCommand(`VEL ${selectedAxis()} ${direction * speed}`, 'ジョグを開始しました。');
  if (!ok) jogActive = false;
}

async function stopJog() {
  if (!jogActive || !connected) return;
  jogActive = false;
  await sendCommand(`STOP ${selectedAxis()}`, 'ジョグを停止しました。');
}

function renderStatus(status) {
  trendDashboard.add(status);
  $('#microstep-value').textContent = status.microstep || '—';
  $('#voltage-value').textContent = formatNumber(status.telemetry?.voltageV, 3);
  $('#current-value').textContent = formatNumber(status.telemetry?.currentMa, 1);
  $('#updated-value').textContent = status.observedAt ? new Date(status.observedAt).toLocaleTimeString('ja-JP', { hour12: false }) : '—';

  for (const axis of status.axes || []) {
    const row = document.querySelector(`tr[data-axis="${axis.id}"]`);
    if (!row) continue;
    const encoder = status.telemetry?.encoders?.[axis.id];
    row.querySelector('.axis-state').replaceChildren(makeStateBadge(axis.state));
    row.querySelector('.axis-pos').textContent = formatNumber(axis.pos);
    row.querySelector('.axis-enc').textContent = formatNumber(encoder);
    row.querySelector('.axis-error').textContent = encoder == null ? '—' : formatNumber(encoder - axis.pos);
    row.querySelector('.axis-vel').textContent = formatNumber(axis.vel);
  }
  renderFault(status.telemetry?.fault, status.axes || []);
}

function renderFault(fault, axes) {
  const axisFault = axes.some((axis) => axis.state === 'FAULT');
  const active = axisFault || (fault && fault.reason !== 'NONE');
  $('#fault-banner').hidden = !active;
  if (!active) return;
  $('#fault-reason').textContent = fault?.reason || 'FAULT';
  $('#fault-detail').textContent = `軸マスク: ${fault?.axisMask || '不明'} — 原因を確認してから復帰してください。`;
}

function makeStateBadge(state) {
  const badge = document.createElement('span');
  badge.className = `axis-badge state-${String(state).toLowerCase()}`;
  badge.textContent = state;
  return badge;
}

function formatNumber(value, fractionDigits = 0) {
  return Number.isFinite(value) ? value.toLocaleString('ja-JP', { maximumFractionDigits: fractionDigits }) : '—';
}

function setBusy(value) { busy = value; updateControls(); }
function updateControls() {
  refreshButton.disabled = busy || connected;
  portSelect.disabled = busy || connected;
  pollingSelect.disabled = busy;
  connectButton.disabled = busy || connected || !portSelect.value;
  disconnectButton.disabled = busy || !connected;
  estopButton.disabled = !connected;
  controlFieldset.disabled = !connected;
}

function showMessage(text, isError = false) {
  messageElement.textContent = text;
  messageElement.classList.toggle('error', isError);
}

function renderState({ state, message = '' }) {
  const labels = { disconnected: '切断', connecting: '接続中', connected: '接続済み', error: 'エラー' };
  stateElement.textContent = labels[state] || state;
  stateElement.className = `state ${state}`;
  connected = state === 'connected';
  if (message) showMessage(message, state === 'error');
  updateControls();
}

function addLog(container, entry, direction = entry.direction) {
  const row = document.createElement('div');
  row.className = `log-row ${direction}`;
  const time = new Date(entry.timestamp).toLocaleTimeString('ja-JP', { hour12: false, fractionalSecondDigits: 3 });
  row.textContent = `${time}  ${direction.toUpperCase()}  ${entry.text}`;
  container.append(row);
  while (container.children.length > 500) container.firstElementChild.remove();
  container.scrollTop = container.scrollHeight;
}

function handleEvent(entry) {
  addLog(eventOutput, entry, 'evt');
  const match = /^EVT (OVERCURRENT|STALL_FAULT|FAULT)\b(.*)$/.exec(entry.text);
  if (match) {
    $('#fault-banner').hidden = false;
    $('#fault-reason').textContent = match[1];
    $('#fault-detail').textContent = `${match[2].trim() || '詳細なし'} — 原因を確認してから復帰してください。`;
  }
}

refreshButton.addEventListener('click', refreshPorts);
connectButton.addEventListener('click', connect);
disconnectButton.addEventListener('click', disconnect);
portSelect.addEventListener('change', updateControls);
pollingSelect.addEventListener('change', async () => {
  try { await window.motorMonitor.setPollingInterval(Number(pollingSelect.value)); }
  catch (error) { showMessage(error.message, true); }
});
$('#trend-window-select').addEventListener('change', (event) => trendDashboard.setWindow(Number(event.target.value)));
$('#trend-pause-button').addEventListener('click', (event) => {
  const paused = event.currentTarget.getAttribute('aria-pressed') !== 'true';
  event.currentTarget.setAttribute('aria-pressed', String(paused));
  event.currentTarget.textContent = paused ? '表示を再開' : '表示を一時停止';
  $('#trend-pause-state').hidden = !paused;
  trendDashboard.setPaused(paused);
});
$('#stall-threshold').addEventListener('change', () => trendDashboard.redrawThresholds());
$('#current-threshold').addEventListener('change', () => trendDashboard.redrawThresholds());
estopButton.addEventListener('click', () => sendCommand('ESTOP', '緊急停止を送信しました。'));
$('#clear-fault-button').addEventListener('click', () => sendCommand('CLEAR_FAULT'));
document.querySelectorAll('[data-command]').forEach((button) => button.addEventListener('click', () => sendCommand(`${button.dataset.command} ${selectedAxis()}`)));
$('#move-form').addEventListener('submit', (event) => { event.preventDefault(); sendCommand(`MOVE ${selectedAxis()} ${Math.trunc(Number($('#move-steps').value))}`); });
$('#moveto-form').addEventListener('submit', (event) => { event.preventDefault(); sendCommand(`MOVETO ${selectedAxis()} ${Math.trunc(Number($('#moveto-position').value))}`); });
$('#jog-negative').addEventListener('pointerdown', () => startJog(-1));
$('#jog-positive').addEventListener('pointerdown', () => startJog(1));
window.addEventListener('pointerup', stopJog);
window.addEventListener('pointercancel', stopJog);
window.addEventListener('blur', stopJog);
$('#clear-log-button').addEventListener('click', () => logOutput.replaceChildren());
$('#clear-event-button').addEventListener('click', () => eventOutput.replaceChildren());
window.motorMonitor.onState(renderState);
window.motorMonitor.onLog((entry) => addLog(logOutput, entry));
window.motorMonitor.onStatus(renderStatus);
window.motorMonitor.onEvent(handleEvent);
window.motorMonitor.onError((message) => showMessage(`通信エラー: ${message}`, true));

refreshPorts();
