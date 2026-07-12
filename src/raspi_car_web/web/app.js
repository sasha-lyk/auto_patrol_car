// Raspi Car web console: sends drive commands and polls live telemetry.
'use strict';

const API = '';                 // same origin
const HOLD_INTERVAL = 150;      // ms between repeated cmds while held
const POLL_INTERVAL = 200;      // ms status poll
const MAX_WHEEL = 0.9;          // m/s, for bar scaling (matches max_wheel_speed)

let speed = 2;
let holdTimer = null;
let activeCmd = null;

async function postCmd(c) {
  try {
    await fetch(API + '/api/cmd', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({c: c}),
    });
  } catch (e) { /* offline; status poll shows it */ }
}

function startHold(cmd) {
  if (activeCmd === cmd) return;
  activeCmd = cmd;
  postCmd(cmd);
  clearInterval(holdTimer);
  holdTimer = setInterval(() => postCmd(cmd), HOLD_INTERVAL);
}
function stopHold() {
  clearInterval(holdTimer);
  holdTimer = null;
  if (activeCmd && activeCmd !== 'S' && activeCmd !== 'E') postCmd('S');
  activeCmd = null;
}

// --- button wiring (mouse + touch) ---
document.querySelectorAll('.pad button[data-cmd]').forEach(btn => {
  const cmd = btn.dataset.cmd;
  if (cmd === 'S' || cmd === 'E') {
    btn.addEventListener('click', () => postCmd(cmd));
    return;
  }
  btn.addEventListener('mousedown', () => startHold(cmd));
  btn.addEventListener('touchstart', e => { e.preventDefault(); startHold(cmd); }, {passive:false});
  ['mouseup','mouseleave','touchend','touchcancel'].forEach(ev =>
    btn.addEventListener(ev, stopHold));
});

document.querySelectorAll('.speeds button').forEach(btn => {
  btn.addEventListener('click', () => {
    speed = parseInt(btn.dataset.spd, 10);
    postCmd(String(speed));
    document.querySelectorAll('.speeds button').forEach(b => b.classList.remove('sel'));
    btn.classList.add('sel');
  });
});

// --- keyboard ---
const KEYMAP = {
  'w':'F','arrowup':'F','s':'B','arrowdown':'B',
  'a':'L','arrowleft':'L','d':'R','arrowright':'R',' ':'S',
};
document.addEventListener('keydown', e => {
  if (e.repeat) return;
  const k = e.key.toLowerCase();
  if (k in KEYMAP) {
    e.preventDefault();
    const c = KEYMAP[k];
    if (c === 'S') postCmd('S'); else startHold(c);
  }
});
document.addEventListener('keyup', e => {
  const k = e.key.toLowerCase();
  if (k in KEYMAP && KEYMAP[k] !== 'S') stopHold();
});

// --- telemetry poll ---
const $ = id => document.getElementById(id);
const R2D = 180 / Math.PI;

function fmtAge(ms) {
  if (ms === null || ms === undefined) return '无';
  return ms + ' ms';
}

async function poll() {
  let s;
  try {
    const r = await fetch(API + '/api/status');
    s = await r.json();
  } catch (e) {
    $('dot').classList.remove('on');
    $('link').textContent = '离线';
    return;
  }
  $('dot').classList.add('on');
  $('link').textContent = '在线 · ' + (s.update_time || '');
  $('ns').textContent = s.namespace || 'car01';

  $('pos').textContent = `${s.x.toFixed(2)} / ${s.y.toFixed(2)}`;
  $('yaw').textContent = (s.yaw * R2D).toFixed(1);
  $('v').textContent = s.v.toFixed(2);
  $('w').textContent = s.w.toFixed(2);
  $('rp').textContent = `${(s.roll*R2D).toFixed(1)} / ${(s.pitch*R2D).toFixed(1)}`;
  $('gz').textContent = s.gz.toFixed(2);

  const odomOk = s.odom_age_ms !== null && s.odom_age_ms < 1000;
  const imuOk = s.imu_age_ms !== null && s.imu_age_ms < 1000;
  $('age').innerHTML =
    `odom ${fmtAge(s.odom_age_ms)} · imu ${fmtAge(s.imu_age_ms)}`;

  const b = s.base || {};
  if (b.type === 'base_status') {
    const tl = b.target_left ?? 0, ml = b.meas_left ?? 0;
    const tr = b.target_right ?? 0, mr = b.meas_right ?? 0;
    $('lw').textContent = `${tl.toFixed(2)} / ${ml.toFixed(2)}`;
    $('rw').textContent = `${tr.toFixed(2)} / ${mr.toFixed(2)}`;
    $('lwbar').style.width = Math.min(100, Math.abs(ml)/MAX_WHEEL*100) + '%';
    $('rwbar').style.width = Math.min(100, Math.abs(mr)/MAX_WHEEL*100) + '%';
    $('pwm').textContent = `${(b.left_pwm ?? 0).toFixed(2)} / ${(b.right_pwm ?? 0).toFixed(2)}`;
    const cl = b.closed_loop;
    $('mode').textContent = cl ? '闭环 PID (编码器反馈)' : '开环 (无反馈)';
    const badge = $('loopbadge');
    if (b.dry_run) { badge.textContent = 'DRY-RUN (无GPIO输出)'; badge.className='badge warn'; }
    else if (b.timed_out) { badge.textContent = 'cmd_vel 超时 · 已停车'; badge.className='badge err'; }
    else if (cl) { badge.textContent = '闭环运行中 · 编码器反馈正常'; badge.className='badge ok'; }
    else { badge.textContent = '开环回退中 · 编码器数据缺失'; badge.className='badge warn'; }
  }
}
setInterval(poll, POLL_INTERVAL);
poll();
