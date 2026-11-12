// MiniX cluster HMI.
// Buttons repeat while held so the Arduino watchdog acts as a real
// safety net: let go, lose signal, or close the tab and it stops.

const REPEAT_MS = 150;

// Releasing a steering button should straighten the wheels, not kill
// the throttle. Releasing a throttle button should actually stop.
const STEER_ACTS = new Set(['left', 'right', 'center']);

function releaseAction(act) {
  return STEER_ACTS.has(act) ? 'center' : 'stop';
}

function post(action, extra) {
  const body = Object.assign({ action: action }, extra || {});
  return fetch('/cmd', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body)
  }).catch(() => {});
}

// ---- hold-to-drive ----
document.querySelectorAll('.hold').forEach(btn => {
  let timer = null;

  const start = (e) => {
    e.preventDefault();
    if (timer) return;
    btn.classList.add('active');
    post(btn.dataset.act);
    timer = setInterval(() => post(btn.dataset.act), REPEAT_MS);
  };

  const end = () => {
    if (!timer) return;
    clearInterval(timer);
    timer = null;
    btn.classList.remove('active');
    post(releaseAction(btn.dataset.act));
  };

  btn.addEventListener('pointerdown', start);
  btn.addEventListener('pointerup', end);
  btn.addEventListener('pointercancel', end);
  btn.addEventListener('pointerleave', end);
});

document.getElementById('stop')
  .addEventListener('click', () => post('stop'));
document.getElementById('ebrake')
  .addEventListener('click', () => post('ebrake'));

// ---- LKA on/off toggle ----
const lkaBtn = document.getElementById('lka-toggle');
let lkaEnabled = true;  // starts ON, matches the server default
lkaBtn.addEventListener('click', () => {
  lkaEnabled = !lkaEnabled;
  post(lkaEnabled ? 'lka_on' : 'lka_off');
  lkaBtn.textContent = 'LKA: ' + (lkaEnabled ? 'On' : 'Off');
  lkaBtn.classList.toggle('active', lkaEnabled);
});

// ---- joystick: combined smooth steer + throttle ----
(function () {
  const pad = document.getElementById('joystick');
  const knob = document.getElementById('joyknob');
  let dragging = false;
  let steer = 0, throttle = 0;   // -100..100 each
  let repeatTimer = null;
  let originX = 0, originY = 0;  // press point becomes the zero-reference

  function setKnob(nx, ny) {
    const r = pad.clientWidth / 2;
    knob.style.transform = `translate(${nx * r * 0.7}px, ${ny * r * 0.7}px)`;
  }

  function fromDrag(e) {
    const rect = pad.getBoundingClientRect();
    const r = rect.width / 2;
    let nx = (e.clientX - originX) / r;
    let ny = (e.clientY - originY) / r;
    const mag = Math.hypot(nx, ny);
    if (mag > 1) { nx /= mag; ny /= mag; }
    steer = Math.round(nx * 100);
    throttle = Math.round(-ny * 100); // up = forward = positive
    setKnob(nx, ny);
  }

  function sendJoystick() {
    post('joystick', { steer, throttle });
  }

  function start(e) {
    e.preventDefault();
    dragging = true;
    pad.setPointerCapture(e.pointerId);
    // Always start at zero regardless of where inside the circle you
    // pressed — prevents an edge-tap from jumping straight to near-full
    // throttle before any drag has happened.
    originX = e.clientX;
    originY = e.clientY;
    steer = 0;
    throttle = 0;
    setKnob(0, 0);
    sendJoystick();
    if (!repeatTimer) repeatTimer = setInterval(sendJoystick, REPEAT_MS);
  }

  function move(e) {
    if (!dragging) return;
    fromDrag(e);
  }

  function end() {
    if (!dragging) return;
    dragging = false;
    steer = 0;
    throttle = 0;
    setKnob(0, 0);
    sendJoystick();
    clearInterval(repeatTimer);
    repeatTimer = null;
  }

  pad.addEventListener('pointerdown', start);
  pad.addEventListener('pointermove', move);
  pad.addEventListener('pointerup', end);
  pad.addEventListener('pointercancel', end);
  pad.addEventListener('pointerleave', end);
})();

// ---- speed ----
const spd = document.getElementById('spd');
const spdOut = document.getElementById('spd-out');
spd.addEventListener('input', () => {
  spdOut.textContent = spd.value;
  post('speed', { speed: parseInt(spd.value, 10) });
});

// ---- keyboard, for laptop driving ----
const KEYS = { w: 'forward', s: 'reverse', a: 'left', d: 'right' , c: 'center' };
const held = {};
window.addEventListener('keydown', e => {
  const act = KEYS[e.key.toLowerCase()];
  if (!act || held[act]) return;
  held[act] = setInterval(() => post(act), REPEAT_MS);
  post(act);
});
window.addEventListener('keyup', e => {
  const act = KEYS[e.key.toLowerCase()];
  if (!act || !held[act]) return;
  clearInterval(held[act]);
  delete held[act];
  post(releaseAction(act));
});

// ---- status polling ----
const el = {
  dist: document.getElementById('dist'),
  lr: document.getElementById('lr'),
  aeb: document.getElementById('tt-aeb'),
  aebVal: document.getElementById('aeb-val'),
  ldw: document.getElementById('tt-ldw'),
  ldwVal: document.getElementById('ldw-val'),
  fill: document.getElementById('fill'),
  fps: document.getElementById('fps'),
  link: document.getElementById('link'),
  events: document.getElementById('events'),
  flood: document.getElementById('flood')
};

const LADDER_MAX = 200; // cm shown on the ladder

function apply(s) {
  el.dist.textContent = s.dist >= 999 ? '---' : s.dist;
  el.lr.textContent = s.left + ' / ' + s.right;
  el.fps.textContent = s.fps + ' fps';
  el.events.textContent = s.aeb_events;

  const pct = Math.max(0, Math.min(100, (s.dist / LADDER_MAX) * 100));
  el.fill.style.height = pct + '%';
  el.fill.style.background =
    s.aeb === 'braking' ? '#e8402a' :
    s.aeb === 'caution' ? '#ffb020' : '#4fd1a5';

  el.aebVal.textContent = s.aeb;
  el.aeb.className = 'tt ' + (
    s.aeb === 'braking' ? 'alarm' :
    s.aeb === 'caution' ? 'caution' : 'good');

  el.ldwVal.textContent = s.ldw;
  el.ldw.className = 'tt ' + (
    s.ldw === 'departure' ? 'caution' :
    s.ldw === 'in lane' ? 'good' : '');

  el.link.textContent = s.link_ok ? 'link ok' : 'no link';
  el.link.className = 'link ' + (s.link_ok ? 'ok' : 'bad');

  if (s.state === 3) {
    el.link.textContent = 'MRM: link lost';
    el.link.className = 'link bad';
  }

  if (typeof s.lka_enabled === 'boolean' && s.lka_enabled !== lkaEnabled) {
    lkaEnabled = s.lka_enabled;
    lkaBtn.textContent = 'LKA: ' + (lkaEnabled ? 'On' : 'Off');
    lkaBtn.classList.toggle('active', lkaEnabled);
  }

  el.flood.classList.toggle('on', s.aeb === 'braking');
}

async function poll() {
  try {
    const r = await fetch('/status');
    apply(await r.json());
  } catch (err) {
    el.link.textContent = 'no server';
    el.link.className = 'link bad';
  }
}
setInterval(poll, 200);
poll();
