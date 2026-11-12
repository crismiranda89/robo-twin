// The real micro:bit car in a game: camera tracking (calibration made in real.html) + Bluetooth driving.
// Floor positions are "uv" = fractions of the projected picture (0..1, before the keystone), which is what the
// camera calibration maps to; the game turns uv into its own world coordinates.
// Settings are shared with real.html through localStorage 'robo-real' (same site: /twin on Jarvis).

const lsGet = (k, d) => { try { const v = JSON.parse(localStorage.getItem(k)); return v ?? d; } catch (e) { return d; } };
const lsSet = (k, v) => { try { localStorage.setItem(k, JSON.stringify(v)); } catch (e) {} };
export function fitH(pairs) {   // least squares, ≥4 pairs [[x,y],[u,v]] → [a..h,1]
  const M = Array.from({ length: 8 }, () => new Float64Array(9));
  const add = (r, b) => { for (let i = 0; i < 8; i++) { for (let j = 0; j < 8; j++) M[i][j] += r[i] * r[j]; M[i][8] += r[i] * b; } };
  for (const [[x, y], [u, v]] of pairs) { add([x, y, 1, 0, 0, 0, -u * x, -u * y], u); add([0, 0, 0, x, y, 1, -v * x, -v * y], v); }
  for (let c = 0; c < 8; c++) { let p = c; for (let r = c + 1; r < 8; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]]; if (Math.abs(M[c][c]) < 1e-12) return null;
    for (let r = 0; r < 8; r++) if (r !== c) { const f = M[r][c] / M[c][c]; for (let k = c; k < 9; k++) M[r][k] -= f * M[c][k]; } }
  return [...Array(8)].map((_, i) => M[i][8] / M[i][i]).concat(1);
}
export const applyH = (h, x, y) => { const w = h[6] * x + h[7] * y + h[8]; return [(h[0] * x + h[1] * y + h[2]) / w, (h[3] * x + h[4] * y + h[5]) / w]; };
const wait = ms => new Promise(r => setTimeout(r, ms));
// the micro:bit's Bluetooth UART takes at most 20 bytes per write (a longer one dropped the link, e.g. the two-colour
// LED command): send in 20-byte pieces — the car reads up to the end of the line, so the pieces join up again
async function writeChunks(ch, text) { const b = new TextEncoder().encode(text); for (let i = 0; i < b.length; i += 20) await ch.writeValue(b.slice(i, i + 20)); }
const UART = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
export const LEDC = { green: [0, 255, 0], red: [255, 0, 0], blue: [0, 0, 255], white: [255, 255, 255], off: [0, 0, 0] };
// LED command for the chosen colour; 'two' = half green, half red (firmware 'D'), so the camera sees where the car points
export const ledCmd = led => led === 'two' ? 'D0,255,0,255,0,0,150' : (c => `C${c[0]},${c[1]},${c[2]},255`)(LEDC[led] || LEDC.green);
export function invH(h) {
  const [a, b, c, d, e, f, g, k, i] = h, A = e * i - f * k, B = c * k - b * i, C = b * f - c * e, det = a * A + d * B + g * C;
  const r = [A, B, C, f * g - d * i, a * i - c * g, c * d - a * f, d * k - e * g, b * g - a * k, a * e - b * d].map(v => v / det);
  return r.map(v => v / r[8]);
}
// Where the green half and the red half of the LED ring are, around the car's spot (P, metres). Returns the angle of
// red→green on the floor, or null when the two colours are not both seen. d = camera RGBA at AW×AH.
export function ledAxis(d, AW, AH, H, Hi, P, arenaW, depth, lumMin = 60) {
  const [cu, cv] = applyH(Hi, P.x / arenaW, P.y / depth), [ru, rv] = applyH(Hi, (P.x + 0.09) / arenaW, P.y / depth);
  const rad = Math.max(4, Math.hypot((ru - cu) * AW, (rv - cv) * AH)), x0 = Math.max(0, Math.floor(cu * AW - rad)), x1 = Math.min(AW - 1, Math.ceil(cu * AW + rad));
  const y0 = Math.max(0, Math.floor(cv * AH - rad)), y1 = Math.min(AH - 1, Math.ceil(cv * AH + rad));
  let gw = 0, gx = 0, gy = 0, rw = 0, rx = 0, ry = 0;
  for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
    const j = (y * AW + x) * 4, r = d[j], g = d[j + 1], b = d[j + 2];
    if (0.299 * r + 0.587 * g + 0.114 * b < lumMin) continue;
    const G = Math.max(0, g - Math.max(r, b)), Rd = Math.max(0, r - Math.max(g, b));
    gw += G; gx += G * (x + .5); gy += G * (y + .5); rw += Rd; rx += Rd * (x + .5); ry += Rd * (y + .5);
  }
  if (gw < 300 || rw < 300) return null;
  const toM = (x, y) => { const [u, v] = applyH(H, x / AW, y / AH); return [u * arenaW, v * depth]; };
  const G = toM(gx / gw, gy / gw), Rr = toM(rx / rw, ry / rw), len = Math.hypot(G[0] - Rr[0], G[1] - Rr[1]);
  return len < 0.01 ? null : { ang: Math.atan2(G[1] - Rr[1], G[0] - Rr[0]), len, gw, rw };
}
// how the car answers the wheel numbers (measured in real.html → «Medir o carro»; these are guesses until then)
export const CAR_DEF = { carA: 0.006, carP0: 18, carB: 0.10, carMirror: false };

const angDiff = (a, b) => Math.atan2(Math.sin(a - b), Math.cos(a - b));
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

// Where the car points, as seen in the picture (radians, x right, y down). The camera only shows a spot of light, so the
// heading comes from the way it moves, and in between from the turn we asked for.
export class Heading {
  constructor() { this.th = null; this.fixT = -1e9; this.t = null; }
  update(now, P, vCmd, wCmd) {
    const dt = this.t == null ? 0 : Math.min(0.2, (now - this.t) / 1000); this.t = now;
    if (this.th != null) this.th += wCmd * dt;
    const sp = Math.hypot(P.vx, P.vy);
    if (P.ok && sp > 0.05 && Math.abs(vCmd) > 0.02) {
      const thM = Math.atan2(P.vy, P.vx) + (vCmd < 0 ? Math.PI : 0);
      this.th = this.th == null ? thM : this.th + angDiff(thM, this.th) * Math.min(1, dt * 5); this.fixT = now;
    }
  }
  led(now, ang, phi) {   // a reading from the two-colour LEDs: works even when the car stands still
    const hm = ang - phi; this.th = this.th == null ? hm : this.th + angDiff(hm, this.th) * 0.5; this.fixT = now;
  }
  sinceFix(now) { return (now - this.fixT) / 1000; }
}
// "Easy" driving: the stick says where to go on the picture (x right, y down), the car turns that way and goes.
export function easyCmd(sx, sy, th, vmax, wmax = 5) {
  const mag = Math.min(1, Math.hypot(sx, sy)); if (mag < 0.15) return { v: 0, w: 0 };
  if (th == null) return { v: vmax * 0.5 * mag, w: 0 };          // heading not known yet: creep forward to find it
  const err = angDiff(Math.atan2(sy, sx), th);
  return { v: vmax * mag * Math.max(0, Math.cos(err)) ** 3, w: clamp(err * 7, -wmax, wmax) };
}
// speed (m/s) + turn (rad/s, + = clockwise in the picture) → wheel numbers -100..100, from the measured car
export function wheelsFor(v, w, S) {
  const { carA, carP0, carB, carMirror } = Object.assign({}, CAR_DEF, S), wp = carMirror ? -w : w;
  // below ~30 % the motors only hum: anything that should move gets at least carMin (the measured start may say less)
  const lo = S.carMin ?? 30, pct = s => Math.abs(s) < 0.01 ? 0 : Math.sign(s) * Math.max(lo, carP0 + Math.abs(s) / carA);
  let l = pct(v + wp * carB / 2), r = pct(v - wp * carB / 2);
  const m = Math.max(Math.abs(l), Math.abs(r)); if (m > 100) { l *= 100 / m; r *= 100 / m; }
  return [Math.round(l), Math.round(r)];
}
export const carVmax = (S, pct = 100) => { const c = Object.assign({}, CAR_DEF, S); return Math.max(0.05, c.carA * (pct - c.carP0)); };

// ---------- parallax: the LEDs are above the floor ----------
// The calibration is made on the floor, but the LED ring sits a few cm up, so seen from the side its light lands on the
// floor further from the camera than the car is. From the floor→camera homography (square pixels, centre of the picture
// = optical centre) we get the camera's focal length and position; then a light at height h is pulled back towards the
// point under the camera by h / camera height.
const mul3 = (A, B) => { const r = []; for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) r.push(A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j]); return r; };
export function cameraPose(H, arenaW, depth, W = 1280, Hh = 720) {
  if (!H) return null;
  const G = mul3(mul3([W, 0, -W / 2, 0, Hh, -Hh / 2, 0, 0, 1], invH(H)), [1 / arenaW, 0, 0, 0, 1 / depth, 0, 0, 0, 1]);   // floor m → centred pixels
  const g1 = [G[0], G[3], G[6]], g2 = [G[1], G[4], G[7]], g3 = [G[2], G[5], G[8]], f2s = [];
  const a = -(g1[0] * g2[0] + g1[1] * g2[1]) / (g1[2] * g2[2]); if (a > 0) f2s.push(a);
  const b = (g1[0] ** 2 + g1[1] ** 2 - g2[0] ** 2 - g2[1] ** 2) / (g2[2] ** 2 - g1[2] ** 2); if (b > 0) f2s.push(b);
  if (!f2s.length) return null;
  const f = Math.sqrt(f2s.reduce((s, v) => s + v, 0) / f2s.length), ki = g => [g[0] / f, g[1] / f, g[2]];
  const n1 = ki(g1), lam = 1 / Math.hypot(...n1), r1 = n1.map(v => v * lam), r2 = ki(g2).map(v => v * lam), t = ki(g3).map(v => v * lam);
  const r3 = [r1[1] * r2[2] - r1[2] * r2[1], r1[2] * r2[0] - r1[0] * r2[2], r1[0] * r2[1] - r1[1] * r2[0]];
  const C = [r1, r2, r3].map(r => -(r[0] * t[0] + r[1] * t[1] + r[2] * t[2]));   // C = -Rᵀ t, R = [r1 r2 r3] as columns
  const hC = Math.abs(C[2]);
  return hC > 0.3 && hC < 8 ? { x: C[0], y: C[1], h: hC, f } : null;
}
export function unParallax(x, y, pose, ledH) {
  if (!pose || !ledH) return [x, y]; const k = 1 - ledH / pose.h;
  return k > 0.5 ? [pose.x + (x - pose.x) * k, pose.y + (y - pose.y) * k] : [x, y];
}

export function settings() { const s = lsGet('robo-real', {}); if (!s.ledV) s.led = 'two';   // two-colour LEDs became the default (2026-11-12)
  // camera → picture = camera → projector screen (fixed while camera and projector stay put) + the current corners
  const kn = lsGet('robo-keystone', null);
  if (s.Hs && Array.isArray(kn) && kn.length === 4) { const Hk = fitH([[0, 0], [1, 0], [1, 1], [0, 1]].map((p, i) => [p, kn[i]]));
    if (Hk) { const A = invH(Hk), B = s.Hs, r = []; for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) r.push(A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j]); s.H = r.map(v => v / r[8]); } }
  return Object.assign({ arenaW: 2, thr: 200, minA: 3, det: 'bright', led: 'two', carLedPhi: null, camId: '', H: null }, CAR_DEF, s); }
export function saveSettings(patch) { const s = Object.assign(lsGet('robo-real', {}), patch); lsSet('robo-real', s); return s; }

export class RealCar {
  constructor() {
    this.S = settings();
    this.AW = 480; this.AH = 270;
    this.A = document.createElement('canvas'); this.A.width = this.AW; this.A.height = this.AH;
    this.ax = this.A.getContext('2d', { willReadFrequently: true });
    this.vid = document.createElement('video'); this.vid.muted = true; this.vid.playsInline = true;
    this.stream = null; this.fps = 0; this._frames = 0; this._fpsT = performance.now(); this.blobs = [];
    // pose in metres on the floor (x right, y towards the bottom of the picture), velocity in m/s
    this.P = { ok: false, x: 0, y: 0, vx: 0, vy: 0, t: 0 };
    this.exclude = null;                 // (mx, my) => true for spots the game itself projects (e.g. the halo around the car)
    this.dev = null; this.rx = null; this.busy = false; this.queued = []; this.pending = null; this.lastCmd = ''; this.lastSendT = 0; this.teleT = 0;
    this.onchange = () => {};
    this.H = new Heading(); this.cmdV = 0; this.cmdW = 0; this.acq = false; this.lastAcqT = -1e9; this.acqInfo = null;
  }
  get th() { return this.H.th; }
  get calibrated() { return !!this.S.H; }
  get depth() { const PA = screen.width && screen.height ? screen.width / screen.height : innerWidth / innerHeight; return this.S.arenaW / PA; }
  get connected() { return !!this.rx; }
  get seen() { return this.P.ok && performance.now() - this.P.t < 400; }
  uv() { return [this.P.x / this.S.arenaW, this.P.y / this.depth]; }

  // ---------- camera ----------
  async startCamera() {
    if (this.stream) return true;
    try {
      this.stream = await navigator.mediaDevices.getUserMedia({ video: { deviceId: this.S.camId ? { exact: this.S.camId } : undefined, width: { ideal: 1280 }, height: { ideal: 720 }, frameRate: { ideal: 60 } } });
    } catch (e) { this.camError = e.message; this.onchange(); return false; }
    this.vid.srcObject = this.stream; await this.vid.play();
    this.AH = Math.round(this.AW * this.vid.videoHeight / this.vid.videoWidth); this.A.height = this.AH;
    this.buildMask();
    const loop = () => { this.frame(); this.vid.requestVideoFrameCallback ? this.vid.requestVideoFrameCallback(loop) : requestAnimationFrame(loop); };
    loop(); this.onchange(); return true;
  }
  buildMask() {
    const { AW, AH } = this, H = this.S.H; if (!H) { this.inside = null; return; } this.Hi = invH(H);
    this.pose = cameraPose(H, this.S.arenaW, this.depth, this.vid.videoWidth || 1280, this.vid.videoHeight || 720);
    this.inside = new Uint8Array(AW * AH);
    for (let y = 0; y < AH; y++) for (let x = 0; x < AW; x++) { const [u, v] = applyH(H, (x + .5) / AW, (y + .5) / AH); this.inside[y * AW + x] = u > -0.03 && u < 1.03 && v > -0.03 && v < 1.03 ? 1 : 0; }
  }
  score(d, j) {
    const L = 0.299 * d[j] + 0.587 * d[j + 1] + 0.114 * d[j + 2];
    if (this.S.det === 'bright') return L;
    const r = d[j], g = d[j + 1], b = d[j + 2], c = this.S.led === 'red' ? r - Math.max(g, b) : this.S.led === 'blue' ? b - Math.max(r, g) : this.S.led === 'green' ? g - Math.max(r, b) : this.S.led === 'two' ? Math.max(g - Math.max(r, b), r - Math.max(g, b)) : 0;
    return Math.max(c * 1.6, L > 245 ? L : 0);
  }
  toM(cx, cy) { const [u, v] = applyH(this.S.H, cx, cy); return unParallax(u * this.S.arenaW, v * this.depth, this.pose, (this.S.ledH ?? 6) / 100); }
  frame() {
    if (!this.stream || this.vid.readyState < 2 || !this.S.H) return;
    const { AW, AH } = this, n = AW * AH, thr = this.S.thr, now = performance.now();
    this.ax.drawImage(this.vid, 0, 0, AW, AH); const d = this.ax.getImageData(0, 0, AW, AH).data;
    const sc = new Float32Array(n), seen = new Uint8Array(n), out = [], st = [];
    for (let i = 0; i < n; i++) if (!this.inside || this.inside[i]) sc[i] = this.score(d, i * 4);
    for (let i = 0; i < n; i++) {
      if (seen[i] || sc[i] < thr) continue;
      let a = 0, sx = 0, sy = 0, x0 = 1e9, y0 = 1e9, x1 = -1, y1 = -1; st.push(i); seen[i] = 1;
      while (st.length) { const k = st.pop(), x = k % AW, y = (k / AW) | 0; a++; sx += x + .5; sy += y + .5;
        if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y;
        for (const q of [k - 1, k + 1, k - AW, k + AW]) if (q >= 0 && q < n && !seen[q] && sc[q] >= thr && Math.abs((q % AW) - x) <= 1) { seen[q] = 1; st.push(q); } }
      if (a < this.S.minA || out.length > 200) continue;
      const b = { cx: sx / a / AW, cy: sy / a / AH, a, box: [x0 / AW, y0 / AH, (x1 + 1) / AW, (y1 + 1) / AH], fill: a / ((x1 - x0 + 1) * (y1 - y0 + 1)) };
      [b.mx, b.my] = this.toM(b.cx, b.cy);
      const p0 = this.toM(b.box[0], b.box[1]), p1 = this.toM(b.box[2], b.box[3]);
      b.size = Math.hypot(p1[0] - p0[0], p1[1] - p0[1]);
      b.bad = b.size > 0.35 || (b.size > 0.12 && b.fill < 0.2) || (this.exclude ? this.exclude(b.mx, b.my) : false);
      out.push(b);
    }
    this.blobs = out; if (!this.acq) this.locate(out, now); this.H.update(now, this.P, this.cmdV, this.cmdW);
    if (this.P.ok && this.S.led === 'two' && this.Hi) {
      const ax = ledAxis(d, AW, AH, this.S.H, this.Hi, this.P, this.S.arenaW, this.depth); this.axis = ax;
      if (ax) { this.learnPhi(ax.ang, now); if (this.S.carLedPhi != null) this.H.led(now, ax.ang, this.S.carLedPhi); } }
    this._frames++; if (now - this._fpsT > 1000) { this.fps = this._frames * 1000 / (now - this._fpsT); this._frames = 0; this._fpsT = now; this.onchange(); }
  }
  // With the game projected the camera sees 100+ bright spots, so the car is never "the biggest light": once found it is
  // followed only near where it was; when lost (or when it should be moving and its spot is not), it is found again by
  // switching its LEDs off and on and looking at what changed (acquire()).
  locate(bl, now) {
    const T = this.P, good = bl.filter(b => !b.bad);
    const dtk = Math.max(1e-3, (now - T.t) / 1000);
    let seed = null;
    if (T.ok && dtk < 1.5) {
      const px = T.x + T.vx * Math.min(dtk, 0.3), py = T.y + T.vy * Math.min(dtk, 0.3);
      let bd = 0.22; for (const b of good) { const dd = Math.hypot(b.mx - px, b.my - py); if (dd < bd) { bd = dd; seed = b; } }
      if (!seed) { if (now - T.t > 400) { T.ok = false; this.lostT = now; } return; }
    } else {
      T.ok = false;
      if (this.rx && this.stream) { if (now - this.lastAcqT > 2000) this.acquire('perdido'); return; }   // the car can blink: find it that way
      if (!good.length) return;
      let best = -1; for (const b of good) { let a = 0; for (const o of good) if (Math.hypot(o.mx - b.mx, o.my - b.my) < 0.10) a += o.a; if (a > best) { best = a; seed = b; } }
    }
    let w = 0, x = 0, y = 0; for (const b of good) if (Math.hypot(b.mx - seed.mx, b.my - seed.my) < 0.10) { w += b.a; x += b.a * b.mx; y += b.a * b.my; }
    x /= w; y /= w;
    if (!T.ok || dtk > 0.5) { T.x = x; T.y = y; T.vx = T.vy = 0; }
    else { const nx = T.x + 0.6 * (x - T.x), ny = T.y + 0.6 * (y - T.y), k = Math.min(1, dtk * 6);
      T.vx += k * ((nx - T.x) / dtk - T.vx); T.vy += k * ((ny - T.y) / dtk - T.vy); T.x = nx; T.y = ny; }
    T.ok = true; T.t = now;
    // told to drive forward for 1.5 s but the spot has not moved: probably following something else → check by blinking
    const lr = this.lastLR || [0, 0];
    if (this.rx && (lr[0] + lr[1]) / 2 > 25) {
      if (!this.stuck || Math.hypot(T.x - this.stuck.x, T.y - this.stuck.y) > 0.04) this.stuck = { x: T.x, y: T.y, t: now };
      else if (now - this.stuck.t > 1500 && now - this.lastAcqT > 2000) { this.stuck = null; this.acquire('não se mexe'); }
    } else this.stuck = null;
  }
  // which way the green half points on the car: learnt while it drives forward (colour axis vs the way it moves)
  learnPhi(ang, now) {
    const P = this.P, sp = Math.hypot(P.vx, P.vy); if (sp < 0.08 || this.cmdV < 0.03 || Math.abs(this.cmdW) > 1.5) return;
    const o = ang - Math.atan2(P.vy, P.vx), L = this.phiL || (this.phiL = { x: 0, y: 0, n: 0, saved: 0 });
    L.x = L.x * 0.97 + Math.cos(o); L.y = L.y * 0.97 + Math.sin(o); L.n++;
    if (L.n >= 20 && Math.hypot(L.x, L.y) > 0.6 * Math.min(L.n, 33)) {   // enough samples that agree
      this.S.carLedPhi = +Math.atan2(L.y, L.x).toFixed(3);
      if (now - L.saved > 3000) { L.saved = now; saveSettings({ carLedPhi: this.S.carLedPhi }); } }
  }
  luma() { const { AW, AH } = this; this.ax.drawImage(this.vid, 0, 0, AW, AH); const d = this.ax.getImageData(0, 0, AW, AH).data, L = new Float32Array(AW * AH);
    for (let i = 0; i < AW * AH; i++) L[i] = 0.299 * d[i * 4] + 0.587 * d[i * 4 + 1] + 0.114 * d[i * 4 + 2]; return L; }
  // LEDs off → picture, LEDs on → picture: the car is where the light appeared
  async acquire(why = '') {
    if (this.acq || !this.rx || !this.stream || !this.S.H) return false;
    this.acq = true; this.lastAcqT = performance.now(); this.wheels(0, 0); this.onchange();
    const on = ledCmd(this.S.led === 'off' ? 'green' : this.S.led), { AW, AH } = this;
    let found = null;
    try {
      for (let tries = 0; tries < 2 && !found; tries++) {
        this.send('C0,0,0'); await wait(450); const A = this.luma();
        this.send(on); await wait(450); const B = this.luma();
        let best = 0, bi = -1; const diff = new Float32Array(AW * AH);
        for (let i = 0; i < AW * AH; i++) { if (this.inside && !this.inside[i]) continue; const q = B[i] - A[i]; diff[i] = q; if (q > best) { best = q; bi = i; } }
        if (best < 45) continue;
        const thr = Math.max(30, best * 0.35), seen = new Uint8Array(AW * AH), st = [bi]; seen[bi] = 1; let sw = 0, sx = 0, sy = 0, n = 0;
        while (st.length && n < 4000) { const k = st.pop(), x = k % AW, y = (k / AW) | 0, wv = diff[k]; sw += wv; sx += wv * (x + .5); sy += wv * (y + .5); n++;
          for (const q of [k - 1, k + 1, k - AW, k + AW, k - 2, k + 2, k - 2 * AW, k + 2 * AW]) if (q >= 0 && q < AW * AH && !seen[q] && diff[q] >= thr && Math.abs((q % AW) - x) <= 2) { seen[q] = 1; st.push(q); } }
        found = { m: this.toM(sx / sw / AW, sy / sw / AH), best, n };
      }
    } finally { this.acq = false; }
    const now = performance.now(), T = this.P; this.lastAcqT = now;
    if (found) { [T.x, T.y] = found.m; T.vx = T.vy = 0; T.ok = true; T.t = now; this.stuck = null; }
    this.acqInfo = { ok: !!found, why, best: found ? Math.round(found.best) : 0, t: now }; this.onchange();
    return !!found;
  }
  drawPreview(c) {   // small camera view with what the tracker sees (debug)
    if (!this.stream) return; const g = c.getContext('2d'), W = c.width, H = c.height; g.drawImage(this.vid, 0, 0, W, H);
    for (const b of this.blobs) { g.strokeStyle = b.bad ? '#777' : '#ff3030'; g.lineWidth = 2; g.strokeRect(b.box[0] * W - 2, b.box[1] * H - 2, (b.box[2] - b.box[0]) * W + 4, (b.box[3] - b.box[1]) * H + 4); }
  }

  // ---------- Bluetooth (same protocol as Jarvis /robo) ----------
  async connect() {
    if (!navigator.bluetooth) throw new Error('Este browser não tem Web Bluetooth (usa o Chrome).');
    this.dev = await navigator.bluetooth.requestDevice({ filters: [{ namePrefix: 'BBC micro:bit' }], optionalServices: [UART] });
    this.dev.addEventListener('gattserverdisconnected', () => this.lost());
    await this.open();
  }
  async open() {
    const svc = await (await this.dev.gatt.connect()).getPrimaryService(UART);
    let tx = null; for (const c of await svc.getCharacteristics()) { if (c.properties.notify || c.properties.indicate) tx = c; else if (c.properties.write || c.properties.writeWithoutResponse) this.rx = c; }
    if (!tx || !this.rx) throw new Error('serviço UART incompleto');
    tx.addEventListener('characteristicvaluechanged', () => { this.teleT = performance.now(); });
    await tx.startNotifications(); this.retry = 0;
    this.send(ledCmd(this.S.led));
    setTimeout(() => this.acquire('ligado'), 600);
    this.onchange();
  }
  async lost() {
    this.rx = null; this.busy = false; this.queued = []; this.pending = null; this.onchange();
    if (!this.dev || (this.retry = (this.retry || 0) + 1) > 3) return;
    try { await this.open(); } catch (e) { setTimeout(() => this.lost(), 1500); }
  }
  async pump() {
    if (this.busy || !this.rx) return;
    let cmd; if (this.queued.length) cmd = this.queued.shift(); else { cmd = this.pending; this.pending = null; }
    if (cmd == null) return;
    this.busy = true;
    try { await Promise.race([writeChunks(this.rx, cmd + '\n'), wait(1000).then(() => { throw new Error('timeout'); })]); } catch (e) {}
    this.busy = false; this.pump();
  }
  send(cmd) { this.queued.push(cmd); this.pump(); }
  wheels(l, r) {   // -100..100 each; repeated while moving (the car stops by itself 0.7 s after the last one)
    l = Math.round(Math.max(-100, Math.min(100, l))); r = Math.round(Math.max(-100, Math.min(100, r)));
    const cmd = `M${l},${r}`, now = performance.now();
    this.lastLR = [l, r];
    if (!this.rx || (cmd === this.lastCmd && (!(l || r) || now - this.lastSendT < 150))) return;
    this.pending = cmd; this.lastCmd = cmd; this.lastSendT = now; this.pump();
  }
  stop() { this.cmdV = this.cmdW = 0; this.wheels(0, 0); }
  // speed (m/s) + turn rate (rad/s, positive = clockwise as seen in the picture). If the heading has not been confirmed
  // by movement for a while (e.g. it only turned on the spot), creep forward so the camera can see where it points.
  drive(v, w) {
    const now = performance.now();
    if (this.acq) { this.cmdV = this.cmdW = 0; this.wheels(0, 0); return; }   // stand still while it blinks to be found
    if ((Math.abs(w) > 0.3 || v > 0.01) && this.H.sinceFix(now) > 1.5 && this.seen) v = Math.max(v, 0.08);
    this.cmdV = v; this.cmdW = w;
    const [l, r] = wheelsFor(v, w, this.S); this.wheels(l, r);
  }
}
