/*
 * goniometre_core.js — Temas açısı ölçüm motoru (saf JS, DOM'suz).
 * contact_angle.py'nin birebir portu. Hem tarayıcıda (<script>) hem Node'da çalışır.
 * Girdi: gri tonlamalı Float/Uint dizi (satır-öncelikli) + genişlik/yükseklik + baseline.
 * Bağımlılık YOK — kendi 3x3 en-küçük-kareler çözücüsünü içerir.
 */
(function (root) {
  "use strict";

  // ── 3x3 lineer sistem çözücü (Cramer) — daire-fit ve polyfit için ───────────
  function solve3(M, v) {
    const d = det3(M);
    if (Math.abs(d) < 1e-12) return null;
    const mx = [[v[0], M[0][1], M[0][2]], [v[1], M[1][1], M[1][2]], [v[2], M[2][1], M[2][2]]];
    const my = [[M[0][0], v[0], M[0][2]], [M[1][0], v[1], M[1][2]], [M[2][0], v[2], M[2][2]]];
    const mz = [[M[0][0], M[0][1], v[0]], [M[1][0], M[1][1], v[1]], [M[2][0], M[2][1], v[2]]];
    return [det3(mx) / d, det3(my) / d, det3(mz) / d];
  }
  function det3(m) {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
         - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
         + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  }
  // Normal denklemler: satır tasarım matrisi A (Nx3) + hedef b → A^T A x = A^T b
  function lstsq3(rows, targets) {
    const M = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], v = [0, 0, 0];
    for (let i = 0; i < rows.length; i++) {
      const a = rows[i], t = targets[i];
      for (let r = 0; r < 3; r++) {
        v[r] += a[r] * t;
        for (let c = 0; c < 3; c++) M[r][c] += a[r] * a[c];
      }
    }
    return solve3(M, v);
  }

  // ── Otsu eşiği (arka plan / damla ayrımı) ───────────────────────────────────
  function otsu(gray) {
    const hist = new Float64Array(256);
    for (let i = 0; i < gray.length; i++) hist[Math.max(0, Math.min(255, gray[i] | 0))]++;
    const total = gray.length;
    let sumAll = 0;
    for (let t = 0; t < 256; t++) sumAll += t * hist[t];
    let wB = 0, sumB = 0, best = 127, bestVar = -1;
    for (let t = 0; t < 256; t++) {
      wB += hist[t];
      if (wB === 0) continue;
      const wF = total - wB;
      if (wF === 0) break;
      sumB += t * hist[t];
      const mB = sumB / wB, mF = (sumAll - sumB) / wF;
      const varBetween = wB * wF * (mB - mF) * (mB - mF);
      if (varBetween > bestVar) { bestVar = varBetween; best = t; }
    }
    return best;
  }

  // ── Damla maskesi (baseline üstü) — koyu/açık damlayı otomatik seçer ─────────
  function dropletMask(gray, W, baselineY) {
    const t = otsu(gray);
    // Baseline üstünde koyu piksel oranı
    let dark = 0, n = baselineY * W;
    for (let i = 0; i < n; i++) if (gray[i] <= t) dark++;
    const darkIsDroplet = (dark / n) < 0.5;   // azınlık sınıf = damla
    const mask = new Uint8Array(gray.length);
    for (let i = 0; i < n; i++) {
      const isDark = gray[i] <= t;
      mask[i] = (isDark === darkIsDroplet) ? 1 : 0;
    }
    return mask;
  }

  // ── Kenar noktaları: her satırın en geniş ardışık koşusunun sol/sağ ucu ──────
  function edgePoints(mask, W, baselineY) {
    const pts = [];               // {x,y} daire-fit için
    const leftX = {}, rightX = {}; // teğet için satır→x
    for (let y = 0; y < baselineY; y++) {
      const row = y * W;
      let bestLo = -1, bestHi = -1, bestLen = 0;
      let lo = -1, prev = -2;
      for (let x = 0; x < W; x++) {
        if (mask[row + x]) {
          if (x !== prev + 1) lo = x;      // yeni koşu başladı
          const len = x - lo + 1;
          if (len > bestLen) { bestLen = len; bestLo = lo; bestHi = x; }
          prev = x;
        }
      }
      if (bestLo >= 0) {
        leftX[y] = bestLo; rightX[y] = bestHi;
        pts.push([bestLo, y]); pts.push([bestHi, y]);
      }
    }
    return { pts, leftX, rightX };
  }

  // ── Daire-fit (Kåsa) → {cx,cy,r} ────────────────────────────────────────────
  function fitCircle(pts) {
    const rows = [], targets = [];
    for (const [x, y] of pts) { rows.push([x, y, 1]); targets.push(-(x * x + y * y)); }
    const sol = lstsq3(rows, targets);
    if (!sol) return null;
    const [D, E, F] = sol;
    const cx = -D / 2, cy = -E / 2;
    const r = Math.sqrt(Math.max(cx * cx + cy * cy - F, 1e-9));
    return { cx, cy, r };
  }

  // θ = 90° − arcsin(H/r); H = merkezin baseline üstü yüksekliği (görüntü y aşağı)
  function angleFromCircle(cy, r, baselineY) {
    const H = baselineY - cy;
    const s = Math.max(-1, Math.min(1, H / r));
    return 90 - (Math.asin(s) * 180 / Math.PI);
  }

  // ── Yerel teğet (sol/sağ ayrı, tüm 0–180° aralık) ───────────────────────────
  function localTangent(edgeByY, baselineY, side, window) {
    window = window || 30;
    const rows = [], targets = [];
    for (let y = baselineY - window; y < baselineY; y++) {
      if (edgeByY[y] === undefined) continue;
      const Y = baselineY - y;                 // yukarı-pozitif, temasta Y=0
      rows.push([Y * Y, Y, 1]); targets.push(edgeByY[y]);  // X = a Y² + b Y + c
    }
    if (rows.length < 4) return null;
    const sol = lstsq3(rows, targets);
    if (!sol) return null;
    const slope = sol[1];                       // dX/dY temasta = b
    const raw = Math.atan2(1, side === "left" ? slope : -slope) * 180 / Math.PI;
    return 180 - raw;                           // sıvı içinden ölçülen açı
  }

  function wettingLabel(theta) {
    if (theta < 90) return "hidrofilik (yayılır)";
    if (theta < 150) return "hidrofobik (boncuklanır)";
    return "süper-hidrofobik";
  }

  // ── Ana ölçüm ───────────────────────────────────────────────────────────────
  function measure(gray, W, H, baselineY) {
    baselineY = baselineY | 0;
    const mask = dropletMask(gray, W, baselineY);
    const { pts, leftX, rightX } = edgePoints(mask, W, baselineY);
    if (pts.length < 10) throw new Error("Damla bulunamadı — baseline/kontrast/arka ışığı kontrol et.");
    const circ = fitCircle(pts);
    const thetaCircle = angleFromCircle(circ.cy, circ.r, baselineY);
    const tl = localTangent(leftX, baselineY, "left");
    const tr = localTangent(rightX, baselineY, "right");
    const disc = circ.r * circ.r - (baselineY - circ.cy) * (baselineY - circ.cy);
    const half = disc > 0 ? Math.sqrt(disc) : 0;
    return {
      baselineY,
      thetaCircle: round1(thetaCircle),
      thetaLeft: tl == null ? null : round1(tl),
      thetaRight: tr == null ? null : round1(tr),
      wetting: wettingLabel(thetaCircle),
      circle: { cx: round1(circ.cx), cy: round1(circ.cy), r: round1(circ.r) },
      contacts: [[circ.cx - half, baselineY], [circ.cx + half, baselineY]],
    };
  }
  function round1(x) { return Math.round(x * 10) / 10; }

  // ── Sentetik damla üretici (test/doğrulama) ─────────────────────────────────
  function synth(thetaDeg, opts) {
    opts = opts || {};
    const r = opts.r || 160, W = opts.W || 640, Hh = opts.H || 480;
    const baselineY = opts.baselineY || 360, cx = opts.cx || 320;
    const th = thetaDeg * Math.PI / 180;
    const cy = baselineY - r * Math.cos(th);
    const gray = new Float64Array(W * Hh);
    for (let y = 0; y < Hh; y++) {
      for (let x = 0; x < W; x++) {
        const inDisk = (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r;
        gray[y * W + x] = (inDisk && y < baselineY) ? 30 : 240;
      }
    }
    return { gray, W, H: Hh, baselineY };
  }

  const API = { measure, synth, otsu, fitCircle, angleFromCircle, localTangent,
                dropletMask, edgePoints, wettingLabel };
  if (typeof module !== "undefined" && module.exports) module.exports = API;
  else root.Goniometre = API;
})(typeof self !== "undefined" ? self : this);
