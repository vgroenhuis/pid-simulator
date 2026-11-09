// ─── Default parameter values ────────────────────────────────────────────────
// Plant: mass m (kg), spring constant k (N/m), damping coefficient c (N·s/m),
//        constant disturbance force f (N).
// Controller: proportional Kp, integral Ki, derivative Kd gains.
const defaults = {
  m: 1,
  k: 0,
  c: 3,
  f: 0,
  kp: 0,
  ki: 0,
  kd: 0
};

// ─── DOM references ───────────────────────────────────────────────────────────
// Slider input elements
const sliders = {
  m: document.getElementById("m"),
  k: document.getElementById("k"),
  c: document.getElementById("c"),
  f: document.getElementById("f"),
  kp: document.getElementById("kp"),
  ki: document.getElementById("ki"),
  kd: document.getElementById("kd")
};

// Numeric value display elements next to each slider
const outputs = {
  m: document.getElementById("mOut"),
  k: document.getElementById("kOut"),
  c: document.getElementById("cOut"),
  f: document.getElementById("fOut"),
  kp: document.getElementById("kpOut"),
  ki: document.getElementById("kiOut"),
  kd: document.getElementById("kdOut")
};

// Statistics panel elements
const metricsEl = {
  rise: document.getElementById("riseTime"),
  overshoot: document.getElementById("overshoot"),
  settling: document.getElementById("settlingTime"),
  sse: document.getElementById("sse"),
  dampingRatio: document.getElementById("dampingRatio"),
  dampingType: document.getElementById("dampingType")
};

const statusEl = document.getElementById("status");
const autoScaleYEl = document.getElementById("autoScaleY");
const chart = document.getElementById("chart");
const ctx = chart.getContext("2d");

// Fixed Y-axis range used when auto-scale is disabled, chosen to comfortably
// show the unit step reference and typical over/undershoot.
const fixedYAxis = {
  min: -0.5,
  max: 1.5
};

// ─── Parameter helpers ───────────────────────────────────────────────────────

// Read current slider positions and return all parameters as a plain object.
function getParams() {
  return {
    m: Number(sliders.m.value),
    k: Number(sliders.k.value),
    c: Number(sliders.c.value),
    f: Number(sliders.f.value),
    kp: Number(sliders.kp.value),
    ki: Number(sliders.ki.value),
    kd: Number(sliders.kd.value)
  };
}

// Update the numeric value labels shown next to each slider.
function updateValueLabels(p) {
  outputs.m.innerHTML = `${p.m.toFixed(2)} <span class="unit">kg</span>`;
  outputs.k.innerHTML = `${p.k.toFixed(2)} <span class="unit">N/m</span>`;
  outputs.c.innerHTML = `${p.c.toFixed(2)} <span class="unit">N·s/m</span>`;
  outputs.f.innerHTML = `${p.f.toFixed(2)} <span class="unit">N</span>`;
  outputs.kp.textContent = p.kp.toFixed(2);
  outputs.ki.textContent = p.ki.toFixed(2);
  outputs.kd.textContent = p.kd.toFixed(2);
}

// ─── Simulation ──────────────────────────────────────────────────────────────

// Simulate the closed-loop step response using forward Euler integration.
//
// Plant model (mass-spring-damper):
//   m·ẍ + c·ẋ + k·x = u + f
//
// Parallel PID controller:
//   u = Kp·e + Ki·∫e dt - Kd·(dy/dt)
//   where e = r − x  (error between unit step reference r=1 and output x)
//   Note: Derivative acts on output (not error) to avoid derivative kick on setpoint changes.
//
// The derivative term is low-pass filtered to reduce noise amplification.
// The integral term is clamped (anti-windup) to prevent integrator saturation.
// Returns arrays of time, output y, and reference ref, plus a stability flag.
function simulateStepResponse(params) {
  const tStart = -1; // start time (s)
  const tEnd = 12;   // end time (s)
  const dt = 0.004;  // fixed time step (s)
  const n = Math.floor((tEnd - tStart) / dt) + 1;

  const time = new Array(n);
  const y = new Array(n);
  const ref = new Array(n);

  let x = 0;
  let v = 0;
  let integral = 0;
  let prevX = 0;
  let dFiltered = 0;

  const dAlpha = 1;     // derivative low-pass filter coefficient (0=no filter, 1=no memory)
  const maxAbsState = 1e4; // divergence threshold – marks simulation as unstable

  let stable = true;

  for (let i = 0; i < n; i += 1) {
    const t = tStart + i * dt;
    const r = t >= 0 ? 1 : 0;
    const error = r - x;

    integral += error * dt;
    // integral anti-windup clamping disabled

    const dOutput = (x - prevX) / dt;
    dFiltered = dAlpha * dOutput + (1 - dAlpha) * dFiltered;

    const u = params.kp * error + params.ki * integral - params.kd * dFiltered;
    const a = (u + params.f - params.c * v - params.k * x) / params.m;

    prevX = x;  // Save current x before updating
    v += a * dt;
    x += v * dt;

    if (!Number.isFinite(x) || !Number.isFinite(v) || Math.abs(x) > maxAbsState || Math.abs(v) > maxAbsState) {
      stable = false;
      for (let j = i; j < n; j += 1) {
        time[j] = tStart + j * dt;
        y[j] = Number.NaN;
        ref[j] = tStart + j * dt >= 0 ? 1 : 0;
      }
      break;
    }

    time[i] = t;
    y[i] = x;
    ref[i] = r;
  }

  return { time, y, ref, stable, dt };
}

// ─── Step-response metrics ───────────────────────────────────────────────────

// Compute standard step-response performance metrics from the simulated output.
//   Rise time   : time for output to go from 10 % to 90 % of the setpoint.
//   Overshoot   : peak value above setpoint, expressed as % of setpoint.
//   Settling time: first time the output stays within ±2 % of the final value.
//   SSE         : absolute steady-state error  |1 − y_final|.
// Settling band is centred on the actual final value (not necessarily setpoint)
// to handle cases with non-zero steady-state error.
function computeMetrics(time, y) {
  const finitePairs = [];
  for (let i = 0; i < y.length; i += 1) {
    if (Number.isFinite(y[i])) finitePairs.push([time[i], y[i]]);
  }
  if (finitePairs.length < 2) {
    return { riseTime: null, overshoot: null, settlingTime: null, sse: null };
  }

  const yFinal = finitePairs[finitePairs.length - 1][1];
  const absFinalError = Math.abs(1 - yFinal);

  let t10 = null;
  let t90 = null;
  for (let i = 0; i < finitePairs.length; i += 1) {
    const [t, yv] = finitePairs[i];
    if (t10 === null && yv >= 0.1) t10 = t;
    if (t90 === null && yv >= 0.9) {
      t90 = t;
      break;
    }
  }

  const riseTime = t10 !== null && t90 !== null ? Math.max(0, t90 - t10) : null;

  let yPeak = -Infinity;
  for (let i = 0; i < finitePairs.length; i += 1) {
    if (finitePairs[i][1] > yPeak) yPeak = finitePairs[i][1];
  }
  const overshoot = yPeak > 1 ? ((yPeak - 1) * 100) : 0;

  let settlingTime = null;
  const settlingTarget = yFinal;
  const band = 0.02 * Math.max(1, Math.abs(settlingTarget));
  for (let i = 0; i < finitePairs.length; i += 1) {
    const [t] = finitePairs[i];
    let remainsInside = true;
    for (let j = i; j < finitePairs.length; j += 1) {
      if (Math.abs(finitePairs[j][1] - settlingTarget) > band) {
        remainsInside = false;
        break;
      }
    }
    if (remainsInside) {
      settlingTime = t;
      break;
    }
  }

  return {
    riseTime,
    overshoot,
    settlingTime,
    sse: absFinalError
  };
}

// Format a numeric metric for display; returns "-" when value is unavailable.
function formatMetric(value, unit = "", digits = 3) {
  if (value === null || !Number.isFinite(value)) return "-";
  return `${value.toFixed(digits)}${unit}`;
}

// ─── Damping ratio ───────────────────────────────────────────────────────────

// Solve a3·s³ + a2·s² + a1·s + a0 = 0 analytically using Cardano's formula.
// Returns an array of up to three roots as {re, im} objects.
function solveCubicRoots(a3, a2, a1, a0) {
  if (Math.abs(a3) < 1e-12) return [];

  const a = a2 / a3;
  const b = a1 / a3;
  const c = a0 / a3;

  const p = b - (a * a) / 3;
  const q = (2 * a * a * a) / 27 - (a * b) / 3 + c;
  const disc = (q * q) / 4 + (p * p * p) / 27;
  const shift = a / 3;
  const roots = [];

  if (disc > 1e-12) {
    const sqrtDisc = Math.sqrt(disc);
    const u = Math.cbrt(-q / 2 + sqrtDisc);
    const v = Math.cbrt(-q / 2 - sqrtDisc);
    const t = u + v;

    roots.push({ re: t - shift, im: 0 });
    roots.push({ re: -(t / 2) - shift, im: (Math.sqrt(3) / 2) * (u - v) });
    roots.push({ re: -(t / 2) - shift, im: -(Math.sqrt(3) / 2) * (u - v) });
  } else if (Math.abs(disc) <= 1e-12) {
    const u = Math.cbrt(-q / 2);
    roots.push({ re: 2 * u - shift, im: 0 });
    roots.push({ re: -u - shift, im: 0 });
    roots.push({ re: -u - shift, im: 0 });
  } else {
    const r = Math.sqrt(-(p * p * p) / 27);
    const phi = Math.acos((-q / 2) / r);
    const m = 2 * Math.sqrt(-p / 3);

    roots.push({ re: m * Math.cos(phi / 3) - shift, im: 0 });
    roots.push({ re: m * Math.cos((phi + 2 * Math.PI) / 3) - shift, im: 0 });
    roots.push({ re: m * Math.cos((phi + 4 * Math.PI) / 3) - shift, im: 0 });
  }

  return roots;
}

// Compute the closed-loop damping ratio ζ from the characteristic polynomial.
//
// With unity feedback and a parallel PID controller the closed-loop
// characteristic equation is:
//   m·s³ + (c+Kd)·s² + (k+Kp)·s + Ki = 0
//
// Strategy:
//   1. Solve the cubic analytically.
//   2. If dominant poles are complex  → ζ = −σ / |s|  (standard definition).
//   3. If all dominant poles are real → use the two-real-pole equivalent
//      ζ = (a+b) / (2√(ab))  where a, b are the pole magnitudes.
//   4. Sign convention: ζ < 0 for unstable systems (positive-real-part poles).
function computeClosedLoopDampingRatio(params) {
  // Closed-loop denominator for unity feedback with PID on m*s^2 + c*s + k is:
  // m*s^3 + (c + Kd)*s^2 + (k + Kp)*s + Ki = 0
  const roots = solveCubicRoots(params.m, params.c + params.kd, params.k + params.kp, params.ki);
  if (roots.length < 2) return null;

  const eps = 1e-6;
  const finiteRoots = roots.filter((r) => Number.isFinite(r.re) && Number.isFinite(r.im));
  if (finiteRoots.length < 2) return null;

  const unstable = finiteRoots.some((r) => r.re > eps);

  let bestComplex = null;
  const complexRoots = finiteRoots
    .filter((r) => Math.abs(r.im) > eps)
    .sort((a, b) => b.re - a.re);
  for (let i = 0; i < complexRoots.length; i += 1) {
    const r = complexRoots[i];
    if (Math.abs(r.im) <= eps) continue;
    if (bestComplex === null || r.re > bestComplex.re) bestComplex = r;
  }

  if (bestComplex !== null) {
    return -bestComplex.re / Math.hypot(bestComplex.re, bestComplex.im);
  }

  const realRoots = finiteRoots
    .filter((r) => Math.abs(r.im) <= eps && Math.abs(r.re) > eps)
    .sort((a, b) => b.re - a.re);
  if (realRoots.length < 2) return null;

  const a = Math.abs(realRoots[0].re);
  const b = Math.abs(realRoots[1].re);
  if (a <= eps || b <= eps) return null;

  const zeta = (a + b) / (2 * Math.sqrt(a * b));
  return unstable ? -zeta : zeta;
}

// Map a damping ratio to a human-readable category label.
//   ζ < 0          → unstable
//   0 ≤ ζ ≤ 0.005  → undamped  (near-zero tolerance)
//   |ζ−1| ≤ 0.02   → critically damped
//   ζ < 1          → underdamped
//   ζ > 1          → overdamped
function classifyDampingType(zeta) {
  if (!Number.isFinite(zeta)) return "-";

  const zeroTol = 0.005;
  const critTol = 0.02;

  if (zeta < 0) return "unstable";
  if (zeta >= 0 && zeta <= zeroTol) return "undamped";
  if (Math.abs(zeta - 1) <= critTol) return "critically damped";
  if (zeta < 1) return "underdamped";
  return "overdamped";
}

// ─── Canvas chart ────────────────────────────────────────────────────────────

// Draw the step response chart on the canvas element.
// Renders a grid, the unit step reference, and the output response y(t).
// Y-axis range is either auto-scaled to the data or fixed (see fixedYAxis).
function drawPlot(time, ref, y, autoScaleY) {
  const dpr = window.devicePixelRatio || 1;
  const cssWidth = chart.clientWidth;
  const cssHeight = chart.clientHeight;
  chart.width = Math.floor(cssWidth * dpr);
  chart.height = Math.floor(cssHeight * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

  const w = cssWidth;
  const h = cssHeight;

  ctx.clearRect(0, 0, w, h);

  const pad = { left: 56, right: 16, top: 16, bottom: 36 };
  const plotW = w - pad.left - pad.right;
  const plotH = h - pad.top - pad.bottom;

  const tMin = time[0] || -1;
  const tMax = time[time.length - 1] || 1;

  let yMin;
  let yMax;
  if (autoScaleY) {
    yMin = 0;
    yMax = 1;
    for (let i = 0; i < y.length; i += 1) {
      const yi = y[i];
      if (!Number.isFinite(yi)) continue;
      if (yi < yMin) yMin = yi;
      if (yi > yMax) yMax = yi;
    }
    const margin = 0.08 * Math.max(1, yMax - yMin);
    yMin -= margin;
    yMax += margin;
  } else {
    yMin = fixedYAxis.min;
    yMax = fixedYAxis.max;
  }

  const tx = (t) => pad.left + ((t - tMin) / (tMax - tMin)) * plotW;
  const ty = (val) => pad.top + (1 - (val - yMin) / (yMax - yMin || 1)) * plotH;

  ctx.strokeStyle = "rgba(31, 42, 31, 0.12)";
  ctx.lineWidth = 1;
  const gx = 8;
  const gy = 6;
  for (let i = 0; i <= gx; i += 1) {
    const x = pad.left + (i / gx) * plotW;
    ctx.beginPath();
    ctx.moveTo(x, pad.top);
    ctx.lineTo(x, pad.top + plotH);
    ctx.stroke();
  }
  for (let i = 0; i <= gy; i += 1) {
    const yLine = pad.top + (i / gy) * plotH;
    ctx.beginPath();
    ctx.moveTo(pad.left, yLine);
    ctx.lineTo(pad.left + plotW, yLine);
    ctx.stroke();
  }

  ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue("--line-ref").trim();
  ctx.lineWidth = 1.8;
  ctx.beginPath();
  for (let i = 0; i < time.length; i += 1) {
    const x = tx(time[i]);
    const yv = ty(ref[i]);
    if (i === 0) ctx.moveTo(x, yv);
    else ctx.lineTo(x, yv);
  }
  ctx.stroke();

  ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue("--line-out").trim();
  ctx.lineWidth = 2.2;
  ctx.beginPath();
  let started = false;
  for (let i = 0; i < time.length; i += 1) {
    if (!Number.isFinite(y[i])) continue;
    const x = tx(time[i]);
    const yv = ty(y[i]);
    if (!started) {
      ctx.moveTo(x, yv);
      started = true;
    } else {
      ctx.lineTo(x, yv);
    }
  }
  ctx.stroke();

  ctx.strokeStyle = "rgba(31, 42, 31, 0.5)";
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  ctx.moveTo(pad.left, pad.top + plotH);
  ctx.lineTo(pad.left + plotW, pad.top + plotH);
  ctx.moveTo(pad.left, pad.top);
  ctx.lineTo(pad.left, pad.top + plotH);
  ctx.stroke();

  ctx.fillStyle = "#2f3f2f";
  ctx.font = "12px Segoe UI";
  ctx.fillText("time (s)", pad.left + plotW / 2 - 18, h - 10);
  ctx.save();
  ctx.translate(14, pad.top + plotH / 2 + 18);
  ctx.rotate(-Math.PI / 2);
  ctx.fillText("output y(t)", 0, 0);
  ctx.restore();

  ctx.font = "11px Segoe UI";
  ctx.fillStyle = "#455545";
  for (let i = 0; i <= 4; i += 1) {
    const tt = tMin + (i / 4) * (tMax - tMin);
    const x = tx(tt);
    ctx.fillText(tt.toFixed(1), x - 8, pad.top + plotH + 16);
  }
  ctx.textAlign = "right";
  for (let i = 0; i <= 4; i += 1) {
    const yy = yMin + (i / 4) * (yMax - yMin);
    const yPx = ty(yy);
    ctx.fillText(yy.toFixed(2), pad.left - 6, yPx + 4);
  }
  ctx.textAlign = "left";
}

// ─── Update loop ─────────────────────────────────────────────────────────────

// Debounce rapid slider input by coalescing redraws into a single animation frame.
let rafHandle = 0;
function scheduleUpdate() {
  if (rafHandle) return;
  rafHandle = requestAnimationFrame(() => {
    rafHandle = 0;
    updateAll();
  });
}

// Full update pipeline: read parameters → update labels → simulate → draw → metrics.
function updateAll() {
  const p = getParams();
  updateValueLabels(p);
  const sim = simulateStepResponse(p);
  drawPlot(sim.time, sim.ref, sim.y, autoScaleYEl.checked);
  const zeta = computeClosedLoopDampingRatio(p);

  const m = computeMetrics(sim.time, sim.y);
  metricsEl.rise.textContent = formatMetric(m.riseTime, " s");
  metricsEl.overshoot.textContent = formatMetric(m.overshoot, " %", 2);
  metricsEl.settling.textContent = formatMetric(m.settlingTime, " s");
  metricsEl.sse.textContent = formatMetric(m.sse, "", 4);
  metricsEl.dampingRatio.textContent = formatMetric(zeta, "", 3);
  metricsEl.dampingType.textContent = classifyDampingType(zeta);

  if (!sim.stable) {
    statusEl.textContent = "Unstable / divergent response detected. Reduce gains or increase damping.";
  } else {
    statusEl.textContent = "";
  }
}

// Set multiple sliders at once from a plain object, then re-run the update pipeline.
function setFromObject(values) {
  for (const key of Object.keys(values)) {
    if (sliders[key]) sliders[key].value = values[key];
  }
  updateAll();
}

// ─── Event wiring ────────────────────────────────────────────────────────────

document.getElementById("resetBtn").addEventListener("click", () => setFromObject(defaults));

// Trigger update on both input (while dragging) and change (on release) so the
// chart stays live whether the user drags or steps with keyboard arrows.
Object.values(sliders).forEach((slider) => {
  slider.addEventListener("input", scheduleUpdate);
  slider.addEventListener("change", scheduleUpdate);
});

autoScaleYEl.addEventListener("change", scheduleUpdate);

window.addEventListener("resize", scheduleUpdate);

updateAll();
