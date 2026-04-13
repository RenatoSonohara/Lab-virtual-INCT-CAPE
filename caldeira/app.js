/**
 * ============================
 * Simulador Caldeira — UFSC
 * EDOs
 *
 * ENTRADAS:  W, St, Q
 * SAÍDAS:    L = Lf + Ls + LQ  |  P = Pf + Ps + PQ
 * ============================
 */

class CaldeiraJS {
  constructor(opts) {
    const { W, St, Q, refL, refP, consts, pid } = opts;

    this.DELTA_T = consts.DELTA_T;

    this.W  = W;
    this.St = St;
    this.Q  = Q;
    this.refL = refL;
    this.refP = refP;

    this.Lf = 0; this.Lfd = 0;
    this.Ls = 0; this.Lsd = 0;
    this.LQ = 0; this.LQd = 0;

    this._StPrev = St;
    this._QPrev  = Q;

    this.Pf = 0;
    this.Ps = 0;
    this.PQ = 0;

    this.Kp = pid.kp;
    this.Ki = pid.ki;
    this.Kd = pid.kd;
    this.acaoIntegral = 0;
    this._lastP = 0;
    this.sinalControle = 0;
  }

  controlePressao(P) {
    const uMin = 0, uMax = 100;
    const erro  = this.refP - P;
    const dMeas = (P - this._lastP) / this.DELTA_T;
    this._lastP = P;

    this.acaoIntegral += this.Ki * erro * this.DELTA_T;
    this.acaoIntegral  = Math.max(uMin, Math.min(uMax, this.acaoIntegral));

    let u = this.Kp * erro + this.acaoIntegral - this.Kd * dMeas;
    u = Math.max(uMin, Math.min(uMax, u));

    this.sinalControle = u;
    this.Q = (u / 100) * 500000;
    return erro;
  }

  stepOnce() {
    const dt = this.DELTA_T;
    const W  = this.W;
    const St = this.St;
    const Q  = this.Q;

    const Std = (St - this._StPrev) / dt;
    const Qd  = (Q  - this._QPrev)  / dt;

    const Lf_dd = (-1 / 8.666) * this.Lfd
                + (0.000134 / 8.666) * W;

    const Ls_dd = (-1 / 1.755) * this.Lsd
                + (0.03932 / 1.755) * Std
                - (0.0001515 / 1.755) * St;

    const LQ_dd = (-1 / 7.878) * this.LQd
                - (5e-9 / 7.878) * Qd
                - (1.15e-11 / 7.878) * Q;

    this.Lfd += Lf_dd * dt;  this.Lf += this.Lfd * dt;
    this.Lsd += Ls_dd * dt;  this.Ls += this.Lsd * dt;
    this.LQd += LQ_dd * dt;  this.LQ += this.LQd * dt;

    this.Pf += (-311    * W)  * dt;
    this.Ps += (-1338   * St) * dt;
    this.PQ += (0.0006765 * Q)  * dt;

    const L = this.Lf + this.Ls + this.LQ;
    const P = this.Pf + this.Ps + this.PQ;

    this._StPrev = St;
    this._QPrev  = Q;

    return {
      L, P,
      Lf: this.Lf, Ls: this.Ls, LQ: this.LQ,
      Pf: this.Pf, Ps: this.Ps, PQ: this.PQ,
      W, St, Q,
      sinalControle: this.sinalControle,
      erroP: this.refP - P,
      erroL: this.refL - L,
    };
  }

  setW(v)    { this.W  = v; }
  setSt(v)   { this.St = v; }
  setQ(v)    { this.Q  = v; }
  setRefL(v) { this.refL = v; }
  setRefP(v) { this.refP = v; }
  setGains({kp, ki, kd}) {
    if (kp !== undefined) this.Kp = kp;
    if (ki !== undefined) this.Ki = ki;
    if (kd !== undefined) this.Kd = kd;
  }
}

// ============================================================
// UTILITÁRIOS
// ============================================================
function q(s) { return document.querySelector(s); }
function fmt(x, d=4) { return Number(x).toFixed(d); }

// ============================================================
// CONSTANTES — passo fixo 0.1 s, gráfico a cada 1 s
// ============================================================
const C = { DELTA_T: 0.1 };
const STEPS_PER_POINT = 10; // 0.1 * 10 = 1 s por ponto no gráfico

// ============================================================
// DOM
// ============================================================
const els = {
  status: q('#statusMsg'),

  W:    q('#W'),    St:   q('#St'),   Q:    q('#Q'),
  refP: q('#refP'), refL: q('#refL'),
  kp:   q('#kp'),   ki:   q('#ki'),   kd:   q('#kd'),

  W_txt:    q('#W_txt'),    St_txt:   q('#St_txt'),   Q_txt:    q('#Q_txt'),
  refP_txt: q('#refP_txt'), refL_txt: q('#refL_txt'),
  kp_txt:   q('#kp_txt'),   ki_txt:   q('#ki_txt'),   kd_txt:   q('#kd_txt'),

  start:       q('#startBtn'),
  reset:       q('#resetBtn'),
  runBatch:    q('#runBatchBtn'),
  simDuration: q('#simDuration'),

  kpi_L:    q('#kpi_L'),
  kpi_P:    q('#kpi_P'),
  kpi_ctrl: q('#kpi_ctrl'),
  kpi_refP: q('#kpi_refP'),

  chartL:        q('#chartL'),
  chartP:        q('#chartP'),
  chartEntradas: q('#chartEntradas'),

  toast: q('#toast'),
};

// Valores padrão para reset completo
const DEFAULTS = {
  W: 1, St: 1, Q: 1, refP: 0, refL: 0, kp: 0, ki: 0, kd: 0,
};

// ============================================================
// SINCRONIZAÇÃO SLIDER <-> TEXTO
// ============================================================
const pares = [
  [els.W,    els.W_txt],
  [els.St,   els.St_txt],
  [els.Q,    els.Q_txt],
  [els.refP, els.refP_txt],
  [els.refL, els.refL_txt],
  [els.kp,   els.kp_txt],
  [els.ki,   els.ki_txt],
  [els.kd,   els.kd_txt],
];

pares.forEach(([slider, txt]) => {
  slider.addEventListener('input', () => { txt.value = slider.value; });
  const syncToSlider = () => {
    const v = Number(txt.value);
    if (!Number.isFinite(v)) return;
    const min = Number(slider.min), max = Number(slider.max);
    slider.value = Math.max(min, Math.min(max, v));
    txt.value = slider.value;
  };
  txt.addEventListener('change', syncToSlider);
  txt.addEventListener('keydown', (e) => { if (e.key === 'Enter') syncToSlider(); });
});

// ============================================================
// UI
// ============================================================
function uiRunning(running) {
  els.start.disabled = false;
  els.reset.disabled = running;
  els.runBatch.disabled = running;

  els.start.textContent = running ? 'Parar' : 'Iniciar';
  els.start.classList.toggle('btn-danger', running);
  els.start.classList.toggle('btn-primary', !running);
}

function updateKPIs(s) {
  els.kpi_L.textContent    = fmt(s.L, 6) + ' m';
  els.kpi_P.textContent    = fmt(s.P, 4) + ' Pa';
  els.kpi_ctrl.textContent = fmt(s.sinalControle, 1) + ' %';
  els.kpi_refP.textContent = fmt(Number(els.refP.value), 2);
}

function resetKPIs() {
  ['kpi_L','kpi_P','kpi_ctrl','kpi_refP'].forEach(id => els[id].textContent = '—');
}

// ============================================================
// CHARTS
// ============================================================
function newChart(canvas, yLabel, labels) {
  return new Chart(canvas.getContext('2d'), {
    type: 'line',
    data: {
      labels: [],
      datasets: labels.map((l, i) => ({
        label: l,
        data: [],
        borderWidth: 2,
        tension: 0.22,
        pointRadius: 0,
        pointHoverRadius: 3,
        borderDash: l.startsWith('Ref') ? [5, 5] : [],
        // Componentes (índices 1, 2, 3) desativados por padrão
        hidden: (i >= 1 && i <= 3),
      }))
    },
    options: {
      animation: false, responsive: true,
      scales: {
        x: { title: { display: true, text: 'Tempo (s)' } },
        y: { title: { display: true, text: yLabel } }
      },
      plugins: {
        legend: { position: 'bottom' },
        tooltip: { mode: 'nearest', intersect: false }
      }
    }
  });
}

function initCharts() {
  Object.values(charts).forEach(c => c?.destroy?.());

  charts.L = newChart(els.chartL, 'Nível (m)',
    ['L total', 'Lf (por W)', 'Ls (por St)', 'LQ (por Q)', 'Ref. Nível']);

  charts.P = newChart(els.chartP, 'Pressão (Pa)',
    ['P total', 'Pf (por W)', 'Ps (por St)', 'PQ (por Q)', 'Ref. Pressão']);

  charts.ent = newChart(els.chartEntradas, 'Entradas',
    ['W (kg/s)', 'St (kg/s)', 'Q (kW)']);
}

function pushPoint(chart, lbl, arrs) {
  chart.data.labels.push(lbl);
  chart.data.datasets.forEach((ds, i) => ds.data.push(arrs[i]));
  while (chart.data.labels.length > MAX_POINTS) {
    chart.data.labels.shift();
    chart.data.datasets.forEach(ds => ds.data.shift());
  }
  chart.update('quiet');
}

function _pushAllCharts(tSec, s, refL, refP) {
  pushPoint(charts.L,   tSec, [s.L,  s.Lf, s.Ls, s.LQ, refL]);
  pushPoint(charts.P,   tSec, [s.P,  s.Pf, s.Ps, s.PQ, refP]);
  pushPoint(charts.ent, tSec, [s.W,  s.St, s.Q / 1000]);
}

// ============================================================
// ESTADO GLOBAL
// ============================================================
let charts = {};
let timer = null, sim = null;
const MAX_POINTS = 1200;
let _stepCount = 0;

// ============================================================
// INSTÂNCIA
// ============================================================
function createSim() {
  return new CaldeiraJS({
    W:    Number(els.W.value),
    St:   Number(els.St.value),
    Q:    Number(els.Q.value) * 1000,
    refL: Number(els.refL.value),
    refP: Number(els.refP.value),
    pid:  { kp: Number(els.kp.value), ki: Number(els.ki.value), kd: Number(els.kd.value) },
    consts: { ...C },
  });
}

function pidAtivo() {
  return Number(els.kp.value) > 0 || Number(els.ki.value) > 0 || Number(els.kd.value) > 0;
}

// ============================================================
// TEMPO REAL
// ============================================================
function startRealtime() {
  if (timer) stopRealtime();
  els.status.textContent = '▶️ Executando';
  initCharts();
  _stepCount = 0;
  sim = createSim();
  uiRunning(true);
  // Com Δt=0.1s, roda ~160ms de simulação por tick de 16ms (10 passos/tick ≈ tempo real)
  const intervalMs = 16;
  const stepsPerInterval = Math.max(1, Math.round(intervalMs / (C.DELTA_T * 1000)));
  timer = setInterval(() => tickRealtime(stepsPerInterval), intervalMs);
}

function stopRealtime() {
  if (timer) clearInterval(timer);
  timer = null; sim = null;
  uiRunning(false);
  els.status.textContent = '⏹️ Parado.';
}

function tickRealtime(stepsPerInterval) {
  if (!sim) return;

  sim.setW(Number(els.W.value));
  sim.setSt(Number(els.St.value));
  sim.setQ(Number(els.Q.value) * 1000);
  sim.setRefL(Number(els.refL.value));
  sim.setRefP(Number(els.refP.value));
  sim.setGains({ kp: Number(els.kp.value), ki: Number(els.ki.value), kd: Number(els.kd.value) });

  let lastS;
  for (let i = 0; i < stepsPerInterval; i++) {
    lastS = sim.stepOnce();
    if (pidAtivo()) sim.controlePressao(lastS.P);
    _stepCount++;
    if (_stepCount % STEPS_PER_POINT === 0) {
      const tSec = parseFloat((_stepCount * C.DELTA_T).toFixed(1));
      _pushAllCharts(tSec, lastS, sim.refL, sim.refP);
    }
  }

  updateKPIs(lastS);
  updateViz(lastS);
}

// ============================================================
// MODO TEMPO SIMULADO (batch)
// ============================================================
function runBatch() {
  const durS = Math.max(1, Number(els.simDuration.value) || 100);
  const totalSteps = Math.round(durS / C.DELTA_T);

  els.status.textContent = '⏳ Calculando…';
  initCharts();
  resetKPIs();
  _stepCount = 0;

  const bSim = createSim();
  let lastS;

  for (let i = 0; i < totalSteps; i++) {
    lastS = bSim.stepOnce();
    if (pidAtivo()) bSim.controlePressao(lastS.P);
    _stepCount++;
    if (_stepCount % STEPS_PER_POINT === 0) {
      const tSec = parseFloat((_stepCount * C.DELTA_T).toFixed(1));
      _pushAllCharts(tSec, lastS, bSim.refL, bSim.refP);
    }
  }

  updateKPIs(lastS);
  updateViz(lastS);
  els.status.textContent = `✅ Simulado: ${durS} s`;
}

// ============================================================
// RESET COMPLETO
// ============================================================
function fullReset() {
  if (timer) stopRealtime();
  initCharts();
  resetKPIs();
  _stepCount = 0;
  _vizL = 0; _vizP = 0;
  els.status.textContent = 'Pronto';
  uiRunning(false);

  // Restaurar sliders e textos
  const sliderMap = { W: els.W, St: els.St, Q: els.Q, refP: els.refP, refL: els.refL, kp: els.kp, ki: els.ki, kd: els.kd };
  const txtMap    = { W: els.W_txt, St: els.St_txt, Q: els.Q_txt, refP: els.refP_txt, refL: els.refL_txt, kp: els.kp_txt, ki: els.ki_txt, kd: els.kd_txt };
  Object.keys(DEFAULTS).forEach(k => {
    sliderMap[k].value = DEFAULTS[k];
    txtMap[k].value    = DEFAULTS[k];
  });

  // Resetar visualização
  updateViz({ L: 0, P: 0, Lf: 0, Ls: 0, LQ: 0 });
}

// ============================================================
// VISUALIZAÇÃO DINÂMICA DA CALDEIRA
// ============================================================
const vizWaterFill = document.getElementById('waterFill');
const vizSteamFill = document.getElementById('steamFill');
const vizWaterWave = document.getElementById('waterWave');
const vizPressure  = document.getElementById('vizPressure');

let _vizL = 0, _vizP = 0;

function updateViz(s) {
  const drumTop    = 67;
  const drumBottom = 303;
  const drumH      = drumBottom - drumTop;

  const Lmin = -0.05, Lmax = 0.05;
  const ratio = Math.max(0, Math.min(1, (s.L - Lmin) / (Lmax - Lmin)));
  _vizL += (ratio - _vizL) * 0.15;

  const waterY = drumBottom - _vizL * drumH;
  const waterH = drumBottom - waterY;
  const steamH = waterY - drumTop;

  if (vizWaterFill) {
    vizWaterFill.setAttribute('y', waterY);
    vizWaterFill.setAttribute('height', Math.max(0, waterH));
  }
  if (vizSteamFill) {
    vizSteamFill.setAttribute('y', drumTop);
    vizSteamFill.setAttribute('height', Math.max(0, steamH));
  }

  if (vizWaterWave) {
    const wy = waterY;
    const amp = 4;
    vizWaterWave.setAttribute('d',
      `M122,${wy} Q152,${wy-amp} 182,${wy} Q212,${wy+amp} 242,${wy} ` +
      `Q272,${wy-amp} 302,${wy} Q332,${wy+amp} 358,${wy} ` +
      `L358,${wy} L122,${wy} Z`
    );
  }

  _vizP += (s.P - _vizP) * 0.2;
  if (vizPressure) {
    const pStr = _vizP.toFixed(4);
    if (vizPressure.textContent !== pStr) {
      vizPressure.textContent = pStr;
      vizPressure.style.animation = 'none';
      void vizPressure.offsetWidth;
      vizPressure.style.animation = 'pressurePulse 0.3s ease';
    }
  }
}

// ============================================================
// EVENTOS
// ============================================================
els.start.addEventListener('click', () => {
  timer ? stopRealtime() : startRealtime();
});

els.reset.addEventListener('click', fullReset);

els.runBatch.addEventListener('click', () => {
  if (timer) stopRealtime();
  runBatch();
});

// ============================================================
// TOAST & BOOT
// ============================================================
document.querySelectorAll('[data-coming-soon]').forEach(el =>
  el.addEventListener('click', e => { e.preventDefault(); showToast('Em breve…'); })
);
function showToast(msg) {
  els.toast.textContent = msg;
  els.toast.classList.remove('hidden');
  setTimeout(() => els.toast.classList.add('hidden'), 1600);
}

function boot() {
  initCharts();
}
boot();
