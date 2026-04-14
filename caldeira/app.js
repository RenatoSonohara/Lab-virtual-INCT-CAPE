/**
 * ============================
 * Simulador Caldeira — UFSC
 * EDOs com PRÉ-COMPENSADOR DESACOPLANTE + 2 PIDs
 *
 * ENTRADAS (originais):  W, St, Q
 * SAÍDAS:    L = Lf + Ls + LQ  |  P = Pf + Ps + PQ
 * CONTROLE:  PID_L controla L via u1_pid
 *            PID_P controla P via u2_pid
 *            DESACOPLADOR transforma [u1_pid, u2_pid] → [W, Q]
 *            com compensação do acoplamento de W e da perturbação St em P
 * ============================
 */

class CaldeiraJS {
  constructor(opts) {
    const { W, St, Q, refL, refP, consts, pid1, pid2 } = opts;

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

    // ──── PID 1: Controla NÍVEL L ────
    this.Kp1 = pid1.kp;
    this.Ki1 = pid1.ki;
    this.Kd1 = pid1.kd;
    this.acaoIntegral1 = 0;
    this._lastL = 0;
    this.sinalControle1 = 0; // u1_pid antes do desacoplador

    // ──── PID 2: Controla PRESSÃO P ────
    this.Kp2 = pid2.kp;
    this.Ki2 = pid2.ki;
    this.Kd2 = pid2.kd;
    this.acaoIntegral2 = 0;
    this._lastP = 0;
    this.sinalControle2 = 0; // u2_pid antes do desacoplador

    // ──── PRÉ-COMPENSADOR DESACOPLANTE ────
    // Dinâmica de pressão: dP/dt = -311*W -1338*St + 0.0006765*Q
    // Escolha: u2_pid representa a contribuição desejada em dP/dt (Pa/s).
    // Logo: Q = (u2_pid + 311*W + 1338*St) / 0.0006765
    // Isso cancela o efeito de W e St sobre P e deixa u2_pid governar P.
    this.desacoplador = {
      gPw: -311,
      gPs: -1338,
      gPq: 0.0006765,
    };

    // Saídas do desacoplador
    this.u1_apos_desacoplador = 0;
    this.u2_apos_desacoplador = 0;
  }

  // ──── PID 1: Controla NÍVEL ────
  controleNivel(L) {
    const uMin = -50, uMax = 50; // Limites do sinal PID
    const erro = this.refL - L;
    const dMeas = (L - this._lastL) / this.DELTA_T;
    this._lastL = L;

    this.acaoIntegral1 += this.Ki1 * erro * this.DELTA_T;
    this.acaoIntegral1 = Math.max(uMin, Math.min(uMax, this.acaoIntegral1));

    let u = this.Kp1 * erro + this.acaoIntegral1 - this.Kd1 * dMeas;
    u = Math.max(uMin, Math.min(uMax, u));

    this.sinalControle1 = u;
    return erro;
  }

  // ──── PID 2: Controla PRESSÃO ────
  controlePressao(P) {
    // u2_pid e tratado como termo desejado de dP/dt (Pa/s)
    const uMin = -500, uMax = 500;
    const erro = this.refP - P;
    const dMeas = (P - this._lastP) / this.DELTA_T;
    this._lastP = P;

    this.acaoIntegral2 += this.Ki2 * erro * this.DELTA_T;
    this.acaoIntegral2 = Math.max(uMin, Math.min(uMax, this.acaoIntegral2));

    let u = this.Kp2 * erro + this.acaoIntegral2 - this.Kd2 * dMeas;
    u = Math.max(uMin, Math.min(uMax, u));

    this.sinalControle2 = u;
    return erro;
  }

  // ──── DESACOPLADOR ────
  // Aplica desacoplamento [u1_pid, u2_pid] → [W_planta, Q_planta]
  aplicarDesacoplador() {
    const u1 = this.sinalControle1;
    const u2 = this.sinalControle2;
    const { gPw, gPs, gPq } = this.desacoplador;

    // p_c1 atua em W
    const p_c1 = u1;
    // p_c2 atua em Q com compensação de acoplamento de W e perturbação St
    const p_c2 = (u2 - gPw * p_c1 - gPs * this.St) / gPq;

    // Saturação das variáveis manipuladas para manter coerência com a UI
    this.u1_apos_desacoplador = Math.max(0, Math.min(100, p_c1));
    this.u2_apos_desacoplador = Math.max(0, Math.min(500000, p_c2));

    // Atualizar as entradas da planta
    this.W = this.u1_apos_desacoplador;
    this.Q = this.u2_apos_desacoplador;
  }

  // ──── RESETAR PIDs ────
  resetPIDs() {
    this.acaoIntegral1 = 0;
    this._lastL = 0;
    this.sinalControle1 = 0;
    
    this.acaoIntegral2 = 0;
    this._lastP = 0;
    this.sinalControle2 = 0;
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
      sinalControle1: this.sinalControle1,
      sinalControle2: this.sinalControle2,
      u1_apos_desacoplador: this.u1_apos_desacoplador,
      u2_apos_desacoplador: this.u2_apos_desacoplador,
      p_c1: this.u1_apos_desacoplador,
      p_c2: this.u2_apos_desacoplador,
      erroL: this.refL - L,
      erroP: this.refP - P,
    };
  }

  setW(v)    { this.W  = v; }
  setSt(v)   { this.St = v; }
  setQ(v)    { this.Q  = v; }
  setRefL(v) { this.refL = v; }
  setRefP(v) { this.refP = v; }
  setGains1({kp, ki, kd}) {
    if (kp !== undefined) this.Kp1 = kp;
    if (ki !== undefined) this.Ki1 = ki;
    if (kd !== undefined) this.Kd1 = kd;
  }
  setGains2({kp, ki, kd}) {
    if (kp !== undefined) this.Kp2 = kp;
    if (ki !== undefined) this.Ki2 = ki;
    if (kd !== undefined) this.Kd2 = kd;
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
  
  // PID 1 (Nível L)
  kp1:   q('#kp1'),   ki1:   q('#ki1'),   kd1:   q('#kd1'),
  
  // PID 2 (Pressão P)
  kp2:   q('#kp2'),   ki2:   q('#ki2'),   kd2:   q('#kd2'),

  W_txt:    q('#W_txt'),    St_txt:   q('#St_txt'),   Q_txt:    q('#Q_txt'),
  refP_txt: q('#refP_txt'), refL_txt: q('#refL_txt'),
  
  // PID 1 text
  kp1_txt:   q('#kp1_txt'),   ki1_txt:   q('#ki1_txt'),   kd1_txt:   q('#kd1_txt'),
  
  // PID 2 text
  kp2_txt:   q('#kp2_txt'),   ki2_txt:   q('#ki2_txt'),   kd2_txt:   q('#kd2_txt'),

  loopModeToggle: q('#loopModeToggle'),
  loopModeLabel:  q('#loopModeLabel'),
  applyIdealGains: q('#applyIdealGainsBtn'),

  start:       q('#startBtn'),
  reset:       q('#resetBtn'),
  runBatch:    q('#runBatchBtn'),
  cancelBatch: q('#cancelBatchBtn'),
  simDuration: q('#simDuration'),

  kpi_L:    q('#kpi_L'),
  kpi_P:    q('#kpi_P'),
  kpi_u1:   q('#kpi_u1'),
  kpi_u2:   q('#kpi_u2'),
  kpi_refP: q('#kpi_refP'),
  kpi_refL: q('#kpi_refL'),

  chartL:        q('#chartL'),
  chartP:        q('#chartP'),
  chartEntradas: q('#chartEntradas'),
  chartControles: q('#chartControles'),

  toast: q('#toast'),
};

// Valores padrão para reset completo
const DEFAULTS = {
  W: 1, St: 1, Q: 1, refP: 0, refL: 0, 
  kp1: 0, ki1: 0, kd1: 0,
  kp2: 0, ki2: 0, kd2: 0,
};

// Ganhos sugeridos (conservadores) para plantas com possível fase não mínima.
const IDEAL_GAINS = {
  kp1: 1.2,  ki1: 0.04, kd1: 0.35, // PID 1: L -> W
  kp2: 0.22, ki2: 0.01, kd2: 0.08, // PID 2: P -> Q
};

function applyIdealGains() {
  const maps = [
    ['kp1', els.kp1, els.kp1_txt], ['ki1', els.ki1, els.ki1_txt], ['kd1', els.kd1, els.kd1_txt],
    ['kp2', els.kp2, els.kp2_txt], ['ki2', els.ki2, els.ki2_txt], ['kd2', els.kd2, els.kd2_txt],
  ];

  maps.forEach(([k, slider, txt]) => {
    const v = IDEAL_GAINS[k];
    if (slider) slider.value = v;
    if (txt) txt.value = v;
  });

  setLoopMode(true);
  if (sim) {
    sim.setGains1({ kp: IDEAL_GAINS.kp1, ki: IDEAL_GAINS.ki1, kd: IDEAL_GAINS.kd1 });
    sim.setGains2({ kp: IDEAL_GAINS.kp2, ki: IDEAL_GAINS.ki2, kd: IDEAL_GAINS.kd2 });
    sim.resetPIDs();
  }
  els.status.textContent = 'Ganhos conservadores aplicados (MF).';
}

// ============================================================
// SINCRONIZAÇÃO SLIDER <-> TEXTO
// ============================================================
const pares = [
  [els.W,    els.W_txt],
  [els.St,   els.St_txt],
  [els.Q,    els.Q_txt],
  [els.refP, els.refP_txt],
  [els.refL, els.refL_txt],
];

pares.forEach(([slider, txt]) => {
  if (!slider || !txt) return;
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

// Sincronização dos ganhos dos PIDs
const gainPairs = [
  [els.kp1, els.kp1_txt],
  [els.ki1, els.ki1_txt],
  [els.kd1, els.kd1_txt],
  [els.kp2, els.kp2_txt],
  [els.ki2, els.ki2_txt],
  [els.kd2, els.kd2_txt],
];

gainPairs.forEach(([slider, txt]) => {
  if (!slider || !txt) return;
  slider.addEventListener('input', () => { txt.value = slider.value; });
  const syncGainSliderOnly = () => {
    const v = Number(txt.value);
    if (!Number.isFinite(v)) return;
    const clamped = Math.max(Number(slider.min), Math.min(Number(slider.max), v));
    slider.value = clamped;
  };
  txt.addEventListener('input', syncGainSliderOnly);
  txt.addEventListener('change', syncGainSliderOnly);
  txt.addEventListener('keydown', (e) => { if (e.key === 'Enter') syncGainSliderOnly(); });
});

function gainFromText(txtEl, sliderEl) {
  const v = Number(txtEl?.value);
  if (Number.isFinite(v)) return v;
  return Number(sliderEl?.value ?? 0);
}

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

function setRunBatchLoading(loading) {
  if (!els.runBatch) return;
  if (loading) {
    if (!els.runBatch.dataset.baseLabel) {
      els.runBatch.dataset.baseLabel = els.runBatch.innerHTML;
    }
    els.runBatch.disabled = true;
    els.runBatch.innerHTML = '<span class="spinner-border spinner-border-sm me-2" role="status" aria-hidden="true"></span>Calculando...';
  } else {
    els.runBatch.disabled = false;
    if (els.runBatch.dataset.baseLabel) {
      els.runBatch.innerHTML = els.runBatch.dataset.baseLabel;
    }
  }
}

function setCancelBatchEnabled(enabled) {
  if (!els.cancelBatch) return;
  els.cancelBatch.disabled = !enabled;
}

function setParamsLocked(locked) {
  const lockables = [
    els.W, els.W_txt,
    els.St, els.St_txt,
    els.Q, els.Q_txt,
    els.refP, els.refP_txt,
    els.refL, els.refL_txt,
    els.kp1, els.kp1_txt,
    els.ki1, els.ki1_txt,
    els.kd1, els.kd1_txt,
    els.kp2, els.kp2_txt,
    els.ki2, els.ki2_txt,
    els.kd2, els.kd2_txt,
    els.loopModeToggle,
    els.applyIdealGains,
    els.simDuration,
  ];

  lockables.forEach(el => {
    if (el) el.disabled = locked;
  });

  // Durante batch, os comandos principais ficam travados.
  if (els.start) els.start.disabled = locked;
  if (els.reset) els.reset.disabled = locked;
}

function updateKPIs(s) {
  els.kpi_L.textContent    = fmt(s.L, 6) + ' m';
  els.kpi_P.textContent    = fmt(s.P, 4) + ' Pa';
  els.kpi_u1.textContent   = fmt(s.u1_apos_desacoplador, 2) + ' kg/s';
  els.kpi_u2.textContent   = fmt(s.u2_apos_desacoplador / 1000, 2) + ' kW';
  els.kpi_refP.textContent = fmt(Number(els.refP.value), 2);
  els.kpi_refL.textContent = fmt(Number(els.refL.value), 6);
}

function resetKPIs() {
  ['kpi_L','kpi_P','kpi_u1','kpi_u2','kpi_refP','kpi_refL'].forEach(id => els[id].textContent = '—');
}

function setLoopMode(closedLoop) {
  if (!els.loopModeToggle || !els.loopModeLabel) return;
  els.loopModeToggle.checked = closedLoop;
  els.loopModeLabel.textContent = closedLoop ? 'Malha Fechada' : 'Malha Aberta';
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

  charts.ctrl = newChart(els.chartControles, 'Ações de Controle',
    ['u1 (PID L)', 'u2 (PID P)', 'p_c1 -> W', 'p_c2 -> Q (compensa W e St)']);
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
  pushPoint(charts.ctrl, tSec, [s.sinalControle1, s.sinalControle2, s.u1_apos_desacoplador, s.u2_apos_desacoplador / 1000]);
}

// ============================================================
// ESTADO GLOBAL
// ============================================================
let charts = {};
let timer = null, sim = null;
const MAX_POINTS = 1200;
let _stepCount = 0;
let _batchRunning = false;
let _batchCancelRequested = false;
let _batchTimer = null;

// ============================================================
// INSTÂNCIA
// ============================================================
function createSim() {
  const kp1 = gainFromText(els.kp1_txt, els.kp1);
  const ki1 = gainFromText(els.ki1_txt, els.ki1);
  const kd1 = gainFromText(els.kd1_txt, els.kd1);
  const kp2 = gainFromText(els.kp2_txt, els.kp2);
  const ki2 = gainFromText(els.ki2_txt, els.ki2);
  const kd2 = gainFromText(els.kd2_txt, els.kd2);

  return new CaldeiraJS({
    W:    Number(els.W.value),
    St:   Number(els.St.value),
    Q:    Number(els.Q.value) * 1000,
    refL: Number(els.refL.value),
    refP: Number(els.refP.value),
    pid1: { kp: kp1, ki: ki1, kd: kd1 },
    pid2: { kp: kp2, ki: ki2, kd: kd2 },
    consts: { ...C },
  });
}

function closedLoopAtivo() {
  return Boolean(els.loopModeToggle?.checked);
}

let _controlMode = 'MA'; // Malha Aberta ou Malha Fechada

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
  const closedLoop = closedLoopAtivo();

  // St e sempre perturbacao externa.
  sim.setSt(Number(els.St.value));
  sim.setRefL(Number(els.refL.value));
  sim.setRefP(Number(els.refP.value));
  
  // Atualizar ganhos (isso muda para MF se mexerem)
  sim.setGains1({
    kp: gainFromText(els.kp1_txt, els.kp1),
    ki: gainFromText(els.ki1_txt, els.ki1),
    kd: gainFromText(els.kd1_txt, els.kd1),
  });
  sim.setGains2({
    kp: gainFromText(els.kp2_txt, els.kp2),
    ki: gainFromText(els.ki2_txt, els.ki2),
    kd: gainFromText(els.kd2_txt, els.kd2),
  });

  // Entradas manipuladas: W e Q.
  const currentW = Number(els.W.value);
  const currentQ = Number(els.Q.value) * 1000;
  
  if (!closedLoop) {
    // Em MA, permite que o usuário mexe livremente
    sim.setW(currentW);
    sim.setQ(currentQ);
  }

  let lastS;
  for (let i = 0; i < stepsPerInterval; i++) {
    lastS = sim.stepOnce();
    
    if (closedLoop) {
      // Em MF, aplica os dois PIDs e o desacoplador
      sim.controleNivel(lastS.L);
      sim.controlePressao(lastS.P);
      sim.aplicarDesacoplador();
    }
    
    _stepCount++;
    if (_stepCount % STEPS_PER_POINT === 0) {
      const tSec = parseFloat((_stepCount * C.DELTA_T).toFixed(1));
      _pushAllCharts(tSec, lastS, sim.refL, sim.refP);
    }
  }

  // Sincronizar sliders das manipuladas em MF
  if (closedLoop && lastS) {
    els.W.value = lastS.u1_apos_desacoplador;
    els.W_txt.value = lastS.u1_apos_desacoplador.toFixed(2);
    els.Q.value = (lastS.u2_apos_desacoplador / 1000).toFixed(2);
    els.Q_txt.value = (lastS.u2_apos_desacoplador / 1000).toFixed(2);
  }

  updateKPIs(lastS);
  updateViz(lastS);
}

// ============================================================
// MODO TEMPO SIMULADO (batch)
// ============================================================
function runBatch() {
  if (_batchRunning) return;
  _batchRunning = true;
  _batchCancelRequested = false;

  const durS = Math.max(1, Number(els.simDuration.value) || 100);
  const totalSteps = Math.round(durS / C.DELTA_T);
  const stepsPerChunk = 500;

  els.status.textContent = '⏳ Calculando…';
  setParamsLocked(true);
  setRunBatchLoading(true);
  setCancelBatchEnabled(true);
  initCharts();
  resetKPIs();
  _stepCount = 0;

  const bSim = createSim();
  const closedLoop = closedLoopAtivo();
  let i = 0;
  let lastS;

  function finishBatch(statusMsg) {
    if (_batchTimer) {
      clearTimeout(_batchTimer);
      _batchTimer = null;
    }
    setRunBatchLoading(false);
    setCancelBatchEnabled(false);
    setParamsLocked(false);
    _batchRunning = false;
    _batchCancelRequested = false;
    els.status.textContent = statusMsg;
  }

  function processChunk() {
    if (_batchCancelRequested) {
      finishBatch('⏹️ Simulação cancelada.');
      return;
    }

    const stop = Math.min(totalSteps, i + stepsPerChunk);

    for (; i < stop; i++) {
      lastS = bSim.stepOnce();
      if (closedLoop) {
        bSim.controleNivel(lastS.L);
        bSim.controlePressao(lastS.P);
        bSim.aplicarDesacoplador();
      }
      _stepCount++;
      if (_stepCount % STEPS_PER_POINT === 0) {
        const tSec = parseFloat((_stepCount * C.DELTA_T).toFixed(1));
        _pushAllCharts(tSec, lastS, bSim.refL, bSim.refP);
      }
    }

    if (lastS) {
      updateKPIs(lastS);
      updateViz(lastS);
    }

    if (i < totalSteps) {
      _batchTimer = setTimeout(processChunk, 0);
      return;
    }

    finishBatch(`✅ Simulado: ${durS} s`);
  }

  _batchTimer = setTimeout(processChunk, 0);
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
  setLoopMode(false);

  // Restaurar sliders e textos
  const sliderMap = { 
    W: els.W, St: els.St, Q: els.Q, refP: els.refP, refL: els.refL, 
    kp1: els.kp1, ki1: els.ki1, kd1: els.kd1,
    kp2: els.kp2, ki2: els.ki2, kd2: els.kd2,
  };
  const txtMap    = { 
    W: els.W_txt, St: els.St_txt, Q: els.Q_txt, refP: els.refP_txt, refL: els.refL_txt, 
    kp1: els.kp1_txt, ki1: els.ki1_txt, kd1: els.kd1_txt,
    kp2: els.kp2_txt, ki2: els.ki2_txt, kd2: els.kd2_txt,
  };
  Object.keys(DEFAULTS).forEach(k => {
    if (sliderMap[k]) sliderMap[k].value = DEFAULTS[k];
    if (txtMap[k]) txtMap[k].value    = DEFAULTS[k];
  });

  // Resetar visualização
  updateViz({ L: 0, P: 0, Lf: 0, Ls: 0, LQ: 0, u1_apos_desacoplador: 0, u2_apos_desacoplador: 0 });
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

els.cancelBatch?.addEventListener('click', () => {
  if (!_batchRunning) return;
  _batchCancelRequested = true;
  els.status.textContent = '🛑 Cancelando simulação...';
  setCancelBatchEnabled(false);
});

els.applyIdealGains?.addEventListener('click', applyIdealGains);

// Toggle de malha aberta/fechada
els.loopModeToggle?.addEventListener('change', () => {
  setLoopMode(els.loopModeToggle.checked);
  if (sim) sim.resetPIDs();
});

// Se mexer nos ganhos dos PIDs, vai para Malha Fechada
const gainInputs = [
  els.kp1, els.kp1_txt, els.ki1, els.ki1_txt, els.kd1, els.kd1_txt,
  els.kp2, els.kp2_txt, els.ki2, els.ki2_txt, els.kd2, els.kd2_txt,
];
gainInputs.forEach(el => el?.addEventListener('input', () => setLoopMode(true)));
gainInputs.forEach(el => el?.addEventListener('change', () => setLoopMode(true)));

// Se mexer em W ou Q, vai para Malha Aberta
[els.W, els.W_txt, els.Q, els.Q_txt].forEach(el => el?.addEventListener('input', () => setLoopMode(false)));
[els.W, els.W_txt, els.Q, els.Q_txt].forEach(el => el?.addEventListener('change', () => setLoopMode(false)));

// OBS: St é perturbação, não muda o toggle

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
  setLoopMode(false);
  setCancelBatchEnabled(false);
}
boot();
