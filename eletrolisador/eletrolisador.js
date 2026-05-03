/**
 * ============================
 * Simulador Eletrolisador PEM — UFSC
 * Modelo eletroquímico completo (Lebbal & Lecœuche, 2009)
 * com dinâmica térmica.
 *
 * ENTRADAS:  I (corrente, A), Q (fluxo de calor de controle, W),
 *            Ncells (nº células, parâmetro), Tamb (°C)
 * SAÍDAS:    Vcell (V), T_stack (K), nH2 (mol/s)
 * REFERÊNCIA: T_stack
 * ============================
 *
 * Modelo de tensão (Lebbal & Lecœuche, 2009):
 *   Ucell = Vrev + Vact + Vdiff + Vohm
 *
 *   Vrev  = V0 + (RT/2F) * ln( Ph2 * Po2^0.5 / ah2o )
 *   Vact  = (RT / alpha*n*F) * ln(I / I0)
 *   Vdiff = -(RT / z*F) * ln(1 - i/Ilim)
 *   Vohm  = Rmem * I
 *
 *   Ph2  = Pcatodo - Ph2o(T)
 *   Po2  = Panodo  - Ph2o(T)
 *   Ph2o = 6.1078e-3 * exp(17.2694*(T-273.15)/(T-34.85))  [bar]
 *
 *  Dinâmica térmica do stack:
 *    M_th * dT/dt = Q_gen - Q_cool - Q_ctrl
 *    Q_gen  = Ncells * I * (Vcell - E_thermo)   [W]
 *    Q_cool = h_cool * (T - Tamb_K)             [W]
 *    Q_ctrl = Q  (fluxo de calor de controle, W — positivo = resfria)
 *
 *  Vazão molar de H₂:
 *    nH2 = Ncells * I / (2 * F)    [mol/s]
 */

// ── Constantes físicas ──────────────────────────────────────
const F        = 96485;       // C/mol  (Faraday)
const R_gas    = 8.314;       // J/(mol·K)
const alpha    = 0.5;         // coef. transferência de carga
const n_elec   = 2;           // elétrons por mol H₂

// ── Parâmetros PEM padrão ───────────────────────────────────
const V0       = 1.229;       // V  — potencial reversível padrão (25 °C)
const I0       = 1e-3;        // A  — corrente de troca
const Ilim     = 2.0;         // A/cm²  — densidade de corrente limite
const A_cell   = 100;         // cm²  — área da célula
const Rmem     = 0.18e-3;     // Ω  — resistência da membrana (célula)
const z_diff   = 2;           // nº elétrons em Vdiff
const Pcatodo  = 1.012325;    // bar  — pressão total cátodo
const Panodo   = 1.012325;    // bar  — pressão total ânodo
const ah2o     = 1.0;         // atividade da água líquida

// ── Parâmetros térmicos ─────────────────────────────────────
const M_th     = 5000;        // J/K
const h_cool   = 10;          // W/K
const E_thermo = 1.48;        // V  — tensão termoneutra

// ────────────────────────────────────────────────────────────
//  Pressão de vapor d'água (bar) — eq. (12)
// ────────────────────────────────────────────────────────────
function ph2o_bar(T_K) {
  return 6.1078e-3 * Math.exp(17.2694 * (T_K - 273.15) / (T_K - 34.85));
}

// ────────────────────────────────────────────────────────────
//  Componentes de tensão isoladas (para gráfico)
// ────────────────────────────────────────────────────────────
function calcComponents(I, T_K) {
  if (I <= 0) return { Vrev: 0, Vact: 0, Vdiff: 0, Vohm: 0 };

  const Ph2o = ph2o_bar(T_K);
  const Ph2  = Math.max(Pcatodo - Ph2o, 1e-9);
  const Po2  = Math.max(Panodo  - Ph2o, 1e-9);

  const Vrev  = V0 + (R_gas * T_K / (2 * F)) * Math.log(Ph2 * Math.sqrt(Po2) / ah2o);
  const Vact  = (R_gas * T_K / (alpha * n_elec * F)) * Math.log(I / I0);
  const Vohm  = Rmem * I;
  const i_d   = I / A_cell;
  const Vdiff = (i_d < Ilim)
    ? -(R_gas * T_K / (z_diff * F)) * Math.log(1 - i_d / Ilim)
    : 0;

  return { Vrev, Vact, Vdiff, Vohm };
}

// ────────────────────────────────────────────────────────────
class EletrolisadorJS {
  constructor(opts) {
    const { I, Q, Ncells, Tamb_C, refT, consts, pid } = opts;

    this.DELTA_T  = consts.DELTA_T;
    this.I        = I;
    this.Q_ctrl   = Q;           // W — fluxo de calor de controle
    this.Ncells   = Ncells;
    this.Tamb_K   = Tamb_C + 273.15;
    this.refT     = refT;
    this.T_stack  = this.Tamb_K;
    // PID
    this.Kp = pid.kp;
    this.Ki = pid.ki;
    this.Kd = pid.kd;
    this.acaoIntegral = 0;
    this._lastErro = 0;
    this.sinalControle = 0;
  }

  _calcVcell(I, T_K) {
    if (I <= 0) return 0;
    const c = calcComponents(I, T_K);
    return c.Vrev + c.Vact + c.Vdiff + c.Vohm;
  }

  controleTemperatura(T) {
    const erro = this.refT - T;
    const dErro = (erro - this._lastErro) / this.DELTA_T;
    this._lastErro = erro;

    this.acaoIntegral += erro * this.DELTA_T;
    const iTerm = this.Ki * this.acaoIntegral;

    const u = this.Kp * erro + iTerm + this.Kd * dErro;
    this.sinalControle = u;
    this.Q_ctrl = u;
    return erro;
  }

  stepOnce() {
    const dt     = this.DELTA_T;
    const I      = this.I;
    const T      = this.T_stack;

    const Vcell  = this._calcVcell(I, T);
    const Q_gen  = this.Ncells * I * Math.max(0, Vcell - E_thermo);
    const Q_cool = h_cool * (T - this.Tamb_K);
    const dT     = (Q_gen - Q_cool - this.Q_ctrl) / M_th;
    this.T_stack += dT * dt;

    const nH2 = (I > 0) ? (this.Ncells * I) / (2 * F) : 0;

    return {
      Vcell,
      T_stack: this.T_stack,
      nH2,
      I,
      Ncells:  this.Ncells,
      Q_ctrl:  this.Q_ctrl,
      Tamb_C:  this.Tamb_K - 273.15,
      sinalControle: this.sinalControle,
      erroT:   this.refT - this.T_stack,
    };
  }

  setI(v)       { this.I = v; }
  setQ(v)       { this.Q_ctrl = v; }
  setNcells(v)  { this.Ncells = v; }
  setTamb(c)    { this.Tamb_K = c + 273.15; }
  setRefT(v)    { this.refT = v; }
  setGains({kp, ki, kd}) {
    if (kp !== undefined) this.Kp = kp;
    if (ki !== undefined) this.Ki = ki;
    if (kd !== undefined) this.Kd = kd;
  }

  resetPID() {
    this.acaoIntegral = 0;
    this._lastErro = 0;
    this.sinalControle = this.Q_ctrl;
  }

  currentState() {
    const Vcell = this._calcVcell(this.I, this.T_stack);
    const nH2 = (this.I > 0) ? (this.Ncells * this.I) / (2 * F) : 0;
    return {
      Vcell,
      T_stack: this.T_stack,
      nH2,
      I: this.I,
      Ncells: this.Ncells,
      Q_ctrl: this.Q_ctrl,
      Tamb_C: this.Tamb_K - 273.15,
      sinalControle: this.sinalControle,
      erroT: this.refT - this.T_stack,
    };
  }

}

// ============================================================
// UTILITÁRIOS
// ============================================================
function q(s) { return document.querySelector(s); }
function fmt(x, d=4) { return Number(x).toFixed(d); }
function clamp(v, min, max) { return Math.max(min, Math.min(max, v)); }

// ============================================================
// CONSTANTES
// ============================================================
const C = { DELTA_T: 0.1 };
const STEPS_PER_POINT = 10;

// ============================================================
// DOM
// ============================================================
const els = {
  status: q('#statusMsg'),

  I:      q('#I'),      Q:      q('#Q'),      Tamb:  q('#Tamb'),
  refT:   q('#refT'),   Ncells: q('#Ncells'),
  kp:     q('#kp'),     ki:     q('#ki'),     kd:    q('#kd'),

  I_txt:      q('#I_txt'),    Q_txt:      q('#Q_txt'),    Tamb_txt:   q('#Tamb_txt'),
  refT_txt:   q('#refT_txt'), Ncells_txt: q('#Ncells_txt'),
  kp_txt:     q('#kp_txt'),   ki_txt:     q('#ki_txt'),   kd_txt:     q('#kd_txt'),
  loopModeToggle: q('#loopModeToggle'),
  loopModeLabel:  q('#loopModeLabel'),

  start:       q('#startBtn'),
  reset:       q('#resetBtn'),
  runBatch:    q('#runBatchBtn'),
  simDuration: q('#simDuration'),
  kpi_V:    q('#kpi_V'),
  kpi_T:    q('#kpi_T'),
  kpi_nH2:  q('#kpi_nH2'),
  kpi_refT: q('#kpi_refT'),

  chartV:        q('#chartV'),
  chartT:        q('#chartT'),
  chartPowerTemp: q('#chartPowerTemp'),
  chartH2:       q('#chartH2'),
  chartEntradas: q('#chartEntradas'),

  toast: q('#toast'),
};

const DEFAULTS = {
  I: 0, Q: 0, Ncells: 50, Tamb: 25, refT: 353, kp: 0, ki: 0, kd: 0,
};

// ============================================================
// SINCRONIZAÇÃO SLIDER <-> TEXTO
// ============================================================
function readParamValue(sliderEl, txtEl) {
  const txtV = Number(txtEl?.value);
  if (Number.isFinite(txtV)) return txtV;
  return Number(sliderEl?.value ?? 0);
}

const pares = [
  [els.I,      els.I_txt],
  [els.Q,      els.Q_txt],
  [els.Tamb,   els.Tamb_txt],
  [els.refT,   els.refT_txt],
  [els.Ncells, els.Ncells_txt],
];

pares.forEach(([slider, txt]) => {
  if (!slider || !txt) return;
  slider.addEventListener('input', () => { txt.value = slider.value; });
  const syncSliderOnly = () => {
    const v = Number(txt.value);
    if (!Number.isFinite(v)) return;
    slider.value = Math.max(Number(slider.min), Math.min(Number(slider.max), v));
  };
  txt.addEventListener('input', syncSliderOnly);
  txt.addEventListener('change', syncSliderOnly);
  txt.addEventListener('keydown', e => { if (e.key === 'Enter') syncSliderOnly(); });
});

const gainPairs = [
  [els.kp, els.kp_txt],
  [els.ki, els.ki_txt],
  [els.kd, els.kd_txt],
];

gainPairs.forEach(([slider, txt]) => {
  if (!slider || !txt) return;

  // Ao mexer no slider, o ganho passa a ser exatamente o valor do slider.
  slider.addEventListener('input', () => { txt.value = slider.value; });

  // Ao editar texto, mantém o valor digitado para o ganho e satura apenas a posição visual do slider.
  const syncGainSliderOnly = () => {
    const v = Number(txt.value);
    if (!Number.isFinite(v)) return;
    const clamped = clamp(v, Number(slider.min), Number(slider.max));
    slider.value = clamped;
  };

  txt.addEventListener('input', syncGainSliderOnly);
  txt.addEventListener('change', syncGainSliderOnly);
  txt.addEventListener('keydown', e => { if (e.key === 'Enter') syncGainSliderOnly(); });
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
  els.start.disabled   = false;
  els.reset.disabled   = running;
  els.runBatch.disabled = running;
  els.start.textContent = running ? 'Parar' : 'Iniciar';
  els.start.classList.toggle('btn-danger',  running);
  els.start.classList.toggle('btn-primary', !running);
}

function updateKPIs(s) {
  els.kpi_V.textContent    = fmt(s.Vcell, 4) + ' V';
  els.kpi_T.textContent    = fmt(s.T_stack, 2) + ' K';
  els.kpi_nH2.textContent  = fmt(s.nH2, 6) + ' mol/s';
  els.kpi_refT.textContent = fmt(readParamValue(els.refT, els.refT_txt), 1) + ' K';
}

function resetKPIs() {
  ['kpi_V','kpi_T','kpi_nH2','kpi_refT'].forEach(id => els[id].textContent = '—');
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
      datasets: labels.map(l => ({
        label: l,
        data: [],
        borderWidth: 2,
        tension: 0.22,
        pointRadius: 0,
        pointHoverRadius: 3,
        borderDash: l.startsWith('Ref') ? [5, 5] : [],
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
  charts.V   = newChart(els.chartV,   'Tensão (V)',    ['U_cell', 'V_rev', 'V_act', 'V_diff', 'V_ohm']);
  charts.T   = newChart(els.chartT,   'Temp. (K)',     ['T_stack', 'Ref. Temperatura']);
  charts.H2  = newChart(els.chartH2,  'Vazão (mol/s)', ['ṅ_H₂']);
  charts.ent = newChart(els.chartEntradas, 'Entradas', ['I (A)', 'Q (W)', 'T_amb (°C)']);

  // Gráfico combinado: Potência (W, esquerda) & Temperatura do stack (K, direita)
  const ctx = els.chartPowerTemp.getContext('2d');
  charts.powerTemp = new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: 'T_stack (K)', data: [], borderColor: '#f97316', backgroundColor: '#f97316', yAxisID: 'y1', tension: 0.22, pointRadius: 0 },
        { label: 'P_eletrolisador (W)', data: [], borderColor: '#2563eb', backgroundColor: '#2563eb', yAxisID: 'y', tension: 0.22, pointRadius: 0 }
      ]
    },
    options: {
      responsive: true,
      animation: false,
      scales: {
        x: { title: { display: true, text: 'Tempo (s)' } },
        y: { position: 'left', title: { display: true, text: 'Potência (W)' } },
        y1: { position: 'right', grid: { drawOnChartArea: false }, title: { display: true, text: 'Temperatura (K)' } }
      },
      plugins: {
        title: { display: true, text: 'Potência consumida e temperatura da pilha' },
        // legenda na parte inferior para ocupar menos espaço lateral
        legend: { position: 'bottom', align: 'center' },
        tooltip: { mode: 'nearest', intersect: false }
      }
    }
  });
}

function pushPoint(chart, lbl, arrs) {
  chart.data.labels.push(lbl);
  chart.data.datasets.forEach((ds, i) => ds.data.push(arrs[i] ?? null));
  chart.update('quiet');
}

function _pushAllCharts(tSec, s, refT) {
  const comp = calcComponents(s.I, s.T_stack);
  pushPoint(charts.V,   tSec, [s.Vcell, comp.Vrev, comp.Vact, comp.Vdiff, comp.Vohm]);
  pushPoint(charts.T,   tSec, [s.T_stack, refT]);
  pushPoint(charts.H2,  tSec, [s.nH2]);
  pushPoint(charts.ent, tSec, [s.I, s.Q_ctrl, s.Tamb_C]);
  // Potência elétrica total do stack (W) = Vcell * Ncells * I
  if (charts.powerTemp) {
    const P_elec = (s.Vcell || 0) * (s.Ncells || 0) * (s.I || 0);
    // ordem dos datasets: [T_stack, P_eletrolisador]
    pushPoint(charts.powerTemp, tSec, [s.T_stack, P_elec]);
  }
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
  const kp = gainFromText(els.kp_txt, els.kp);
  const ki = gainFromText(els.ki_txt, els.ki);
  const kd = gainFromText(els.kd_txt, els.kd);
  return new EletrolisadorJS({
    I:      readParamValue(els.I, els.I_txt),
    Q:      readParamValue(els.Q, els.Q_txt),
    Ncells: readParamValue(els.Ncells, els.Ncells_txt),
    Tamb_C: readParamValue(els.Tamb, els.Tamb_txt),
    refT:   readParamValue(els.refT, els.refT_txt),
    pid:    { kp, ki, kd },
    consts: { ...C },
  });
}

function closedLoopAtivo() {
  return Boolean(els.loopModeToggle?.checked);
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
  const s0 = sim.currentState();
  _pushAllCharts(0, s0, sim.refT);
  updateKPIs(s0);
  updateViz(s0);
  uiRunning(true);
  const intervalMs = 16;
  const stepsPerInterval = Math.max(1, Math.round(intervalMs / (C.DELTA_T * 1000)));
  timer = setInterval(() => tickRealtime(stepsPerInterval), intervalMs);
}

function stopRealtime() {
  clearInterval(timer);
  timer = null; sim = null;
  uiRunning(false);
  els.status.textContent = '⏹️ Parado.';
}

function tickRealtime(stepsPerInterval) {
  if (!sim) return;
  const closedLoop = closedLoopAtivo();

  sim.setI(readParamValue(els.I, els.I_txt));
  if (!closedLoop) sim.setQ(readParamValue(els.Q, els.Q_txt));
  sim.setNcells(readParamValue(els.Ncells, els.Ncells_txt));
  sim.setTamb(readParamValue(els.Tamb, els.Tamb_txt));
  sim.setRefT(readParamValue(els.refT, els.refT_txt));
  sim.setGains({
    kp: gainFromText(els.kp_txt, els.kp),
    ki: gainFromText(els.ki_txt, els.ki),
    kd: gainFromText(els.kd_txt, els.kd),
  });

  els.I.value = clamp(sim.I, Number(els.I.min), Number(els.I.max));
  els.I_txt.value = sim.I;
  if (!closedLoop) {
    els.Q.value = clamp(sim.Q_ctrl, Number(els.Q.min), Number(els.Q.max));
    els.Q_txt.value = sim.Q_ctrl;
  }

  let lastS;
  for (let i = 0; i < stepsPerInterval; i++) {
    if (closedLoop) sim.controleTemperatura(sim.T_stack);
    lastS = sim.stepOnce();
    _stepCount++;
    if (_stepCount % STEPS_PER_POINT === 0) {
      const tSec = parseFloat((_stepCount * C.DELTA_T).toFixed(1));
      _pushAllCharts(tSec, lastS, sim.refT);
    }
  }

  if (closedLoop && lastS) {
    const qVal = sim.Q_ctrl;
    els.Q.value = clamp(qVal, Number(els.Q.min), Number(els.Q.max));
    els.Q_txt.value = qVal.toFixed(1);
    lastS.Q_ctrl = qVal;
  }

  updateKPIs(lastS);
  updateViz(lastS);
}

// ============================================================
// MODO BATCH
// ============================================================
function runBatch() {
  const durS = Math.max(1, Number(els.simDuration.value) || 100);
  const totalSteps = Math.round(durS / C.DELTA_T);
  els.status.textContent = '⏳ Calculando…';
  initCharts();
  resetKPIs();
  _stepCount = 0;

  const bSim = createSim();
  const s0 = bSim.currentState();
  _pushAllCharts(0, s0, bSim.refT);
  const closedLoop = closedLoopAtivo();
  let lastS;
  for (let i = 0; i < totalSteps; i++) {
    if (closedLoop) bSim.controleTemperatura(bSim.T_stack);
    lastS = bSim.stepOnce();
    _stepCount++;
    if (_stepCount % STEPS_PER_POINT === 0) {
      const tSec = parseFloat((_stepCount * C.DELTA_T).toFixed(1));
      _pushAllCharts(tSec, lastS, bSim.refT);
    }
  }

  if (closedLoop && lastS) {
    const qVal = bSim.Q_ctrl;
    els.Q.value = clamp(qVal, Number(els.Q.min), Number(els.Q.max));
    els.Q_txt.value = qVal.toFixed(1);
    lastS.Q_ctrl = qVal;
  }

  updateKPIs(lastS);
  updateViz(lastS);
  els.status.textContent = `✅ Simulado: ${durS} s`;
}

// ============================================================
// RESET
// ============================================================
function fullReset() {
  if (timer) stopRealtime();
  initCharts();
  resetKPIs();
  _stepCount = 0;
  _vizT = 0;
  els.status.textContent = 'Pronto';
  uiRunning(false);

  const sliderMap = { I: els.I, Q: els.Q, Ncells: els.Ncells, Tamb: els.Tamb, refT: els.refT, kp: els.kp, ki: els.ki, kd: els.kd };
  const txtMap    = { I: els.I_txt, Q: els.Q_txt, Ncells: els.Ncells_txt, Tamb: els.Tamb_txt, refT: els.refT_txt, kp: els.kp_txt, ki: els.ki_txt, kd: els.kd_txt };
  Object.keys(DEFAULTS).forEach(k => {
    if (sliderMap[k]) sliderMap[k].value = DEFAULTS[k];
    if (txtMap[k])    txtMap[k].value    = DEFAULTS[k];
  });
  setLoopMode(false);
  updateViz({ Vcell: 0, T_stack: DEFAULTS.Tamb + 273.15, nH2: 0, Ncells: DEFAULTS.Ncells });
}

// ============================================================
// VISUALIZAÇÃO DINÂMICA
// ============================================================
const vizTemp   = document.getElementById('vizTemp');
const vizVcell  = document.getElementById('vizVcell');
const vizNcells = document.getElementById('vizNcells');
let _vizT = 0;

function updateViz(s) {
  _vizT += (s.T_stack - _vizT) * 0.15;
  if (vizTemp)   vizTemp.textContent   = _vizT.toFixed(1) + ' K';
  if (vizVcell)  vizVcell.textContent  = fmt(s.Vcell, 4) + ' V';
  if (vizNcells) vizNcells.textContent = s.Ncells;
}

// ============================================================
// EVENTOS
// ============================================================
els.start.addEventListener('click', () => { timer ? stopRealtime() : startRealtime(); });
els.reset.addEventListener('click', fullReset);
els.runBatch.addEventListener('click', () => { if (timer) stopRealtime(); runBatch(); });
els.loopModeToggle?.addEventListener('change', () => {
  setLoopMode(els.loopModeToggle.checked);
  if (sim) sim.resetPID();
});

const gainInputs = [els.kp, els.kp_txt, els.ki, els.ki_txt, els.kd, els.kd_txt];
gainInputs.forEach(el => el?.addEventListener('input', () => setLoopMode(true)));
gainInputs.forEach(el => el?.addEventListener('change', () => setLoopMode(true)));

els.Q?.addEventListener('input', () => setLoopMode(false));
els.Q_txt?.addEventListener('input', () => setLoopMode(false));
els.Q_txt?.addEventListener('change', () => setLoopMode(false));

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
  updateViz({ Vcell: 0, T_stack: DEFAULTS.Tamb + 273.15, nH2: 0, Ncells: DEFAULTS.Ncells });
}
boot();
