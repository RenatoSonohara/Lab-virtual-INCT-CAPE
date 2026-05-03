/** ============================
 * Simulador Coletor Solar — Rev2
 * - Modelo térmico principal trocado para a Equação (11)
 * - Tin simplificado para comportamento rotativo/suave
 * - Ruído pequeno em Tin para parecer mais realista
 * - Vazão na interface: volumétrica (L/min)
 * ============================ */

/* ======= Utilitários DOM ======= */
function q(s){ return document.querySelector(s); }
function fmt(x, d=2){ return Number(x).toFixed(d); }

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v));
}

/* ======= Modelo + Controlador ======= */
class ColetorSolarJS {
  constructor(opts) {
    const {
      irradiacao_solar, temperaturaAmbiente, porcentagem_vazao, referencia,
      consts, pid
    } = opts;

    // Entradas/estados
    this.irradiacao_solar = irradiacao_solar;        // I [W/m²]
    this.TEMPERATURA_AMBIENTE = temperaturaAmbiente; // Ta [°C]
    this.REFERENCIA = referencia;

    // Propriedades do fluido (água - aproximação)
    this.rho = 1000.0;      // [kg/m³]
    this.cp  = 4186.0;      // [J/(kg·K)]

    // Vazão (UI): volumétrica em L/min
    this.vazaoVolNominal_Lmin = consts.vazaoNominal; // [L/min]
    this.vazaoVol_Lmin = (porcentagem_vazao / 100.0) * this.vazaoVolNominal_Lmin;

    // ===== Parâmetros físicos alinhados à equação (11) =====
    this.Ac = consts.Ac;                                  // área do coletor
    this.UL = consts.UL ?? consts.U ?? 4.0;              // coef. global de perdas
    this.eta0 = consts.eta0 ?? consts.tauAlpha ?? 0.70;  // eficiência óptica efetiva
    this.tau = consts.tau ?? 1.0;                        // transmissividade separada (opcional)
    this.V_col = consts.V_col;                           // volume interno [m³]

    // Capacidade térmica efetiva do sistema
    this.mp = consts.mp ?? 6.0;                          // massa equivalente da placa [kg]
    this.cpp = consts.cpp ?? 900.0;                      // calor específico equivalente [J/kg.K]
    this.C = consts.C ?? ((this.rho * this.V_col * this.cp) + (this.mp * this.cpp));

    // Mantidos por compatibilidade com UI/KPIs antigos
    this.NSEG = consts.NSEG ?? 5;
    this.Vseg = this.V_col / this.NSEG;
    this.V_loop = consts.V_loop ?? 0.008;
    this.UA_loop = consts.UA_loop ?? 6.0;

    // ===== Parâmetros da entrada simplificada =====
    this.tinMix = consts.tinMix ?? 0.82;           // inércia/suavização de Tin (0.7~0.95 bom)
    this.tinNoiseAmp = consts.tinNoiseAmp ?? 0.12; // amplitude do ruído [°C]
    this.tinUseOutletMean = consts.tinUseOutletMean ?? true;

    // Numérico
    this.DELTA_T = consts.DELTA_T; // [s]

    // PID
    this.Kp = pid.kp;
    this.Ki = pid.ki;
    this.Kd = pid.kd;
    this.Tf = pid.tf ?? 0;

    this.acaoIntegral = 0.0;
    this._lastMeas = null;
    this._dMeasFilt = 0.0;

    // Estados iniciais
    this.TEMPERATURA_ENTRADA = this.TEMPERATURA_AMBIENTE - 3.0;

    // Tc = temperatura principal do coletor (estado da equação 11)
    this.Tc = this.TEMPERATURA_ENTRADA;

    // Compatibilidade com interface já existente
    this.Tp = this.Tc;
    this.Tseg = new Array(this.NSEG).fill(this.Tc);

    this._lastMeas = this.Tc;
    this._dMeasFilt = 0.0;
  }

  /* ===== Conversões vazão ===== */
  getVdot_m3s() {
    return this.vazaoVol_Lmin / 60000.0; // L/min -> m³/s
  }

  getMdot_kg_s() {
    return this.rho * this.getVdot_m3s();
  }

  /* ===== Capacidades térmicas ===== */
  Cloop() {
    return this.rho * this.V_loop * this.cp;
  }

  /* ===== UI ===== */
  getPorcentagemVazao() {
    const denom = Math.max(this.vazaoVolNominal_Lmin, 1e-9);
    return (this.vazaoVol_Lmin / denom) * 100.0;
  }

  setPorcentagemVazao(pct) {
    this.vazaoVol_Lmin = (pct / 100.0) * this.vazaoVolNominal_Lmin;
  }

  getVazaoVol_Lmin() {
    return this.vazaoVol_Lmin;
  }

  setGains({kp,ki,kd,tf}) {
    if (kp !== undefined) this.Kp = kp;
    if (ki !== undefined) this.Ki = ki;
    if (kd !== undefined) this.Kd = kd;
    if (tf !== undefined) this.Tf = tf;
  }

  setReferencia(r) { this.REFERENCIA = r; }
  setIrradiacao(I) { this.irradiacao_solar = I; }
  setTa(Ta)        { this.TEMPERATURA_AMBIENTE = Ta; }

  /* ===== Ruído simples ===== */
  randUniform(min, max) {
    return min + (max - min) * Math.random();
  }

  /* ===== PID com derivada na medição + filtro derivativo ===== */
  // erro = Tout - Ref
  controleVazao(temperaturaSaida) {
    const erro_pv = temperaturaSaida - this.REFERENCIA;

    const dMeasRaw = (temperaturaSaida - this._lastMeas) / this.DELTA_T;
    this._lastMeas = temperaturaSaida;

    let dMeas = dMeasRaw;
    if (this.Tf > 0) {
      const alpha = this.DELTA_T / (this.Tf + this.DELTA_T);
      this._dMeasFilt = this._dMeasFilt + alpha * (dMeasRaw - this._dMeasFilt);
      dMeas = this._dMeasFilt;
    } else {
      this._dMeasFilt = dMeasRaw;
    }

    this.acaoIntegral += (this.Ki * erro_pv * this.DELTA_T);
    const uPct = (this.Kp * erro_pv) + this.acaoIntegral + (-this.Kd * dMeas);

    this.setPorcentagemVazao(uPct);
    return erro_pv;
  }

  /* ===== Núcleo da simulação ===== */
  stepOnce() {
    const I = this.irradiacao_solar;
    const Ta = this.TEMPERATURA_AMBIENTE;
    const Tin = this.TEMPERATURA_ENTRADA;

    const Vdot = Math.max(this.getVdot_m3s(), 1e-12);   // [m³/s]
    const mdot = Math.max(this.getMdot_kg_s(), 1e-12);  // [kg/s]
    const Ceff = Math.max(this.C, 1e-9);

    const TcAtual = this.Tc;
    const Tav = (Tin + TcAtual) / 2.0;

    // =========================================================
    // EQUAÇÃO (11)
    // dTc/dt = (Ac * tau * eta0 / C) * I
    //        - (UL * Ac / C) * (Tav - Ta)
    //        + (Fc / V) * (Tin - Tc)
    //
    // Aqui:
    // Fc ~ vazão volumétrica Vdot [m³/s], como combinado na adaptação
    // =========================================================
    const termoSolar = ((this.Ac * this.tau * this.eta0) / Ceff) * I;
    const termoPerda = ((this.UL * this.Ac) / Ceff) * (Tav - Ta);
    const termoFluxo = (Vdot / Math.max(this.V_col, 1e-12)) * (Tin - TcAtual);

    const dTc_dt = termoSolar - termoPerda + termoFluxo;

    // Euler explícito
    const TcNovo = TcAtual + dTc_dt * this.DELTA_T;
    this.Tc = TcNovo;

    // Compatibilidade visual com o restante da UI
    this.Tp = TcNovo;
    this.Tseg.fill(TcNovo);

    const Tout = TcNovo;

    // =========================================================
    // Tin simplificado:
    // média entre ambiente e saída, com suavização e pequeno ruído
    // para simular rotatividade e evitar efeito de acúmulo pesado
    // =========================================================
    const baseTinAlvo = this.tinUseOutletMean
      ? ((Ta + Tout) / 2.0)
      : ((Ta + Tin) / 2.0);

    const ruido = this.randUniform(-this.tinNoiseAmp, this.tinNoiseAmp);

    // mistura entre o Tin anterior e o valor alvo
    const TinNovo = (this.tinMix * Tin) + ((1 - this.tinMix) * baseTinAlvo) + ruido;

    // evita valores absurdos
    this.TEMPERATURA_ENTRADA = Math.max(-20, Math.min(150, TinNovo));

    // Métricas
    const Qsol = this.Ac * this.tau * this.eta0 * I;
    const Qloss = this.UL * this.Ac * (Tav - Ta);
    const Qpf = mdot * this.cp * (Tout - Tin);

    const ganhoFluido = Math.max(0, Qpf);
    const eta =
      (I * this.Ac) > 1e-9
        ? Math.max(0, Math.min(1, ganhoFluido / (I * this.Ac)))
        : 0;

    return {
      Tin: this.TEMPERATURA_ENTRADA,
      Tout,
      Tp: this.Tp,
      vazaoPct: this.getPorcentagemVazao(),
      vazaoVol_Lmin: this.getVazaoVol_Lmin(),
      Qsol,
      Qloss,
      Qpf,
      eta,
      mdot_kg_s: mdot,
      dT: (Tout - this.TEMPERATURA_ENTRADA),
      erro: (Tout - this.REFERENCIA),
      dTc_dt,
      Tav
    };
  }

  currentState() {
    const Tin = this.TEMPERATURA_ENTRADA;
    const Tout = this.Tc;
    const Tp = this.Tp;
    const Ta = this.TEMPERATURA_AMBIENTE;
    const I = this.irradiacao_solar;
    const mdot = Math.max(this.getMdot_kg_s(), 1e-12);
    const Tav = (Tin + Tout) / 2.0;
    const Qsol = this.Ac * this.tau * this.eta0 * I;
    const Qloss = this.UL * this.Ac * (Tav - Ta);
    const Qpf = mdot * this.cp * (Tout - Tin);
    const ganhoFluido = Math.max(0, Qpf);
    const eta = (I * this.Ac) > 1e-9
      ? Math.max(0, Math.min(1, ganhoFluido / (I * this.Ac)))
      : 0;

    return {
      Tin,
      Tout,
      Tp,
      vazaoPct: this.getPorcentagemVazao(),
      vazaoVol_Lmin: this.getVazaoVol_Lmin(),
      Qsol,
      Qloss,
      Qpf,
      eta,
      mdot_kg_s: mdot,
      dT: (Tout - Tin),
      erro: (Tout - this.REFERENCIA),
      dTc_dt: 0,
      Tav,
    };
  }
}

/* ======= Utilidades de tempo / CSV ======= */
function parseTimeToSeconds(s) {
  const parts = s.trim().split(':').map(Number);
  if (parts.length < 2) return NaN;
  const h = parts[0] || 0;
  const m = parts[1] || 0;
  const sec = parts[2] || 0;
  return h * 3600 + m * 60 + sec;
}

function secondsToHHMMSS(sec) {
  sec = Math.max(0, Math.round(sec));
  const h = Math.floor(sec / 3600);
  const m = Math.floor((sec % 3600) / 60);
  const s = sec % 60;
  return `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}:${String(s).padStart(2,'0')}`;
}

function secondsToHHMM(sec) {
  sec = Math.max(0, Math.round(sec));
  const h = Math.floor(sec / 3600);
  const m = Math.floor((sec % 3600) / 60);
  return `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}`;
}

function secondsToLabel(sec, stepSec) {
  return (stepSec < 60) ? secondsToHHMMSS(sec) : secondsToHHMM(sec);
}

function parseCSV(text) {
  const lines = text.split(/\r?\n/);
  const data = [];
  for (const line of lines) {
    if (!line.trim()) continue;
    const parts = line.split(',').map(x => x.trim());
    if (parts.length < 3) continue;

    const tsec = parseTimeToSeconds(parts[0]);
    const irr = Number(parts[1].replace(/,/g,'.'));
    const ta  = Number(parts[2].replace(/,/g,'.'));

    if (Number.isFinite(tsec) && Number.isFinite(irr) && Number.isFinite(ta)) {
      data.push({ tsec, irr, ta });
    }
  }
  data.sort((a,b)=>a.tsec-b.tsec);
  return data;
}

function interpolateProfile(table, tsec) {
  if (!table || table.length === 0) return null;
  if (tsec <= table[0].tsec) return { irr: table[0].irr, ta: table[0].ta };
  if (tsec >= table[table.length - 1].tsec) return { irr: table[table.length - 1].irr, ta: table[table.length - 1].ta };

  let i = 1;
  while (i < table.length && table[i].tsec < tsec) i++;

  const a = table[i - 1];
  const b = table[i];
  // Protege contra pontos com mesmo timestamp (evita divisão por zero)
  if (b.tsec === a.tsec) {
    return { irr: a.irr, ta: a.ta };
  }

  const alpha = (tsec - a.tsec) / (b.tsec - a.tsec);
  return {
    irr: a.irr + alpha * (b.irr - a.irr),
    ta : a.ta  + alpha * (b.ta  - a.ta)
  };
}

function defaultSolarProfile(tsec, startSec, endSec) {
  const mid = (startSec + endSec) / 2;
  const span = Math.max(1, (endSec - startSec) / 2);
  const x = Math.max(-1, Math.min(1, (tsec - mid) / span));
  const baseIrr = 900 * Math.max(0, 1 - x * x);
  const Ta = 20 + 7 * Math.max(0, 1 - Math.abs(x));
  return { irr: baseIrr, ta: Ta };
}

/* ======= DOM ======= */
const els = {
  tmReal : q('#tmReal'),
  tmSim  : q('input[name="timeMode"][value="sim"]'),
  simBlock: q('#simulatedBlock'),
  simStart: q('#simStart'),
  simEnd  : q('#simEnd'),
  simStep : q('#simStep'),
  fastRun : q('#fastRun'),
  status  : q('#statusMsg'),
  clockNow: q('#clockNow'),

  irr: q('#irr'), irrTxt: q('#irr_txt'), irrVal: q('#irr_val'),
  ta: q('#ta'), taTxt: q('#ta_txt'), taVal: q('#ta_val'),
  vaz: q('#vaz'), vazTxt: q('#vaz_txt'), vazVal: q('#vaz_val'),

  ref: q('#ref'), refTxt: q('#ref_txt'), refVal: q('#ref_val'),
  kp: q('#kp'), kpTxt: q('#kp_txt'), kpVal: q('#kp_val'),
  ki: q('#ki'), kiTxt: q('#ki_txt'), kiVal: q('#ki_val'),
  kd: q('#kd'), kdTxt: q('#kd_txt'), kdVal: q('#kd_val'),
  tf: q('#tf'), tfTxt: q('#tf_txt'), tfVal: q('#tf_val'),

  start: q('#startBtn'),
  reset: q('#resetBtn'),

  Tin: q('#Tin'),
  Tout: q('#Tout'),
  Tp: q('#Tp'),
  vazaoPct: q('#vazao_pct'),
  eta: q('#eta'),
  dT: q('#dT'),
  mdot: q('#mdot'),
  refNow: q('#refNow'),
  errNow: q('#errNow'),

  csvFile: q('#csvFile'),
  csvCount: q('#csvCount'),
  csvSpan: q('#csvSpan'),

  chartTemp: q('#chartTemp'),
  chartVazao: q('#chartVazao'),
  chartPower: q('#chartPower'),
  chartErro: q('#chartErro'),
  chartIrrTa: q('#chartIrrTa'),

  constBtn: q('#constBtn'),
  constModal: q('#constModal'),
  closeConst: q('#closeConst'),
  cancelConst: q('#cancelConst'),
  applyConst: q('#applyConst'),

  toast: q('#toast')
};

/* ======= Estado ======= */
let charts = {};
let csvTable = [];
let csvStart = null;
let csvEnd = null;

// Constantes do coletor (editáveis no modal)
let C = {
  Ac: 2.0,
  U: 4.0,              // usado como UL
  tauAlpha: 0.70,      // usado como eta0 efetivo
  NSEG: 5,             // mantido por compatibilidade visual
  V_col: 0.002,
  mp: 6.0,
  cpp: 900.0,
  hA_total: 160.0,     // mantido, mas não entra mais no núcleo da equação 11
  UA_tubos_total: 12.0,// mantido, mas não entra mais no núcleo da equação 11
  V_loop: 0.008,
  UA_loop: 6.0,
  DELTA_T: 1.0,
  vazaoNominal: 6.0,   // L/min

  // Novos parâmetros para Tin simplificado
  tinMix: 0.82,
  tinNoiseAmp: 0.12,
  tinUseOutletMean: true
};

let timeMode = 'real';

let timer = null;
let sim = null;
let realtimeState = 'stopped';
let realtimeElapsedSec = 0;

const MAX_POINTS_REALTIME = 300;

let constModalInstance = null;

function readControlValue(sliderEl, txtEl) {
  const txtV = Number(txtEl?.value);
  if (Number.isFinite(txtV)) return txtV;
  return Number(sliderEl?.value ?? 0);
}

function bindSliderText(slider, txt) {
  if (!slider || !txt) return;
  slider.addEventListener('input', () => {
    txt.value = slider.value;
    updateLabels();
  });

  const syncSliderOnly = () => {
    const v = Number(txt.value);
    if (!Number.isFinite(v)) return;
    slider.value = clamp(v, Number(slider.min), Number(slider.max));
    updateLabels();
  };

  txt.addEventListener('input', syncSliderOnly);
  txt.addEventListener('change', syncSliderOnly);
  txt.addEventListener('keydown', e => { if (e.key === 'Enter') syncSliderOnly(); });
}

/* ======= UI ======= */
function updateLabels() {
  const irrV = readControlValue(els.irr, els.irrTxt);
  const taV = readControlValue(els.ta, els.taTxt);
  const refV = readControlValue(els.ref, els.refTxt);
  const vazV = readControlValue(els.vaz, els.vazTxt);
  const kpV = readControlValue(els.kp, els.kpTxt);
  const kiV = readControlValue(els.ki, els.kiTxt);
  const kdV = readControlValue(els.kd, els.kdTxt);
  const tfV = readControlValue(els.tf, els.tfTxt);

  els.irrVal.textContent = Number(irrV).toFixed(0);
  els.taVal.textContent  = Number(taV).toFixed(1);
  els.refVal.textContent = Number(refV).toFixed(1);
  els.vazVal.textContent = `${Number(vazV).toFixed(0)}%`;
  els.kpVal.textContent  = Number(kpV).toFixed(2);
  els.kiVal.textContent  = Number(kiV).toFixed(2);
  els.kdVal.textContent  = Number(kdV).toFixed(3);
  els.tfVal.textContent  = Number(tfV).toFixed(1);

  els.refNow.textContent = els.refVal.textContent + ' °C';
}

[
  [els.irr, els.irrTxt],
  [els.ta, els.taTxt],
  [els.vaz, els.vazTxt],
  [els.ref, els.refTxt],
  [els.kp, els.kpTxt],
  [els.ki, els.kiTxt],
  [els.kd, els.kdTxt],
  [els.tf, els.tfTxt],
].forEach(([slider, txt]) => bindSliderText(slider, txt));

[els.tmReal, els.tmSim].forEach(r => {
  r.addEventListener('change', () => {
    if (timer) pauseRealtime();
    realtimeState = 'stopped';
    realtimeElapsedSec = 0;
    sim = null;

    timeMode = els.tmReal.checked ? 'real' : 'sim';
    els.simBlock.classList.toggle('hidden', timeMode !== 'sim');
    updateClock();
    uiRunning(false);
    initCharts();
  });
});

els.csvFile.addEventListener('change', async (e) => {
  const file = e.target.files?.[0];
  if (!file) {
    csvTable = [];
    els.csvCount.textContent = '0';
    els.csvSpan.textContent = '—';
    return;
  }

  const text = await file.text();
  csvTable = parseCSV(text);
  els.csvCount.textContent = String(csvTable.length);

  if (csvTable.length > 0) {
    csvStart = csvTable[0].tsec;
    csvEnd = csvTable[csvTable.length - 1].tsec;
    els.csvSpan.textContent = `${secondsToHHMM(csvStart)} → ${secondsToHHMM(csvEnd)}`;
  } else {
    els.csvSpan.textContent = '—';
    csvStart = csvEnd = null;
  }
});

function uiRunning(runningBatch) {
  const isBatch = runningBatch && timeMode === 'sim';

  els.start.disabled = isBatch;
  els.reset.disabled = runningBatch;

  els.constBtn.disabled = runningBatch;
  [els.tmReal, els.tmSim, els.csvFile, els.simStart, els.simEnd, els.simStep].forEach(el => el.disabled = runningBatch);

  const simControlsDisabled = isBatch;
  [
    els.irr, els.irrTxt,
    els.ta, els.taTxt,
    els.ref, els.refTxt,
    els.vaz, els.vazTxt,
    els.kp, els.kpTxt,
    els.ki, els.kiTxt,
    els.kd, els.kdTxt,
    els.tf, els.tfTxt,
  ].forEach(el => {
    if (el) el.disabled = simControlsDisabled;
  });

  if (timeMode === 'real') {
    const isRunning = (realtimeState === 'running');
    const isPaused  = (realtimeState === 'paused');
    els.start.textContent = isRunning
      ? 'Pausar (Tempo Real)'
      : (isPaused ? 'Retomar (Tempo Real)' : 'Iniciar (Tempo Real)');
    els.start.classList.toggle('btn-danger', isRunning);
    els.start.classList.toggle('btn-primary', !isRunning);
  } else {
    els.start.textContent = 'Iniciar Simulação';
    els.start.classList.remove('btn-danger');
    els.start.classList.add('btn-primary');
  }
}

/* ======= Charts ======= */
function newChart(canvas, yLabel, labels, xLabel) {
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
        borderDash: (l === 'Referência' ? [5, 5] : [])
      }))
    },
    options: {
      animation: false,
      responsive: true,
      scales: {
        x: { title: { display: true, text: xLabel } },
        y: { title: { display: true, text: yLabel } }
      },
      plugins: {
        // Title (left) and subtitle (right) to keep them on the same top bar
        title: {
          display: true,
          text: yLabel,
          align: 'start'
        },
        subtitle: {
          display: true,
          text: xLabel,
          align: 'end'
        },
        // Legend moved to the right and stacked vertically to free vertical chart space
        legend: {
          position: 'right',
          align: 'center',
          labels: { usePointStyle: false }
        },
        tooltip: { mode: 'nearest', intersect: false }
      }
    }
  });
}

function initCharts() {
  Object.values(charts).forEach(c => c?.destroy?.());

  const xLabel = (timeMode === 'real') ? 't (s)' : 'Hora (HH:MM)';

  charts.temp   = newChart(els.chartTemp,  'Temperatura (°C)',  ['Tin','Tout','Tp','Referência'], xLabel);
  charts.vazao  = newChart(els.chartVazao, 'Vazão (% nominal)', ['Vazão %'], xLabel);
  charts.power  = newChart(els.chartPower, 'Potência (W)',      ['Qsol','Qloss','Qpf'], xLabel);
  charts.erro   = newChart(els.chartErro,  'Erro (°C)',         ['Erro (Tout−Ref)'], xLabel);

  const ctx = els.chartIrrTa.getContext('2d');
  charts.irrta = new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: '☀️ Irradiação (W/m²)', data: [], yAxisID: 'y', borderWidth: 2, tension: 0.22, pointRadius: 0 },
        { label: 'Ta (°C)', data: [], yAxisID: 'y1', borderWidth: 2, tension: 0.22, pointRadius: 0 }
      ]
    },
    options: {
      responsive: true,
      animation: false,
      scales: {
        x: { title: { display: true, text: xLabel } },
        y: { position: 'left', title: { display: true, text: 'Irr (W/m²)' } },
        y1: { position: 'right', grid: { drawOnChartArea: false }, title: { display: true, text: 'Ta (°C)' } }
      },
      plugins: { legend: { position: 'bottom' } }
    }
  });
}

function renderAllCharts(series) {
  const set = (chart, arrs) => {
    chart.data.labels = series.labels;
    chart.data.datasets.forEach((ds, i) => ds.data = arrs[i]);
    chart.update('none');
  };

  set(charts.temp,  [series.Tin, series.Tout, series.Tp, series.Ref]);
  set(charts.vazao, [series.vazPct]);
  set(charts.power, [series.Qsol, series.Qloss, series.Qpf]);
  set(charts.erro,  [series.erro]);

  charts.irrta.data.labels = series.labels;
  charts.irrta.data.datasets[0].data = series.Irr;
  charts.irrta.data.datasets[1].data = series.Ta;
  charts.irrta.update('none');
}

function pushPoint(chart, label, arrs) {
  const lbls = chart.data.labels;
  lbls.push(label);
  chart.data.datasets.forEach((ds, i) => ds.data.push(arrs[i]));

  chart.update('quiet');
}

function updateKPIs(s, reference) {
  els.Tin.textContent = fmt(s.Tin, 2);
  els.Tout.textContent = fmt(s.Tout, 2);
  els.Tp.textContent = fmt(s.Tp, 2);
  els.vazaoPct.textContent = fmt(s.vazaoPct, 1) + ' %';
  els.eta.textContent = fmt(s.eta * 100, 1) + ' %';
  els.dT.textContent = fmt(s.dT, 2) + ' °C';
  els.mdot.textContent = fmt(s.vazaoVol_Lmin, 2) + ' L/min';
  els.refNow.textContent = fmt(reference, 1) + ' °C';
  els.errNow.textContent = fmt(s.erro, 2) + ' °C';
}

/* ======= Relógio ======= */
function updateClock() {
  if (timeMode === 'real') {
    const now = new Date();
    const hh = String(now.getHours()).padStart(2,'0');
    const mm = String(now.getMinutes()).padStart(2,'0');
    const ss = String(now.getSeconds()).padStart(2,'0');
    els.clockNow.textContent = `${hh}:${mm}:${ss} (real)`;
  } else {
    els.clockNow.textContent = `${els.simStart.value || '—'} → ${els.simEnd.value || '—'} (simulado)`;
  }
}

/* ======= Instância ======= */
function createSimulatorInstance() {
  return new ColetorSolarJS({
    irradiacao_solar: readControlValue(els.irr, els.irrTxt),
    temperaturaAmbiente: readControlValue(els.ta, els.taTxt),
    porcentagem_vazao: readControlValue(els.vaz, els.vazTxt),
    referencia: readControlValue(els.ref, els.refTxt),
    pid: {
      kp: readControlValue(els.kp, els.kpTxt),
      ki: readControlValue(els.ki, els.kiTxt),
      kd: readControlValue(els.kd, els.kdTxt),
      tf: readControlValue(els.tf, els.tfTxt)
    },
    consts: { ...C }
  });
}

/* ======= Simulação ======= */
function runSweep() {
  els.status.textContent = '▶️ Executando varredura...';
  updateClock();
  initCharts();

  const simLocal = createSimulatorInstance();

  const labels = [];
  const Tin = [], Tout = [], Tp = [], Ref = [];
  const vazPct = [], Qsol = [], Qloss = [], Qpf = [];
  const erro = [], Irr = [], Ta = [];

  const parsedStart = parseTimeToSeconds(els.simStart.value);
  const parsedEnd = parseTimeToSeconds(els.simEnd.value);
  const startSec = Number.isFinite(parsedStart) ? parsedStart : 8 * 3600;
  const endSec = Number.isFinite(parsedEnd) ? parsedEnd : 18 * 3600;
  const stepVal = Number(els.simStep.value);
  const stepSec = Math.max(1, Number.isFinite(stepVal) ? stepVal : 60);

  const totalSec = Math.max(stepSec, endSec - startSec);
  const stepsOuter = Math.max(1, Math.floor(totalSec / stepSec));
  let tsec = startSec;

  let lastState = null;

  for (let k = 0; k <= stepsOuter; k++) {
    const prof = (csvTable.length > 0)
      ? interpolateProfile(csvTable, tsec)
      : defaultSolarProfile(tsec, startSec, endSec);

    simLocal.setIrradiacao(prof.irr);
    simLocal.setTa(prof.ta);

    const subSteps = Math.max(1, Math.floor(stepSec / simLocal.DELTA_T));
    for (let j = 0; j < subSteps; j++) {
      const s = simLocal.stepOnce();
      simLocal.controleVazao(s.Tout);
      lastState = s;
    }

    if (lastState) {
      labels.push(secondsToLabel(tsec, stepSec));
      Tin.push(lastState.Tin);
      Tout.push(lastState.Tout);
      Tp.push(lastState.Tp);
      Ref.push(simLocal.REFERENCIA);
      vazPct.push(lastState.vazaoPct);
      Qsol.push(lastState.Qsol);
      Qloss.push(lastState.Qloss);
      Qpf.push(lastState.Qpf);
      erro.push(lastState.erro);
      Irr.push(simLocal.irradiacao_solar);
      Ta.push(simLocal.TEMPERATURA_AMBIENTE);
    }

    tsec += stepSec;
    if (tsec > endSec) tsec = endSec;
  }

  if (lastState) updateKPIs(lastState, readControlValue(els.ref, els.refTxt));

  renderAllCharts({ labels, Tin, Tout, Tp, vazPct, Qsol, Qloss, Qpf, erro, Irr, Ta, Ref });

  els.status.textContent = '✅ Varredura concluída';
  uiRunning(false);
}

function startRealtimeFresh() {
  initCharts();
  sim = createSimulatorInstance();
  realtimeElapsedSec = 0;

  const s0 = sim.currentState();
  const label0 = fmt(0, 1);
  pushPoint(charts.temp,  label0, [s0.Tin, s0.Tout, s0.Tp, sim.REFERENCIA]);
  pushPoint(charts.vazao, label0, [s0.vazaoPct]);
  pushPoint(charts.power, label0, [s0.Qsol, s0.Qloss, s0.Qpf]);
  pushPoint(charts.erro,  label0, [s0.erro]);
  pushPoint(charts.irrta, label0, [sim.irradiacao_solar, sim.TEMPERATURA_AMBIENTE]);
  updateKPIs(s0, sim.REFERENCIA);

  els.status.textContent = '▶️ Executando em tempo real...';
  realtimeState = 'running';
  uiRunning(false);

  timer = setInterval(tickRealtime, sim.DELTA_T * 1000);
}

function resumeRealtime() {
  if (!sim) return startRealtimeFresh();

  els.status.textContent = '▶️ Retomado (tempo real)...';
  realtimeState = 'running';
  uiRunning(false);

  timer = setInterval(tickRealtime, sim.DELTA_T * 1000);
}

function pauseRealtime() {
  if (timer) clearInterval(timer);
  timer = null;

  if (realtimeState === 'running') realtimeState = 'paused';
  uiRunning(false);
  els.status.textContent = '⏸️ Pausado (tempo real).';
}

function tickRealtime() {
  if (!sim) return;

  sim.setIrradiacao(readControlValue(els.irr, els.irrTxt));
  sim.setTa(readControlValue(els.ta, els.taTxt));
  sim.setReferencia(readControlValue(els.ref, els.refTxt));
  sim.setPorcentagemVazao(readControlValue(els.vaz, els.vazTxt));
  sim.setGains({
    kp: readControlValue(els.kp, els.kpTxt),
    ki: readControlValue(els.ki, els.kiTxt),
    kd: readControlValue(els.kd, els.kdTxt),
    tf: readControlValue(els.tf, els.tfTxt)
  });

  const s = sim.stepOnce();
  sim.controleVazao(s.Tout);

  realtimeElapsedSec += sim.DELTA_T;
  const label = fmt(realtimeElapsedSec, 1);

  pushPoint(charts.temp,  label, [s.Tin, s.Tout, s.Tp, sim.REFERENCIA]);
  pushPoint(charts.vazao, label, [s.vazaoPct]);
  pushPoint(charts.power, label, [s.Qsol, s.Qloss, s.Qpf]);
  pushPoint(charts.erro,  label, [s.erro]);
  pushPoint(charts.irrta, label, [sim.irradiacao_solar, sim.TEMPERATURA_AMBIENTE]);

  updateKPIs(s, sim.REFERENCIA);
  updateClock();
}

/* ======= Ações ======= */
function startSimulation() {
  if (timeMode === 'sim') {
    uiRunning(true);
    setTimeout(() => runSweep(), 10);
    return;
  }

  if (realtimeState === 'stopped') {
    startRealtimeFresh();
  } else if (realtimeState === 'running') {
    pauseRealtime();
  } else if (realtimeState === 'paused') {
    resumeRealtime();
  }

  uiRunning(false);
}
els.start.addEventListener('click', startSimulation);

els.reset.addEventListener('click', () => {
  pauseRealtime();
  sim = null;
  realtimeState = 'stopped';
  realtimeElapsedSec = 0;

  initCharts();
  updateLabels();
  updateClock();

  ['Tin','Tout','Tp','vazao_pct','eta','dT','mdot','refNow','errNow'].forEach(id => {
    if (els[id]) els[id].textContent = '—';
  });

  els.status.textContent = 'Pronto';
  uiRunning(false);
});

/* ======= Modal Constantes ======= */
function num(sel, fallback) {
  const v = Number(q(sel).value);
  return Number.isFinite(v) ? v : fallback;
}

function clampInt(v, lo, hi) {
  v = Math.round(v);
  return Math.min(hi, Math.max(lo, v));
}

function loadConstsToModal() {
  q('#c_Ac').value = C.Ac;
  q('#c_U').value = C.U;
  q('#c_tau').value = C.tauAlpha;
  q('#c_Nseg').value = C.NSEG;
  q('#c_Vcol').value = C.V_col;
  q('#c_mp').value = C.mp;
  q('#c_cpp').value = C.cpp;
  q('#c_hA').value = C.hA_total;
  q('#c_UAt').value = C.UA_tubos_total;
  q('#c_Vloop').value = C.V_loop;
  q('#c_UAl').value = C.UA_loop;
  q('#c_dt').value = C.DELTA_T;
  q('#c_vazN').value = C.vazaoNominal;
}

function saveConstsFromModal() {
  C.Ac = num('#c_Ac', C.Ac);
  C.U = num('#c_U', C.U);
  C.tauAlpha = num('#c_tau', C.tauAlpha);
  C.NSEG = clampInt(num('#c_Nseg', C.NSEG), 1, 200);
  C.V_col = num('#c_Vcol', C.V_col);
  C.mp = num('#c_mp', C.mp);
  C.cpp = num('#c_cpp', C.cpp);
  C.hA_total = num('#c_hA', C.hA_total);
  C.UA_tubos_total = num('#c_UAt', C.UA_tubos_total);
  C.V_loop = num('#c_Vloop', C.V_loop);
  C.UA_loop = num('#c_UAl', C.UA_loop);
  C.DELTA_T = Math.max(0.05, num('#c_dt', C.DELTA_T));
  C.vazaoNominal = Math.max(0.01, num('#c_vazN', C.vazaoNominal));

  constModalInstance.hide();
  els.status.textContent = '🔧 Constantes aplicadas. Reset recomendado (para recomeçar do zero).';

  initCharts();
}

els.constBtn.addEventListener('click', () => {
  loadConstsToModal();
  constModalInstance.show();
});
els.closeConst.addEventListener('click', () => constModalInstance.hide());
els.cancelConst.addEventListener('click', () => constModalInstance.hide());
els.applyConst.addEventListener('click', saveConstsFromModal);

/* ======= Menus “Em breve...” ======= */
document.querySelectorAll('[data-coming-soon]').forEach(el => {
  el.addEventListener('click', (e) => {
    e.preventDefault();
    toast('Em breve…');
  });
});

function toast(msg) {
  els.toast.textContent = msg;
  els.toast.classList.remove('hidden');
  setTimeout(() => els.toast.classList.add('hidden'), 1600);
}

/* ======= Boot ======= */
function boot() {
  updateLabels();
  updateClock();
  initCharts();

  if (els.constModal) {
    constModalInstance = new bootstrap.Modal(els.constModal);
  }

  uiRunning(false);
}
boot();