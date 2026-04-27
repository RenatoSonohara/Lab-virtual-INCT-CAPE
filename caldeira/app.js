/**
 * ============================
 * Simulador Caldeira — UFSC
 * Estratégias de controle:
 * - Clássica: nível e pressão com desacoplamento entre W e Q
 * - Nível + feed forward: dois PIDs atuando em ΔW
 * ============================
 */

const clamp = (value, min, max) => Math.max(min, Math.min(max, value));

class CaldeiraJS {
  constructor(opts) {
    const {
      W,
      St,
      Q,
      refL,
      refP,
      consts,
      gainsClassic,
      gainsLevel,
      gainsFeedForward,
      controllerMode,
      enabledDynamics,
    } = opts;

    this.DELTA_T = consts.DELTA_T;
    this.controllerMode = controllerMode || 'classic';

    this.W = W;
    this.St = St;
    this.Q = Q;
    this.refL = refL;
    this.refP = refP;

    this.enabledDynamics = {
      Lf: true,
      Ls: true,
      LQ: true,
      Pf: true,
      Ps: true,
      PQ: true,
      ...(enabledDynamics || {}),
    };

    // Disturbance channels are modeled as deltas; start previous samples at zero
    // so an initial non-zero setting is treated as a valid step.
    this._StPrev = 0;
    this._QPrev = 0;

    this._resetPlantStates();
    this._resetControlStates();

    this.Kp1 = gainsClassic?.kp ?? 0;
    this.Ki1 = gainsClassic?.ki ?? 0;
    this.Kd1 = gainsClassic?.kd ?? 0;

    this.Kp2 = gainsClassic?.kp2 ?? 0;
    this.Ki2 = gainsClassic?.ki2 ?? 0;
    this.Kd2 = gainsClassic?.kd2 ?? 0;

    this.KpCtrl = gainsLevel?.kp ?? 0;
    this.KiCtrl = gainsLevel?.ki ?? 0;
    this.KdCtrl = gainsLevel?.kd ?? 0;

    this.KpFF = gainsFeedForward?.kp ?? 0;
    this.KiFF = gainsFeedForward?.ki ?? 0;
    this.KdFF = gainsFeedForward?.kd ?? 0;

    // Dinâmica de pressão: dP/dt = -311*W -1338*St + 0.0006765*Q
    this.desacoplador = {
      gPw: -311,
      gPs: -1338,
      gPq: 0.0006765,
    };

    this.u1_apos_desacoplador = W;
    this.u2_apos_desacoplador = Q;
    this.controlePrincipal = 0;
    this.controleSecundario = 0;
  }

  _resetPlantStates() {
    this.Lf = 0; this.Lfd = 0;
    this.Ls = 0; this.Lsd = 0;
    this.LQ = 0; this.LQd = 0;
    this.Pf = 0;
    this.Ps = 0;
    this.PQ = 0;
  }

  _resetControlStates() {
    this.acaoIntegral1 = 0;
    this.acaoIntegral2 = 0;
    this.acaoIntegralCtrl = 0;
    this.acaoIntegralFF = 0;
    this._lastL = 0;
    this._lastP = 0;
    this._lastCtrlL = 0;
    this._lastFFSt = 0;
    this.sinalControle1 = 0;
    this.sinalControle2 = 0;
    this.controlePrincipal = 0;
    this.controleSecundario = 0;
  }

  setControllerMode(mode) {
    this.controllerMode = mode === 'levelff' ? 'levelff' : 'classic';
    this._resetControlStates();
  }

  setDynamicsEnabled(key, enabled) {
    if (!(key in this.enabledDynamics)) return;
    this.enabledDynamics[key] = Boolean(enabled);
    if (!enabled) {
      this[ key ] = 0;
      this[`${key}d`] = 0;
    }
  }

  controleNivel(L) {
    const erro = this.refL - L;
    const dMeas = (L - this._lastL) / this.DELTA_T;
    this._lastL = L;

    this.acaoIntegral1 += this.Ki1 * erro * this.DELTA_T;
    const u = this.Kp1 * erro + this.acaoIntegral1 - this.Kd1 * dMeas;

    this.sinalControle1 = u;
    this.controlePrincipal = u;
    return erro;
  }

  controlePressao(P) {
    const erro = this.refP - P;
    const dMeas = (P - this._lastP) / this.DELTA_T;
    this._lastP = P;

    this.acaoIntegral2 += this.Ki2 * erro * this.DELTA_T;
    const u = this.Kp2 * erro + this.acaoIntegral2 - this.Kd2 * dMeas;

    this.sinalControle2 = u;
    this.controleSecundario = u;
    return erro;
  }

  controleNivelFF(L) {
    const erro = this.refL - L;
    const dMeas = (L - this._lastCtrlL) / this.DELTA_T;
    this._lastCtrlL = L;

    this.acaoIntegralCtrl += this.KiCtrl * erro * this.DELTA_T;
    const u = this.KpCtrl * erro + this.acaoIntegralCtrl - this.KdCtrl * dMeas;

    this.sinalControle1 = u;
    this.controlePrincipal = u;
    return erro;
  }

  controleFeedForward(St) {
    const erro = 0 - St;
    const dMeas = (St - this._lastFFSt) / this.DELTA_T;
    this._lastFFSt = St;

    this.acaoIntegralFF += this.KiFF * erro * this.DELTA_T;
    const u = this.KpFF * erro + this.acaoIntegralFF - this.KdFF * dMeas;

    this.sinalControle2 = u;
    this.controleSecundario = u;
    return erro;
  }

  aplicarDesacoplador() {
    const u1 = this.sinalControle1;
    const u2 = this.sinalControle2;
    const { gPw, gPs, gPq } = this.desacoplador;

    const p_c1 = u1;
    const p_c2 = (u2 - gPw * p_c1 - gPs * this.St) / gPq;

    this.u1_apos_desacoplador = p_c1;
    this.u2_apos_desacoplador = p_c2;

    this.W = this.u1_apos_desacoplador;
    this.Q = this.u2_apos_desacoplador;
  }

  aplicarControleNivelFF() {
    const deltaW = this.sinalControle1 + this.sinalControle2;
    this.u1_apos_desacoplador = deltaW;
    this.u2_apos_desacoplador = this.Q;
    this.W = deltaW;
  }

  resetPIDs() {
    this._resetControlStates();
  }

  currentState() {
    const L = this.Lf + this.Ls + this.LQ;
    const P = this.Pf + this.Ps + this.PQ;

    return {
      L,
      P,
      Lf: this.Lf,
      Ls: this.Ls,
      LQ: this.LQ,
      Pf: this.Pf,
      Ps: this.Ps,
      PQ: this.PQ,
      W: this.W,
      St: this.St,
      Q: this.Q,
      sinalControle1: this.sinalControle1,
      sinalControle2: this.sinalControle2,
      controlePrincipal: this.controlePrincipal,
      controleSecundario: this.controleSecundario,
      u1_apos_desacoplador: this.u1_apos_desacoplador,
      u2_apos_desacoplador: this.u2_apos_desacoplador,
      p_c1: this.u1_apos_desacoplador,
      p_c2: this.u2_apos_desacoplador,
      erroL: this.refL - L,
      erroP: this.refP - P,
    };
  }

  stepOnce() {
    const dt = this.DELTA_T;
    const W = this.W;
    const St = this.St;
    const Q = this.Q;

    const Std = (St - this._StPrev) / dt;
    const Qd = (Q - this._QPrev) / dt;

    if (this.enabledDynamics.Lf) {
      const Lf_dd = (-1 / 8.666) * this.Lfd + (0.000134 / 8.666) * W;
      this.Lfd += Lf_dd * dt;
      this.Lf += this.Lfd * dt;
    } else {
      this.Lf = 0;
      this.Lfd = 0;
    }

    if (this.enabledDynamics.Ls) {
      const Ls_dd = (-1 / 1.755) * this.Lsd + (0.03932 / 1.755) * Std - (0.0001515 / 1.755) * St;
      this.Lsd += Ls_dd * dt;
      this.Ls += this.Lsd * dt;
    } else {
      this.Ls = 0;
      this.Lsd = 0;
    }

    if (this.enabledDynamics.LQ) {
      const LQ_dd = (-1 / 7.878) * this.LQd - (5e-9 / 7.878) * Qd - (1.15e-11 / 7.878) * Q;
      this.LQd += LQ_dd * dt;
      this.LQ += this.LQd * dt;
    } else {
      this.LQ = 0;
      this.LQd = 0;
    }

    if (this.enabledDynamics.Pf) this.Pf += (-311 * W) * dt;
    else this.Pf = 0;

    if (this.enabledDynamics.Ps) this.Ps += (-1338 * St) * dt;
    else this.Ps = 0;

    if (this.enabledDynamics.PQ) this.PQ += (0.0006765 * Q) * dt;
    else this.PQ = 0;

    const L = this.Lf + this.Ls + this.LQ;
    const P = this.Pf + this.Ps + this.PQ;

    this._StPrev = St;
    this._QPrev = Q;

    return {
      L,
      P,
      Lf: this.Lf,
      Ls: this.Ls,
      LQ: this.LQ,
      Pf: this.Pf,
      Ps: this.Ps,
      PQ: this.PQ,
      W,
      St,
      Q,
      sinalControle1: this.sinalControle1,
      sinalControle2: this.sinalControle2,
      controlePrincipal: this.controlePrincipal,
      controleSecundario: this.controleSecundario,
      u1_apos_desacoplador: this.u1_apos_desacoplador,
      u2_apos_desacoplador: this.u2_apos_desacoplador,
      p_c1: this.u1_apos_desacoplador,
      p_c2: this.u2_apos_desacoplador,
      erroL: this.refL - L,
      erroP: this.refP - P,
    };
  }

  setW(v) { this.W = v; }
  setSt(v) { this.St = v; }
  setQ(v) { this.Q = v; }
  setRefL(v) { this.refL = v; }
  setRefP(v) { this.refP = v; }

  setGainsClassic({ kp, ki, kd, kp2, ki2, kd2 }) {
    if (kp !== undefined) this.Kp1 = kp;
    if (ki !== undefined) this.Ki1 = ki;
    if (kd !== undefined) this.Kd1 = kd;
    if (kp2 !== undefined) this.Kp2 = kp2;
    if (ki2 !== undefined) this.Ki2 = ki2;
    if (kd2 !== undefined) this.Kd2 = kd2;
  }

  setGainsLevel({ kp, ki, kd }) {
    if (kp !== undefined) this.KpCtrl = kp;
    if (ki !== undefined) this.KiCtrl = ki;
    if (kd !== undefined) this.KdCtrl = kd;
  }

  setGainsFeedForward({ kp, ki, kd }) {
    if (kp !== undefined) this.KpFF = kp;
    if (ki !== undefined) this.KiFF = ki;
    if (kd !== undefined) this.KdFF = kd;
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

  W: q('#W'),
  St: q('#St'),
  Q: q('#Q'),
  refP: q('#refP'),
  refL: q('#refL'),

  kp1: q('#kp1'),
  ki1: q('#ki1'),
  kd1: q('#kd1'),
  kp2: q('#kp2'),
  ki2: q('#ki2'),
  kd2: q('#kd2'),

  kpCtrl: q('#kpCtrl'),
  kiCtrl: q('#kiCtrl'),
  kdCtrl: q('#kdCtrl'),
  kpFF: q('#kpFF'),
  kiFF: q('#kiFF'),
  kdFF: q('#kdFF'),

  W_txt: q('#W_txt'),
  St_txt: q('#St_txt'),
  Q_txt: q('#Q_txt'),
  refP_txt: q('#refP_txt'),
  refL_txt: q('#refL_txt'),
  kp1_txt: q('#kp1_txt'),
  ki1_txt: q('#ki1_txt'),
  kd1_txt: q('#kd1_txt'),
  kp2_txt: q('#kp2_txt'),
  ki2_txt: q('#ki2_txt'),
  kd2_txt: q('#kd2_txt'),
  kpCtrl_txt: q('#kpCtrl_txt'),
  kiCtrl_txt: q('#kiCtrl_txt'),
  kdCtrl_txt: q('#kdCtrl_txt'),
  kpFF_txt: q('#kpFF_txt'),
  kiFF_txt: q('#kiFF_txt'),
  kdFF_txt: q('#kdFF_txt'),

  loopModeToggle: q('#loopModeToggle'),
  loopModeLabel: q('#loopModeLabel'),
  controllerModeBtn: q('#controllerModeBtn'),
  controllerClassicBtn: q('#controllerClassicBtn'),
  controllerLevelFFBtn: q('#controllerLevelFFBtn'),
  controllerSummaryBadge: q('#controllerSummaryBadge'),
  controllerSummaryTitle: q('#controllerSummaryTitle'),
  controllerSummaryText: q('#controllerSummaryText'),
  controllerPrimaryRoleKicker: q('#controllerPrimaryRoleKicker'),
  controllerPrimaryRoleTitle: q('#controllerPrimaryRoleTitle'),
  controllerPrimaryRoleDesc: q('#controllerPrimaryRoleDesc'),
  controllerSecondaryRoleKicker: q('#controllerSecondaryRoleKicker'),
  controllerSecondaryRoleTitle: q('#controllerSecondaryRoleTitle'),
  controllerSecondaryRoleDesc: q('#controllerSecondaryRoleDesc'),
  strategyClassicPanel: q('#strategyClassicPanel'),
  strategyLevelFFPanel: q('#strategyLevelFFPanel'),
  applyIdealGains: q('#applyIdealGainsBtn'),

  start: q('#startBtn'),
  reset: q('#resetBtn'),
  runBatch: q('#runBatchBtn'),
  cancelBatch: q('#cancelBatchBtn'),
  simDuration: q('#simDuration'),

  kpi_L: q('#kpi_L'),
  kpi_P: q('#kpi_P'),
  kpi_u1: q('#kpi_u1'),
  kpi_u2: q('#kpi_u2'),
  kpi_refP: q('#kpi_refP'),
  kpi_refL: q('#kpi_refL'),

  chartL: q('#chartL'),
  chartP: q('#chartP'),
  chartEntradas: q('#chartEntradas'),
  chartControles: q('#chartControles'),

  toast: q('#toast'),
};

const DEFAULTS = {
  controllerMode: 'classic',
  W: 1,
  St: 1,
  Q: 1,
  refP: 0,
  refL: 0,
  kp1: 0,
  ki1: 0,
  kd1: 0,
  kp2: 0,
  ki2: 0,
  kd2: 0,
  kpCtrl: 0,
  kiCtrl: 0,
  kdCtrl: 0,
  kpFF: 0,
  kiFF: 0,
  kdFF: 0,
};

const IDEAL_GAINS = {
  kp1: 1.2,
  ki1: 0.04,
  kd1: 0.35,
  kp2: 0.22,
  ki2: 0.01,
  kd2: 0.08,
};

const CONTROL_MODES = {
  classic: {
    label: 'Controle clássico',
    button: 'Estratégia: Controle clássico',
    panel: 'strategyClassicPanel',
    chartLabels: ['PID de nível', 'PID de pressão', 'ΔF aplicada', 'ΔQ aplicada'],
    summaryBadge: 'Controle clássico',
    summaryTitle: 'PID de nível e PID de pressão',
    summaryText: 'A malha clássica usa dois PIDs e o desacoplador entre W e Q.',
    primaryRoleKicker: 'PID principal',
    primaryRoleTitle: 'PID 1 — Controle de NÍVEL L via W',
    primaryRoleDesc: 'Fecha a malha de nível usando a referência e a saída da caldeira.',
    secondaryRoleKicker: 'PID auxiliar',
    secondaryRoleTitle: 'PID 2 — Controle de PRESSÃO P via Q',
    secondaryRoleDesc: 'Compensa a pressão e atua junto ao desacoplador.',
  },
  levelff: {
    label: 'Nível + feed forward',
    button: 'Estratégia: Nível + feed forward',
    panel: 'strategyLevelFFPanel',
    chartLabels: ['Controlador de nível', 'Feed forward', 'ΔF aplicada', 'ΔQ aplicada'],
    summaryBadge: 'Nível + feed forward',
    summaryTitle: 'PID de nível + PID feed forward',
    summaryText: 'O controlador principal fecha a malha de nível e o feed forward corrige ΔSt em ΔW.',
    primaryRoleKicker: 'Controlador',
    primaryRoleTitle: 'PID de NÍVEL L via ΔF',
    primaryRoleDesc: 'Lê L e a referência para ajustar ΔF.',
    secondaryRoleKicker: 'Controlador feed forward',
    secondaryRoleTitle: 'PID via ΔV',
    secondaryRoleDesc: 'Lê ΔV e injeta correção em ΔF.',
  },
};

const PLANT_DYNAMICS_DEFAULT = {
  Lf: true,
  Ls: true,
  LQ: true,
  Pf: true,
  Ps: true,
  PQ: true,
};

let controllerMode = DEFAULTS.controllerMode;
const plantDynamics = { ...PLANT_DYNAMICS_DEFAULT };

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

  setControllerMode('classic');
  setLoopMode(true);
  if (sim) {
    sim.setGainsClassic({ kp: IDEAL_GAINS.kp1, ki: IDEAL_GAINS.ki1, kd: IDEAL_GAINS.kd1, kp2: IDEAL_GAINS.kp2, ki2: IDEAL_GAINS.ki2, kd2: IDEAL_GAINS.kd2 });
    sim.resetPIDs();
  }
  els.status.textContent = 'Ganhos conservadores aplicados (MF).';
}

function gainFromText(txtEl, sliderEl) {
  const v = Number(txtEl?.value);
  if (Number.isFinite(v)) return v;
  return Number(sliderEl?.value ?? 0);
}

function readParamValue(sliderEl, txtEl) {
  const txtV = Number(txtEl?.value);
  if (Number.isFinite(txtV)) return txtV;
  return Number(sliderEl?.value ?? 0);
}

function syncTextToSlider(slider, txt) {
  if (!slider || !txt) return;
  slider.addEventListener('input', () => { txt.value = slider.value; });
  const syncToSlider = () => {
    const v = Number(txt.value);
    if (!Number.isFinite(v)) return;
    const min = Number(slider.min);
    const max = Number(slider.max);
    slider.value = Math.max(min, Math.min(max, v));
  };
  txt.addEventListener('input', syncToSlider);
  txt.addEventListener('change', syncToSlider);
  txt.addEventListener('keydown', e => { if (e.key === 'Enter') syncToSlider(); });
}

function syncNumericPairs(pairs) {
  pairs.forEach(([slider, txt]) => syncTextToSlider(slider, txt));
}

const normalPairs = [
  [els.W, els.W_txt],
  [els.St, els.St_txt],
  [els.Q, els.Q_txt],
  [els.refP, els.refP_txt],
  [els.refL, els.refL_txt],
];

const classicGainPairs = [
  [els.kp1, els.kp1_txt],
  [els.ki1, els.ki1_txt],
  [els.kd1, els.kd1_txt],
  [els.kp2, els.kp2_txt],
  [els.ki2, els.ki2_txt],
  [els.kd2, els.kd2_txt],
];

const levelGainPairs = [
  [els.kpCtrl, els.kpCtrl_txt],
  [els.kiCtrl, els.kiCtrl_txt],
  [els.kdCtrl, els.kdCtrl_txt],
  [els.kpFF, els.kpFF_txt],
  [els.kiFF, els.kiFF_txt],
  [els.kdFF, els.kdFF_txt],
];

syncNumericPairs(normalPairs);
syncNumericPairs(classicGainPairs);
syncNumericPairs(levelGainPairs);

function uiRunning(running) {
  if (els.start) {
    els.start.disabled = false;
    els.start.textContent = running ? 'Parar' : 'Iniciar';
    els.start.classList.toggle('btn-danger', running);
    els.start.classList.toggle('btn-primary', !running);
  }
  if (els.reset) els.reset.disabled = running;
  if (els.runBatch) els.runBatch.disabled = running;
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
  if (els.cancelBatch) els.cancelBatch.disabled = !enabled;
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
    els.kpCtrl, els.kpCtrl_txt,
    els.kiCtrl, els.kiCtrl_txt,
    els.kdCtrl, els.kdCtrl_txt,
    els.kpFF, els.kpFF_txt,
    els.kiFF, els.kiFF_txt,
    els.kdFF, els.kdFF_txt,
    els.loopModeToggle,
    els.controllerModeBtn,
    els.controllerClassicBtn,
    els.controllerLevelFFBtn,
    els.applyIdealGains,
    els.simDuration,
  ];

  lockables.forEach(el => {
    if (el && 'disabled' in el) el.disabled = locked;
  });

  document.querySelectorAll('[data-dynamic-toggle]').forEach(el => {
    if ('disabled' in el) el.disabled = locked;
  });

  if (els.start) els.start.disabled = locked;
  if (els.reset) els.reset.disabled = locked;
}

function updateKPIs(s) {
  if (!s) return;
  els.kpi_L.textContent = fmt(s.L, 6) + ' m';
  els.kpi_P.textContent = fmt(s.P, 4) + ' MPa';
  els.kpi_u1.textContent = fmt(s.u1_apos_desacoplador, 2) + ' kg/s';
  els.kpi_u2.textContent = fmt(s.u2_apos_desacoplador / 1e6, 4) + ' MW';
  els.kpi_refP.textContent = fmt(readParamValue(els.refP, els.refP_txt), 2);
  els.kpi_refL.textContent = fmt(readParamValue(els.refL, els.refL_txt), 6);
}

function resetKPIs() {
  ['kpi_L', 'kpi_P', 'kpi_u1', 'kpi_u2', 'kpi_refP', 'kpi_refL'].forEach(id => {
    if (els[id]) els[id].textContent = '—';
  });
}

function setLoopMode(closedLoop) {
  if (!els.loopModeToggle || !els.loopModeLabel) return;
  els.loopModeToggle.checked = closedLoop;
  els.loopModeLabel.textContent = closedLoop ? 'Malha Fechada' : 'Malha Aberta';
}

function setControllerMode(mode) {
  const nextMode = CONTROL_MODES[mode] ? mode : 'classic';
  controllerMode = nextMode;

  if (els.controllerModeBtn) {
    els.controllerModeBtn.textContent = CONTROL_MODES[nextMode].button;
  }

  renderControllerSummary(nextMode);

  if (els.strategyClassicPanel) {
    els.strategyClassicPanel.classList.toggle('hidden', nextMode !== 'classic');
  }
  if (els.strategyLevelFFPanel) {
    els.strategyLevelFFPanel.classList.toggle('hidden', nextMode !== 'levelff');
  }

  if (els.controllerClassicBtn) {
    els.controllerClassicBtn.classList.toggle('active', nextMode === 'classic');
  }
  if (els.controllerLevelFFBtn) {
    els.controllerLevelFFBtn.classList.toggle('active', nextMode === 'levelff');
  }

  syncControlChartLabels();

  if (sim) {
    sim.setControllerMode(nextMode);
    sim.resetPIDs();
  }
}

function controlChartLabels() {
  return CONTROL_MODES[controllerMode]?.chartLabels || CONTROL_MODES.classic.chartLabels;
}

function renderControllerSummary(mode) {
  const activeMode = CONTROL_MODES[mode] ? mode : 'classic';
  const config = CONTROL_MODES[activeMode];

  if (els.controllerSummaryBadge) els.controllerSummaryBadge.textContent = config.summaryBadge;
  if (els.controllerSummaryTitle) els.controllerSummaryTitle.textContent = config.summaryTitle;
  if (els.controllerSummaryText) els.controllerSummaryText.textContent = config.summaryText;
  if (els.controllerPrimaryRoleKicker) els.controllerPrimaryRoleKicker.textContent = config.primaryRoleKicker;
  if (els.controllerPrimaryRoleTitle) els.controllerPrimaryRoleTitle.textContent = config.primaryRoleTitle;
  if (els.controllerPrimaryRoleDesc) els.controllerPrimaryRoleDesc.textContent = config.primaryRoleDesc;
  if (els.controllerSecondaryRoleKicker) els.controllerSecondaryRoleKicker.textContent = config.secondaryRoleKicker;
  if (els.controllerSecondaryRoleTitle) els.controllerSecondaryRoleTitle.textContent = config.secondaryRoleTitle;
  if (els.controllerSecondaryRoleDesc) els.controllerSecondaryRoleDesc.textContent = config.secondaryRoleDesc;
}

function syncControlChartLabels() {
  if (!charts.ctrl) return;
  const labels = controlChartLabels();
  charts.ctrl.data.datasets.forEach((ds, index) => {
    if (labels[index]) ds.label = labels[index];
  });
  charts.ctrl.update('quiet');
}

function setPlantDynamic(key, enabled) {
  if (!(key in plantDynamics)) return;
  plantDynamics[key] = Boolean(enabled);
  if (sim) sim.setDynamicsEnabled(key, enabled);
  renderPlantDynamics();
}

function renderPlantDynamics() {
  document.querySelectorAll('[data-dynamic-toggle]').forEach(button => {
    const key = button.getAttribute('data-dynamic-toggle');
    const active = key ? plantDynamics[key] : true;
    button.classList.toggle('is-commented', !active);
    button.setAttribute('aria-pressed', String(active));
    const state = button.querySelector('.plant-block-state');
    const meta = button.querySelector('.plant-block-meta');
    if (state) state.textContent = active ? 'Ativa' : 'Comentada';
    if (meta) meta.textContent = active ? (button.getAttribute('data-active-meta') || meta.textContent) : 'Entrada e saída zeradas';
    button.title = key
      ? (active
        ? `Dinâmica ${key} ativa`
        : `Dinâmica ${key} comentada: entrada e saída zeradas`)
      : '';
  });
}

// ============================================================
// CHARTS
// ============================================================
function newChart(canvas, yLabel, labels, hiddenIndices = []) {
  return new Chart(canvas.getContext('2d'), {
    type: 'line',
    data: {
      labels: [],
      datasets: labels.map((label, index) => ({
        label,
        data: [],
        borderWidth: 2,
        tension: 0.22,
        pointRadius: 0,
        pointHoverRadius: 3,
        borderDash: label.startsWith('Ref') ? [5, 5] : [],
        hidden: hiddenIndices.includes(index),
      })),
    },
    options: {
      animation: false,
      responsive: true,
      scales: {
        x: { title: { display: true, text: 'Tempo (s)' } },
        y: { title: { display: true, text: yLabel } },
      },
      plugins: {
        legend: { position: 'bottom' },
        tooltip: { mode: 'nearest', intersect: false },
      },
    },
  });
}

function initCharts() {
  Object.values(charts).forEach(chart => chart?.destroy?.());

  charts.L = newChart(els.chartL, 'Nível ΔL (m)', ['L total', 'Lf (por ΔF)', 'Ls (por ΔV)', 'LQ (por ΔQ)', 'Ref. Nível'], [1, 2, 3]);
  charts.P = newChart(els.chartP, 'Pressão ΔP (MPa)', ['P total', 'Pf (por ΔF)', 'Ps (por ΔV)', 'PQ (por ΔQ)', 'Ref. Pressão'], [1, 2, 3]);
  charts.ent = newChart(els.chartEntradas, 'Entradas', ['ΔF (kg/s)', 'ΔV (%)', 'ΔQ (MJ)']);
  charts.ctrl = newChart(els.chartControles, 'Ações de Controle', controlChartLabels());
}

function pushPoint(chart, lbl, arrs) {
  if (!chart) return;
  chart.data.labels.push(lbl);
  chart.data.datasets.forEach((dataset, index) => {
    dataset.data.push(arrs[index]);
  });
  chart.update('quiet');
}

function _pushAllCharts(tSec, s, refL, refP) {
  pushPoint(charts.L, tSec, [s.L, s.Lf, s.Ls, s.LQ, refL]);
  pushPoint(charts.P, tSec, [s.P/1e6, s.Pf/1e6, s.Ps/1e6, s.PQ/1e6, refP/1e6]);
  pushPoint(charts.ent, tSec, [s.W, s.St, s.Q / 1000]);
  pushPoint(charts.ctrl, tSec, [s.controlePrincipal ?? s.sinalControle1, s.controleSecundario ?? s.sinalControle2, s.u1_apos_desacoplador, s.u2_apos_desacoplador / 1000]);
}

// ============================================================
// ESTADO GLOBAL
// ============================================================
let charts = {};
let timer = null;
let sim = null;
const MAX_POINTS = 1200;
let _stepCount = 0;
let _batchRunning = false;
let _batchCancelRequested = false;
let _batchTimer = null;

// ============================================================
// INSTÂNCIA
// ============================================================
function createSim() {
  return new CaldeiraJS({
    W: readParamValue(els.W, els.W_txt),
    St: readParamValue(els.St, els.St_txt),
    Q: readParamValue(els.Q, els.Q_txt) * 1e6,
    refL: readParamValue(els.refL, els.refL_txt),
    refP: readParamValue(els.refP, els.refP_txt),
    gainsClassic: {
      kp: gainFromText(els.kp1_txt, els.kp1),
      ki: gainFromText(els.ki1_txt, els.ki1),
      kd: gainFromText(els.kd1_txt, els.kd1),
      kp2: gainFromText(els.kp2_txt, els.kp2),
      ki2: gainFromText(els.ki2_txt, els.ki2),
      kd2: gainFromText(els.kd2_txt, els.kd2),
    },
    gainsLevel: {
      kp: gainFromText(els.kpCtrl_txt, els.kpCtrl),
      ki: gainFromText(els.kiCtrl_txt, els.kiCtrl),
      kd: gainFromText(els.kdCtrl_txt, els.kdCtrl),
    },
    gainsFeedForward: {
      kp: gainFromText(els.kpFF_txt, els.kpFF),
      ki: gainFromText(els.kiFF_txt, els.kiFF),
      kd: gainFromText(els.kdFF_txt, els.kdFF),
    },
    controllerMode,
    enabledDynamics: { ...plantDynamics },
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
  syncControlChartLabels();
  _stepCount = 0;
  sim = createSim();
  const s0 = sim.currentState();
  _pushAllCharts(0, s0, sim.refL, sim.refP);
  updateKPIs(s0);
  updateViz(s0);
  uiRunning(true);
  const intervalMs = 16;
  const stepsPerInterval = Math.max(1, Math.round(intervalMs / (C.DELTA_T * 1000)));
  timer = setInterval(() => tickRealtime(stepsPerInterval), intervalMs);
}

function stopRealtime() {
  if (timer) clearInterval(timer);
  timer = null;
  sim = null;
  uiRunning(false);
  els.status.textContent = '⏹️ Parado.';
}

function tickRealtime(stepsPerInterval) {
  if (!sim) return;
  const closedLoop = closedLoopAtivo();

  sim.setSt(readParamValue(els.St, els.St_txt));
  sim.setRefL(readParamValue(els.refL, els.refL_txt));
  sim.setRefP(readParamValue(els.refP, els.refP_txt));
  sim.setGainsClassic({
    kp: gainFromText(els.kp1_txt, els.kp1),
    ki: gainFromText(els.ki1_txt, els.ki1),
    kd: gainFromText(els.kd1_txt, els.kd1),
    kp2: gainFromText(els.kp2_txt, els.kp2),
    ki2: gainFromText(els.ki2_txt, els.ki2),
    kd2: gainFromText(els.kd2_txt, els.kd2),
  });
  sim.setGainsLevel({
    kp: gainFromText(els.kpCtrl_txt, els.kpCtrl),
    ki: gainFromText(els.kiCtrl_txt, els.kiCtrl),
    kd: gainFromText(els.kdCtrl_txt, els.kdCtrl),
  });
  sim.setGainsFeedForward({
    kp: gainFromText(els.kpFF_txt, els.kpFF),
    ki: gainFromText(els.kiFF_txt, els.kiFF),
    kd: gainFromText(els.kdFF_txt, els.kdFF),
  });

  const currentW = readParamValue(els.W, els.W_txt);
  const currentQ = readParamValue(els.Q, els.Q_txt) * 1000;

  if (!closedLoop) {
    sim.setW(currentW);
    sim.setQ(currentQ);
  } else if (controllerMode === 'levelff') {
    sim.setQ(currentQ);
  }

  let lastS;
  for (let i = 0; i < stepsPerInterval; i++) {
    lastS = sim.stepOnce();

    if (closedLoop) {
      if (controllerMode === 'classic') {
        sim.controleNivel(lastS.L);
        sim.controlePressao(lastS.P);
        sim.aplicarDesacoplador();
      } else {
        sim.controleNivelFF(lastS.L);
        sim.controleFeedForward(lastS.St);
        sim.aplicarControleNivelFF();
      }
    }

    _stepCount++;
    if (_stepCount % STEPS_PER_POINT === 0) {
      const tSec = parseFloat((_stepCount * C.DELTA_T).toFixed(1));
      _pushAllCharts(tSec, lastS, sim.refL, sim.refP);
    }
  }

  if (closedLoop && lastS) {
    if (controllerMode === 'classic') {
      els.W.value = lastS.u1_apos_desacoplador.toFixed(2);
      els.W_txt.value = lastS.u1_apos_desacoplador.toFixed(2);
      els.Q.value = (lastS.u2_apos_desacoplador / 1000).toFixed(2);
      els.Q_txt.value = (lastS.u2_apos_desacoplador / 1000).toFixed(2);
    } else {
      els.W.value = lastS.u1_apos_desacoplador.toFixed(2);
      els.W_txt.value = lastS.u1_apos_desacoplador.toFixed(2);
      els.Q.value = Number(els.Q.value).toFixed(2);
      els.Q_txt.value = Number(els.Q.value).toFixed(2);
    }
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
  syncControlChartLabels();
  resetKPIs();
  _stepCount = 0;

  const bSim = createSim();
  const s0 = bSim.currentState();
  _pushAllCharts(0, s0, bSim.refL, bSim.refP);
  const closedLoop = closedLoopAtivo();
  let i = 0;
  let lastS;

  if (!closedLoop) {
    bSim.setW(readParamValue(els.W, els.W_txt));
    bSim.setQ(readParamValue(els.Q, els.Q_txt) * 1000);
  } else if (controllerMode === 'levelff') {
    bSim.setQ(readParamValue(els.Q, els.Q_txt) * 1000);
  }

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
        if (controllerMode === 'classic') {
          bSim.controleNivel(lastS.L);
          bSim.controlePressao(lastS.P);
          bSim.aplicarDesacoplador();
        } else {
          bSim.controleNivelFF(lastS.L);
          bSim.controleFeedForward(lastS.St);
          bSim.aplicarControleNivelFF();
        }
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
  syncControlChartLabels();
  resetKPIs();
  _stepCount = 0;
  _vizL = 0;
  _vizP = 0;
  controllerMode = DEFAULTS.controllerMode;
  els.status.textContent = 'Pronto';
  uiRunning(false);
  setLoopMode(false);
  setControllerMode(DEFAULTS.controllerMode);

  const sliderMap = {
    W: els.W,
    St: els.St,
    Q: els.Q,
    refP: els.refP,
    refL: els.refL,
    kp1: els.kp1,
    ki1: els.ki1,
    kd1: els.kd1,
    kp2: els.kp2,
    ki2: els.ki2,
    kd2: els.kd2,
    kpCtrl: els.kpCtrl,
    kiCtrl: els.kiCtrl,
    kdCtrl: els.kdCtrl,
    kpFF: els.kpFF,
    kiFF: els.kiFF,
    kdFF: els.kdFF,
  };
  const txtMap = {
    W: els.W_txt,
    St: els.St_txt,
    Q: els.Q_txt,
    refP: els.refP_txt,
    refL: els.refL_txt,
    kp1: els.kp1_txt,
    ki1: els.ki1_txt,
    kd1: els.kd1_txt,
    kp2: els.kp2_txt,
    ki2: els.ki2_txt,
    kd2: els.kd2_txt,
    kpCtrl: els.kpCtrl_txt,
    kiCtrl: els.kiCtrl_txt,
    kdCtrl: els.kdCtrl_txt,
    kpFF: els.kpFF_txt,
    kiFF: els.kiFF_txt,
    kdFF: els.kdFF_txt,
  };

  Object.keys(DEFAULTS).forEach(key => {
    if (key === 'controllerMode') return;
    if (sliderMap[key]) sliderMap[key].value = DEFAULTS[key];
    if (txtMap[key]) txtMap[key].value = DEFAULTS[key];
  });

  Object.keys(plantDynamics).forEach(key => {
    plantDynamics[key] = true;
  });
  renderPlantDynamics();

  updateViz({ L: 0, P: 0, Lf: 0, Ls: 0, LQ: 0, u1_apos_desacoplador: 0, u2_apos_desacoplador: 0 });
}

// ============================================================
// VISUALIZAÇÃO DINÂMICA DA CALDEIRA
// ============================================================
const vizWaterFill = document.getElementById('waterFill');
const vizSteamFill = document.getElementById('steamFill');
const vizWaterWave = document.getElementById('waterWave');
const vizPressure = document.getElementById('vizPressure');

let _vizL = 0;
let _vizP = 0;

function updateViz(s) {
  if (!s) return;
  const drumTop = 67;
  const drumBottom = 303;
  const drumH = drumBottom - drumTop;

  const Lmin = -0.05;
  const Lmax = 0.05;
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
    vizWaterWave.setAttribute(
      'd',
      `M122,${wy} Q152,${wy - amp} 182,${wy} Q212,${wy + amp} 242,${wy} ` +
      `Q272,${wy - amp} 302,${wy} Q332,${wy + amp} 358,${wy} ` +
      `L358,${wy} L122,${wy} Z`,
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
els.start?.addEventListener('click', () => {
  timer ? stopRealtime() : startRealtime();
});

els.reset?.addEventListener('click', fullReset);

els.runBatch?.addEventListener('click', () => {
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

els.controllerClassicBtn?.addEventListener('click', () => setControllerMode('classic'));
els.controllerLevelFFBtn?.addEventListener('click', () => setControllerMode('levelff'));

els.loopModeToggle?.addEventListener('change', () => {
  setLoopMode(els.loopModeToggle.checked);
  if (sim) sim.resetPIDs();
});

const gainInputs = [
  els.kp1, els.kp1_txt, els.ki1, els.ki1_txt, els.kd1, els.kd1_txt,
  els.kp2, els.kp2_txt, els.ki2, els.ki2_txt, els.kd2, els.kd2_txt,
  els.kpCtrl, els.kpCtrl_txt, els.kiCtrl, els.kiCtrl_txt, els.kdCtrl, els.kdCtrl_txt,
  els.kpFF, els.kpFF_txt, els.kiFF, els.kiFF_txt, els.kdFF, els.kdFF_txt,
];
gainInputs.forEach(el => el?.addEventListener('input', () => setLoopMode(true)));
gainInputs.forEach(el => el?.addEventListener('change', () => setLoopMode(true)));

[els.W, els.W_txt, els.Q, els.Q_txt].forEach(el => el?.addEventListener('input', () => {
  if (controllerMode === 'classic') setLoopMode(false);
}));
[els.W, els.W_txt, els.Q, els.Q_txt].forEach(el => el?.addEventListener('change', () => {
  if (controllerMode === 'classic') setLoopMode(false);
}));

document.querySelectorAll('[data-dynamic-toggle]').forEach(button => {
  button.addEventListener('click', () => {
    const key = button.getAttribute('data-dynamic-toggle');
    if (!key) return;
    setPlantDynamic(key, !plantDynamics[key]);
  });
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
  setControllerMode(DEFAULTS.controllerMode);
  setLoopMode(false);
  setCancelBatchEnabled(false);
  renderPlantDynamics();
}

boot();
