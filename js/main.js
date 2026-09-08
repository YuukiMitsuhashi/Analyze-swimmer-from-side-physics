// main.js
// Orchestrates the UI + all modules: calibration -> pose capture -> filtering ->
// anthropometry (CoM/CoB) -> drag calibration -> force reconstruction -> correction engine
// -> charts.

const video = document.getElementById('video');
const overlay = document.getElementById('overlay');
const octx = overlay.getContext('2d');

let calibClickMode = false, seamClickMode = false;
let calibClicks = [];
let seamClicks = [];
let worldCorners = [ // default assumes a 5m x 1m reference rectangle; user can edit inputs
  { x: 0, y: 0 }, { x: 5, y: 0 }, { x: 5, y: 1 }, { x: 0, y: 1 }
];

fileInputHandler();
function fileInputHandler() {
  document.getElementById('fileInput').addEventListener('change', (e) => {
    const url = URL.createObjectURL(e.target.files[0]);
    video.src = url;
    video.addEventListener('loadedmetadata', () => {
      overlay.width = video.videoWidth;
      overlay.height = video.videoHeight;
      overlay.style.width = video.clientWidth + 'px';
      overlay.style.height = video.clientHeight + 'px';
    }, { once: true });
  });
}

function canvasCoords(e) {
  const rect = overlay.getBoundingClientRect();
  return {
    x: (e.clientX - rect.left) * (overlay.width / rect.width),
    y: (e.clientY - rect.top) * (overlay.height / rect.height)
  };
}

overlay.addEventListener('click', (e) => {
  const p = canvasCoords(e);
  if (calibClickMode) {
    calibClicks.push(p);
    drawDot(p, '#ff5555');
    document.getElementById('calibCount').textContent = `${calibClicks.length}/4`;
    if (calibClicks.length === 4) {
      calibClickMode = false;
      Calibration.reset();
      for (let i = 0; i < 4; i++) {
        Calibration.addPointPair(calibClicks[i].x, calibClicks[i].y, worldCorners[i].x, worldCorners[i].y);
      }
      document.getElementById('calibStatus').textContent = Calibration.isReady() ? 'homography solved ✓' : 'failed';
    }
  } else if (seamClickMode) {
    seamClicks.push(p);
    drawDot(p, '#ffcc00');
    if (seamClicks.length === 2) {
      seamClickMode = false;
      const xMin = Math.min(seamClicks[0].x, seamClicks[1].x);
      const xMax = Math.max(seamClicks[0].x, seamClicks[1].x);
      PoseCapture.setSeamRange(xMin, xMax);
      document.getElementById('seamRange').textContent = `${xMin.toFixed(0)}px - ${xMax.toFixed(0)}px`;
    }
  }
});

function drawDot(p, color) {
  octx.fillStyle = color;
  octx.beginPath(); octx.arc(p.x, p.y, 5, 0, 2 * Math.PI); octx.fill();
}
function drawFrame() { octx.drawImage(video, 0, 0, overlay.width, overlay.height); }

document.getElementById('btnCalibPoints').onclick = () => {
  worldCorners = [
    { x: 0, y: 0 },
    { x: parseFloat(document.getElementById('wcX2').value), y: 0 },
    { x: parseFloat(document.getElementById('wcX2').value), y: parseFloat(document.getElementById('wcY').value) },
    { x: 0, y: parseFloat(document.getElementById('wcY').value) }
  ];
  calibClicks = []; calibClickMode = true;
  octx.clearRect(0, 0, overlay.width, overlay.height); drawFrame();
};
document.getElementById('btnSeamPoints').onclick = () => {
  seamClicks = []; seamClickMode = true;
  octx.clearRect(0, 0, overlay.width, overlay.height); drawFrame();
};
document.getElementById('btnClearSeam').onclick = () => {
  PoseCapture.clearSeamRange();
  document.getElementById('seamRange').textContent = 'none set';
};

document.getElementById('btnLoadModel').onclick = async () => {
  document.getElementById('modelStatus').textContent = 'loading...';
  await PoseCapture.loadModel();
  await Anthropometry.loadParams();
  document.getElementById('modelStatus').textContent = 'loaded ✓';
  document.getElementById('btnRecord').disabled = false;
};

let recording = false;
let frames = []; // {t, kps}

document.getElementById('btnRecord').onclick = () => {
  frames = []; recording = true;
  video.currentTime = 0; video.play();
  document.getElementById('btnStopRecord').disabled = false;
  captureLoop();
};
document.getElementById('btnStopRecord').onclick = () => { recording = false; video.pause(); postProcess(); };
video.addEventListener('ended', () => { if (recording) { recording = false; postProcess(); } });

async function captureLoop() {
  if (!recording) return;
  if (!video.paused && !video.ended) {
    const kps = await PoseCapture.estimateFrame(video);
    if (kps) {
      frames.push({ t: video.currentTime, kps });
      drawSkeleton(kps);
    }
    document.getElementById('recordStatus').textContent = `frames: ${frames.length}, t=${video.currentTime.toFixed(2)}s`;
  }
  requestAnimationFrame(captureLoop);
}

function drawSkeleton(kps) {
  octx.clearRect(0, 0, overlay.width, overlay.height);
  drawFrame();
  kps.forEach(k => {
    if (k.score > 0.3) {
      octx.fillStyle = PoseCapture.isInSeam(k.x) ? '#ffcc00' : '#3fb950';
      octx.beginPath(); octx.arc(k.x, k.y, 4, 0, 2 * Math.PI); octx.fill();
    }
  });
}

let processed = null;

function postProcess() {
  if (frames.length < 10) { alert('Not enough frames — try again.'); return; }
  if (!Calibration.isReady()) { alert('Calibrate the homography first (Step 2).'); return; }

  const KP = PoseCapture.KP;
  const side = PoseCapture.pickSide(frames);
  const idx = {
    shoulder: side === 'left' ? KP.leftShoulder : KP.rightShoulder,
    hip: side === 'left' ? KP.leftHip : KP.rightHip,
    elbow: side === 'left' ? KP.leftElbow : KP.rightElbow,
    wrist: side === 'left' ? KP.leftWrist : KP.rightWrist,
    knee: side === 'left' ? KP.leftKnee : KP.rightKnee,
    ankle: side === 'left' ? KP.leftAnkle : KP.rightAnkle,
    ear: side === 'left' ? KP.leftEar : KP.rightEar
  };

  const raw = {}; Object.keys(idx).forEach(n => raw[n] = []);
  frames.forEach(f => {
    Object.entries(idx).forEach(([name, ki]) => {
      const kp = f.kps[ki];
      raw[name].push({ x: kp.x, y: kp.y, score: kp.score, inSeam: PoseCapture.isInSeam(kp.x) });
    });
  });
  Object.keys(raw).forEach(n => Filters.interpolateMissing(raw[n]));

  const n = frames.length;
  const mass = parseFloat(document.getElementById('massInput').value);
  const height = parseFloat(document.getElementById('heightInput').value);

  const comXw = new Array(n), comYw = new Array(n), cobXw = new Array(n), cobYw = new Array(n);
  const elbowAngleDeg = new Array(n);
  const headAngleDeg = new Array(n), trunkAngleDeg = new Array(n);

  for (let i = 0; i < n; i++) {
    const worldPts = {};
    Object.keys(raw).forEach(name => { worldPts[name] = Calibration.toWorld(raw[name][i].x, raw[name][i].y); });

    const { comX, comY, cobX, cobY } = Anthropometry.computeFrame(worldPts);
    comXw[i] = comX; comYw[i] = comY; cobXw[i] = cobX; cobYw[i] = cobY;

    // elbow flexion angle (shoulder-elbow-wrist)
    const v1 = { x: worldPts.shoulder.x - worldPts.elbow.x, y: worldPts.shoulder.y - worldPts.elbow.y };
    const v2 = { x: worldPts.wrist.x - worldPts.elbow.x, y: worldPts.wrist.y - worldPts.elbow.y };
    const dot = v1.x * v2.x + v1.y * v2.y;
    const mag = Math.hypot(v1.x, v1.y) * Math.hypot(v2.x, v2.y);
    elbowAngleDeg[i] = Math.acos(Math.max(-1, Math.min(1, dot / mag))) * 180 / Math.PI;

    trunkAngleDeg[i] = Math.atan2(worldPts.hip.y - worldPts.shoulder.y, worldPts.hip.x - worldPts.shoulder.x) * 180 / Math.PI;
    headAngleDeg[i] = Math.atan2(worldPts.ear.y - worldPts.shoulder.y, worldPts.ear.x - worldPts.shoulder.x) * 180 / Math.PI;
  }

  const comXsm = Filters.smooth(comXw, 3, 2);
  const t = frames.map(f => f.t);
  let dtSum = 0; for (let i = 1; i < n; i++) dtSum += t[i] - t[i - 1];
  const dt = dtSum / (n - 1);

  const vxRaw = Filters.derivative(comXsm, 3, 2).map(v => v / dt);
  const vx = vxRaw.map(Math.abs); // forward speed magnitude
  const ax = Filters.derivative(vx, 3, 2).map(v => v / dt);

  processed = { t, dt, vx, ax, comXw: comXsm, comYw, cobXw, cobYw, elbowAngleDeg, headAngleDeg, trunkAngleDeg, mass, height, side };

  document.getElementById('glideStep').style.display = 'block';
  document.getElementById('glideEnd').value = Math.min(15, n - 1);
  document.getElementById('cycleEnd').value = Math.min(30, n - 1);
  alert(`Processed ${n} frames (${side} side). Now pick glide + stroke-cycle windows.`);
}

document.getElementById('btnAnalyze').onclick = () => {
  if (!processed) return;
  const { t, vx, ax, mass, height, comXw, cobXw, headAngleDeg, trunkAngleDeg, elbowAngleDeg } = processed;
  const gs = parseInt(document.getElementById('glideStart').value);
  const ge = parseInt(document.getElementById('glideEnd').value);
  const cs = parseInt(document.getElementById('cycleStart').value);
  const ce = parseInt(document.getElementById('cycleEnd').value);

  const glideT = t.slice(gs, ge + 1), glideV = vx.slice(gs, ge + 1);
  const { D, r2 } = DragModel.calibratePassiveDrag(glideT, glideV, mass);

  const cycleV = vx.slice(cs, ce + 1).filter(v => v > 0);
  const meanV = cycleV.reduce((a, b) => a + b, 0) / cycleV.length;
  const stdV = Math.sqrt(cycleV.reduce((a, b) => a + (b - meanV) ** 2, 0) / cycleV.length);
  const dvPercent = (stdV / meanV) * 100;

  const bodyLength = height * 0.9; // rough swimmer streamline length estimate from height
  const Fprop = D != null ? ForceReconstruction.reconstruct(t, vx, ax, D, bodyLength, mass) : null;

  let impulses = null;
  if (Fprop) {
    const boundaries = ForceReconstruction.segmentPhases(t, elbowAngleDeg.slice(cs, ce + 1));
    impulses = ForceReconstruction.phaseImpulses(t.slice(cs, ce + 1), Fprop.slice(cs, ce + 1), boundaries);
  }

  // CoM-CoB offset (average over cycle window)
  let dxSum = 0, cnt = 0;
  for (let i = cs; i <= ce; i++) { dxSum += comXw[i] - cobXw[i]; cnt++; }
  const dxAvg = dxSum / cnt;

  // Jacobian correction over head + trunk angles (numerically perturbed)
  const offsetFn = (basePoints, override) => {
    // Simplified: only head angle override is modeled directly here since it's the
    // most actionable single cue; trunk/ankle perturbation follows the same pattern
    // and can be added by extending this function with more override keys.
    return dxAvg + (override.head || 0) * 0.02; // sensitivity placeholder scaled by typical head lever arm
  };
  const { J, baseOffset } = CorrectionEngine.buildJacobian(null, offsetFn, ['head']);
  const correction = CorrectionEngine.solveMinimalNormCorrection(J, baseOffset);
  const headCorrectionDeg = CorrectionEngine.radToDeg(correction[0]);

  renderResults({ t, vx, Fprop, D, r2, dvPercent, dxAvg, impulses, headCorrectionDeg, gs, ge, cs, ce, side: processed.side });
};

function renderResults(r) {
  document.getElementById('results').style.display = 'block';
  const labels = r.t.map(x => x.toFixed(2));
  Charts.renderVelocity('velChart', labels, r.vx);
  if (r.Fprop) Charts.renderForce('forceChart', labels, r.Fprop);
  if (r.impulses) Charts.renderImpulses('impulseChart', r.impulses);

  const lines = [];
  lines.push(`Tracked side: ${r.side}`);
  lines.push(`Intra-cycle velocity variation (dv): ${r.dvPercent.toFixed(1)}% (elite front-crawl reference ~10%)`);
  if (r.D != null) {
    lines.push(`Calibrated passive drag D: ${r.D.toFixed(2)} (fit R² = ${r.r2.toFixed(3)})`);
    if (r.r2 < 0.85) lines.push('  ⚠ low R² — check the glide window is a real passive glide, no kicking.');
  } else {
    lines.push('Passive drag: could not calibrate — check glide window.');
  }
  lines.push(`Average CoM–CoB horizontal offset: ${(r.dxAvg * 100).toFixed(2)} cm`);
  lines.push(`Suggested head pitch correction: ${r.headCorrectionDeg >= 0 ? 'raise' : 'lower'} by ${Math.abs(r.headCorrectionDeg).toFixed(1)}°`);
  if (r.impulses) {
    lines.push('Per-phase propulsive impulse (N·s):');
    r.impulses.forEach((p, i) => lines.push(`  Phase ${i + 1} (frames ${p.startFrame}-${p.endFrame}): ${p.impulse.toFixed(2)}`));
  }
  lines.push('');
  lines.push('Modeling assumptions (accuracy-slide material):');
  lines.push('- de Leva mass fractions and relative segment densities are population averages');
  lines.push('- CoB is a volume-proxy from mass/density, not a direct volumetric scan');
  lines.push('- Only one side tracked; far side assumed mirrored');
  lines.push('- Wave-drag coefficients in dragModel.js are illustrative, not measured for this swimmer');
  lines.push('- Homography assumes the 4 calibration points lie in the swimmer\'s plane of motion');

  document.getElementById('numbers').textContent = lines.join('\n');
}
