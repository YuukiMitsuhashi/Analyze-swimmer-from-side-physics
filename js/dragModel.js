// dragModel.js
// Two drag contributions:
//   1. Passive form drag D, calibrated per-swimmer from a real streamline-glide decay.
//   2. Wave-making drag, which is negligible at low Froude number but rises sharply as
//      swim speed approaches the "hull speed" limit set by body length -- the same physics
//      that limits displacement-hull boat speed.

const DragModel = (() => {
  const G = 9.81;

  // Passive drag calibration.
  // Physics: m dv/dt = -D v^2  =>  d(1/v)/dt = D/m  -- LINEAR in 1/v vs t.
  // So instead of a nonlinear curve fit, a simple linear regression of 1/v against t
  // during a real passive glide (no kicking, no pulling) recovers D directly.
  function calibratePassiveDrag(tWindow, vWindow, mass) {
    const t0 = tWindow[0];
    const xs = [], ys = [];
    for (let i = 0; i < vWindow.length; i++) {
      if (vWindow[i] > 0.05) { xs.push(tWindow[i] - t0); ys.push(1 / vWindow[i]); }
    }
    if (xs.length < 3) return { D: null, r2: null };
    const fit = Filters.linreg(xs, ys);
    return { D: fit.slope * mass, r2: fit.r2, intercept: fit.intercept };
  }

  function froudeNumber(v, bodyLength) {
    return v / Math.sqrt(G * bodyLength);
  }

  // Semi-empirical wave-drag multiplier: negligible below Fr ~0.35, rises steeply
  // approaching Fr ~0.45+ (this mirrors the empirical hull-speed resistance curve shape
  // used in naval architecture; the exact coefficients here are illustrative, not
  // measured for this swimmer, and should be stated as such on an accuracy slide).
  function waveDragMultiplier(Fr) {
    if (Fr < 0.35) return 1.0;
    const excess = Fr - 0.35;
    return 1.0 + 8 * Math.pow(excess, 3); // steep cubic rise past the threshold
  }

  // Total resistive force at a given instant.
  function totalDrag(v, D, bodyLength) {
    const formDrag = D * v * v;
    const Fr = froudeNumber(v, bodyLength);
    return formDrag * waveDragMultiplier(Fr);
  }

  return { calibratePassiveDrag, froudeNumber, waveDragMultiplier, totalDrag };
})();
