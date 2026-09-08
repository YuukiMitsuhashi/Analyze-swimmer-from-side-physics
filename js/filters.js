// filters.js
// Generalized Savitzky-Golay smoothing/derivative filter, derived from first principles
// rather than a hardcoded coefficient table, plus confidence-weighted interpolation for
// frames where a keypoint was low-confidence or fell inside the camera seam region.

const Filters = (() => {

  // Build SG coefficients for a given half-window size (m) and polynomial order (p),
  // for derivative order 'deriv' (0 = smoothing, 1 = first derivative, in units of "per frame",
  // caller divides by dt afterward).
  //
  // Method: for window points at offsets k = -m..m, fit a degree-p polynomial by least squares.
  // The smoothing/derivative coefficients are rows of (A^T A)^-1 A^T, where A is the Vandermonde
  // matrix of offsets. This reproduces the classic Savitzky-Golay tables but works for any
  // window/order instead of being limited to hand-copied constants.
  function sgCoefficients(halfWindow, order, deriv = 0) {
    const offsets = [];
    for (let k = -halfWindow; k <= halfWindow; k++) offsets.push(k);
    const n = offsets.length;

    // Vandermonde matrix A: rows = frame offsets, cols = powers 0..order
    const A = offsets.map(k => {
      const row = [];
      for (let p = 0; p <= order; p++) row.push(Math.pow(k, p));
      return row;
    });

    const At = transpose(A);
    const AtA = matMul(At, A);
    const AtA_inv = invertMatrix(AtA);
    const pseudoInverse = matMul(AtA_inv, At); // (order+1) x n

    // The coefficient row we want is the one corresponding to the requested derivative order,
    // scaled by deriv! (standard SG derivative scaling).
    let factorial = 1;
    for (let i = 2; i <= deriv; i++) factorial *= i;
    return pseudoInverse[deriv].map(v => v * factorial);
  }

  function transpose(M) {
    return M[0].map((_, c) => M.map(row => row[c]));
  }
  function matMul(A, B) {
    const result = [];
    for (let i = 0; i < A.length; i++) {
      const row = [];
      for (let j = 0; j < B[0].length; j++) {
        let sum = 0;
        for (let k = 0; k < B.length; k++) sum += A[i][k] * B[k][j];
        row.push(sum);
      }
      result.push(row);
    }
    return result;
  }
  function invertMatrix(M) {
    const n = M.length;
    const aug = M.map((row, i) => [...row, ...Array(n).fill(0).map((_, j) => (i === j ? 1 : 0))]);
    for (let col = 0; col < n; col++) {
      let pivot = col;
      for (let r = col + 1; r < n; r++) if (Math.abs(aug[r][col]) > Math.abs(aug[pivot][col])) pivot = r;
      [aug[col], aug[pivot]] = [aug[pivot], aug[col]];
      const pv = aug[col][col];
      for (let c = 0; c < 2 * n; c++) aug[col][c] /= pv;
      for (let r = 0; r < n; r++) {
        if (r === col) continue;
        const f = aug[r][col];
        for (let c = 0; c < 2 * n; c++) aug[r][c] -= f * aug[col][c];
      }
    }
    return aug.map(row => row.slice(n));
  }

  function applyKernel(series, coeffs) {
    const n = series.length, half = Math.floor(coeffs.length / 2);
    const out = new Array(n);
    for (let i = 0; i < n; i++) {
      let sum = 0;
      for (let k = -half; k <= half; k++) {
        let idx = i + k;
        if (idx < 0) idx = 0;
        if (idx >= n) idx = n - 1;
        sum += series[idx] * coeffs[k + half];
      }
      out[i] = sum;
    }
    return out;
  }

  function smooth(series, halfWindow = 3, order = 2) {
    return applyKernel(series, sgCoefficients(halfWindow, order, 0));
  }
  function derivative(series, halfWindow = 3, order = 2) {
    return applyKernel(series, sgCoefficients(halfWindow, order, 1));
  }

  // Confidence-weighted interpolation: for a series of {x, y, score, inSeam},
  // treat points with score < threshold OR inSeam as missing, and fill by linear
  // interpolation, weighting the surrounding valid points' influence isn't needed for
  // straight linear fill, but we keep the confidence signal available upstream for
  // deciding how much to trust each frame's contribution to fits (e.g. the drag regression).
  function interpolateMissing(points, scoreThreshold = 0.3) {
    const n = points.length;
    const valid = points.map(p => p.score >= scoreThreshold && !p.inSeam);
    let lastValid = -1;
    for (let i = 0; i < n; i++) {
      if (valid[i]) {
        if (lastValid !== -1 && lastValid !== i - 1) {
          const a = points[lastValid], b = points[i];
          for (let j = lastValid + 1; j < i; j++) {
            const f = (j - lastValid) / (i - lastValid);
            points[j].x = a.x + (b.x - a.x) * f;
            points[j].y = a.y + (b.y - a.y) * f;
            points[j].interpolated = true;
          }
        }
        lastValid = i;
      }
    }
    return points;
  }

  function linreg(xs, ys) {
    const n = xs.length;
    let sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (let i = 0; i < n; i++) { sx += xs[i]; sy += ys[i]; sxx += xs[i] * xs[i]; sxy += xs[i] * ys[i]; }
    const slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
    const intercept = (sy - slope * sx) / n;
    const yMean = sy / n;
    let ssTot = 0, ssRes = 0;
    for (let i = 0; i < n; i++) { const pred = slope * xs[i] + intercept; ssRes += (ys[i] - pred) ** 2; ssTot += (ys[i] - yMean) ** 2; }
    return { slope, intercept, r2: 1 - ssRes / ssTot };
  }

  return { sgCoefficients, smooth, derivative, interpolateMissing, linreg };
})();
