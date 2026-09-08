// calibration.js
// Homography-based pixel -> real-world-plane calibration.
// Instead of a single pixel distance / known distance scale (which only works if the camera
// is perfectly perpendicular to the swimmer's plane of motion), we solve for a full 3x3
// homography from 4 point correspondences: 4 pixel points the user clicks, matched to 4
// known real-world (x,y) coordinates in meters (e.g. the 4 corners of a lane-marker rectangle
// of known size). This corrects for camera tilt / perspective distortion in the swim plane.

const Calibration = (() => {
  let pixelPoints = [];   // [{x,y}, ...] length 4
  let worldPoints = [];   // [{x,y}, ...] length 4, meters
  let H = null;           // 3x3 homography matrix, row-major array of 9 numbers

  function reset() {
    pixelPoints = [];
    worldPoints = [];
    H = null;
  }

  function addPointPair(px, py, wx, wy) {
    pixelPoints.push({ x: px, y: py });
    worldPoints.push({ x: wx, y: wy });
    if (pixelPoints.length === 4) {
      H = solveHomography(pixelPoints, worldPoints);
    }
  }

  function isReady() { return H !== null; }

  // Solve H such that [wx, wy, 1]^T ~ H * [px, py, 1]^T
  // Standard DLT (direct linear transform) approach: build 8x8 linear system from 4 point pairs
  // (each pair gives 2 equations), solve via Gaussian elimination.
  function solveHomography(src, dst) {
    // Build A (8x8) and b (8x1) for unknowns h11..h32 (h33 fixed = 1)
    const A = [];
    const b = [];
    for (let i = 0; i < 4; i++) {
      const { x: X, y: Y } = src[i];
      const { x: xp, y: yp } = dst[i];
      A.push([X, Y, 1, 0, 0, 0, -X * xp, -Y * xp]);
      b.push(xp);
      A.push([0, 0, 0, X, Y, 1, -X * yp, -Y * yp]);
      b.push(yp);
    }
    const h = gaussianSolve(A, b); // 8 values: h11,h12,h13,h21,h22,h23,h31,h32
    return [h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1];
  }

  // Solve A*x = b for square A using Gaussian elimination with partial pivoting.
  function gaussianSolve(A, b) {
    const n = A.length;
    const M = A.map((row, i) => [...row, b[i]]);
    for (let col = 0; col < n; col++) {
      let pivot = col;
      for (let r = col + 1; r < n; r++) {
        if (Math.abs(M[r][col]) > Math.abs(M[pivot][col])) pivot = r;
      }
      [M[col], M[pivot]] = [M[pivot], M[col]];
      const pivotVal = M[col][col];
      if (Math.abs(pivotVal) < 1e-12) continue; // singular-ish, best effort
      for (let c = col; c <= n; c++) M[col][c] /= pivotVal;
      for (let r = 0; r < n; r++) {
        if (r === col) continue;
        const factor = M[r][col];
        for (let c = col; c <= n; c++) M[r][c] -= factor * M[col][c];
      }
    }
    return M.map(row => row[n]);
  }

  // Apply homography to a pixel point -> world-plane meters
  function toWorld(px, py) {
    if (!H) throw new Error('Calibration not ready');
    const x = H[0] * px + H[1] * py + H[2];
    const y = H[3] * px + H[4] * py + H[5];
    const w = H[6] * px + H[7] * py + H[8];
    return { x: x / w, y: y / w };
  }

  return { reset, addPointPair, isReady, toWorld, get pixelPoints() { return pixelPoints; } };
})();
