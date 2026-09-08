// forceReconstruction.js
// Reconstructs net propulsive force via Newton's second law: F_prop = m*a + drag(v),
// then segments the stroke cycle into phases using zero-crossings of elbow angular
// velocity, and integrates F_prop over each phase to get an impulse per phase --
// so feedback can point at *which part* of the stroke is underpowered.

const ForceReconstruction = (() => {

  function reconstruct(t, vx, ax, D, bodyLength, mass) {
    return vx.map((v, i) => mass * ax[i] + DragModel.totalDrag(v, D, bodyLength));
  }

  // elbowAngleDeg: array of elbow flexion angle per frame (shoulder-elbow-wrist angle).
  // Returns array of phase boundaries (frame indices) where angular velocity changes sign.
  function segmentPhases(t, elbowAngleDeg) {
    const angVel = Filters.derivative(elbowAngleDeg, 2, 2).map((v, i) => v); // per-frame, sign is what matters
    const boundaries = [0];
    for (let i = 1; i < angVel.length; i++) {
      if (Math.sign(angVel[i]) !== Math.sign(angVel[i - 1]) && angVel[i - 1] !== 0) {
        boundaries.push(i);
      }
    }
    boundaries.push(elbowAngleDeg.length - 1);
    return boundaries;
  }

  // Trapezoidal integration of F_prop(t) over each phase -> impulse (N*s) per phase.
  function phaseImpulses(t, Fprop, boundaries) {
    const impulses = [];
    for (let p = 0; p < boundaries.length - 1; p++) {
      const start = boundaries[p], end = boundaries[p + 1];
      let impulse = 0;
      for (let i = start; i < end; i++) {
        const dt = t[i + 1] - t[i];
        impulse += 0.5 * (Fprop[i] + Fprop[i + 1]) * dt;
      }
      impulses.push({ startFrame: start, endFrame: end, impulse });
    }
    return impulses;
  }

  return { reconstruct, segmentPhases, phaseImpulses };
})();
