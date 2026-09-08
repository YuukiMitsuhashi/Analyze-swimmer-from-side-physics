// correctionEngine.js
// Instead of solving only for a head-angle correction, this numerically perturbs several
// joint angles (head pitch, hip flexion, ankle position) one at a time to build a Jacobian
// of how each affects the CoM-CoB horizontal offset, then solves the minimal-norm correction
// vector that nulls the offset. Same category of math as basic inverse kinematics.

const CorrectionEngine = (() => {

  // pointsFn(anglesOverride) must return the frame's landmark points given a set of
  // angle overrides, so we can recompute CoM/CoB under a hypothetical joint change.
  // angleNames: e.g. ['head','hip','ankle']
  function buildJacobian(basePoints, computeOffsetFn, angleNames, perturbRad = 0.02) {
    const baseOffset = computeOffsetFn(basePoints, {});
    const J = [];
    angleNames.forEach(name => {
      const override = {}; override[name] = perturbRad;
      const perturbedOffset = computeOffsetFn(basePoints, override);
      J.push((perturbedOffset - baseOffset) / perturbRad); // d(offset)/d(angle) in meters/radian
    });
    return { J, baseOffset };
  }

  // Solve minimal-norm correction: given Jacobian J (1 x n, since we have 1 constraint --
  // null the horizontal offset -- and n joint angles), the minimal-norm solution is
  // delta = J^T * (J J^T)^-1 * (-offset), which for a 1-constraint case reduces to
  // distributing the correction proportionally to each joint's sensitivity.
  function solveMinimalNormCorrection(J, offset) {
    const JJt = J.reduce((sum, j) => sum + j * j, 0);
    if (Math.abs(JJt) < 1e-9) return J.map(() => 0);
    const lambda = -offset / JJt;
    return J.map(j => j * lambda); // radians, one per angle in angleNames order
  }

  function radToDeg(r) { return r * 180 / Math.PI; }

  return { buildJacobian, solveMinimalNormCorrection, radToDeg };
})();
