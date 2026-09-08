// anthropometry.js
// Computes center of mass (CoM) using de Leva segment mass fractions, AND a volumetric
// center of buoyancy (CoB) using segment relative densities -- instead of assuming CoB
// sits at a fixed fraction of the trunk. The CoM-CoB offset this produces is now a
// consequence of an actual (if simplified) volume/density model, not a guessed constant.

const Anthropometry = (() => {
  let SEG = null; // loaded from data/deLevaParameters.json

  async function loadParams(path = 'data/deLevaParameters.json') {
    const res = await fetch(path);
    SEG = await res.json();
    return SEG;
  }

  function lerpFrac(a, b, f) { return { x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f }; }
  function segLength(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }

  // points: {shoulder, hip, elbow, wrist, knee, ankle, ear} in world-plane meters (post-homography)
  // Returns { comX, comY, cobX, cobY } for one frame.
  // Arm/leg segments are doubled to approximate both sides (only one side is visible in a
  // single side-view camera) -- this is an explicit, documented simplification.
  function computeFrame(points) {
    const { shoulder: S, hip: H, elbow: E, wrist: W, knee: K, ankle: A, ear: Ea } = points;

    const segs = [
      { def: SEG.head,       p0: S, p1: Ea, sides: 1 },
      { def: SEG.trunk,      p0: S, p1: H,  sides: 1 },
      { def: SEG.upperArm,   p0: S, p1: E,  sides: 2 },
      { def: SEG.forearmHand,p0: E, p1: W,  sides: 2 },
      { def: SEG.thigh,      p0: H, p1: K,  sides: 2 },
      { def: SEG.shankFoot,  p0: K, p1: A,  sides: 2 }
    ];

    let totalMass = 0, cmX = 0, cmY = 0;
    let totalVolumeWeight = 0, cbX = 0, cbY = 0;

    segs.forEach(seg => {
      const com = lerpFrac(seg.p0, seg.p1, seg.def.comFrac);
      const massContribution = seg.def.massFrac * seg.sides;
      totalMass += massContribution;
      cmX += massContribution * com.x;
      cmY += massContribution * com.y;

      // Volume is approximated as proportional to mass / relative density
      // (mass = density * volume  =>  volume ∝ mass / density).
      const volumeContribution = massContribution / seg.def.relDensity;
      totalVolumeWeight += volumeContribution;
      cbX += volumeContribution * com.x; // same geometric location used as volume centroid proxy
      cbY += volumeContribution * com.y;
    });

    return {
      comX: cmX / totalMass,
      comY: cmY / totalMass,
      cobX: cbX / totalVolumeWeight,
      cobY: cbY / totalVolumeWeight
    };
  }

  return { loadParams, computeFrame, get params() { return SEG; } };
})();
