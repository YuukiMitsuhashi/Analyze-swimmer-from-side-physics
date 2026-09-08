// poseCapture.js
// Wraps the pose detection model. Isolated so the model (MoveNet now, BlazePose later for
// 3D landmarks) can be swapped without touching downstream physics code.

const PoseCapture = (() => {
  const KP = {
    nose: 0, leftEye: 1, rightEye: 2, leftEar: 3, rightEar: 4,
    leftShoulder: 5, rightShoulder: 6, leftElbow: 7, rightElbow: 8,
    leftWrist: 9, rightWrist: 10, leftHip: 11, rightHip: 12,
    leftKnee: 13, rightKnee: 14, leftAnkle: 15, rightAnkle: 16
  };

  let detector = null;
  let seamXMin = null, seamXMax = null;

  async function loadModel() {
    detector = await poseDetection.createDetector(poseDetection.SupportedModels.MoveNet, {
      modelType: poseDetection.movenet.modelType.SINGLEPOSE_THUNDER
    });
    return detector;
  }

  function setSeamRange(xMin, xMax) { seamXMin = xMin; seamXMax = xMax; }
  function clearSeamRange() { seamXMin = null; seamXMax = null; }

  function isInSeam(x) {
    return seamXMin != null && x > seamXMin && x < seamXMax;
  }

  async function estimateFrame(videoEl) {
    const poses = await detector.estimatePoses(videoEl, { flipHorizontal: false });
    if (poses.length === 0) return null;
    return poses[0].keypoints; // array of 17 {x,y,score,name}
  }

  // Determine which side (left/right) is more reliably tracked across the whole clip.
  function pickSide(frames) {
    let leftScore = 0, rightScore = 0;
    frames.forEach(f => {
      const k = f.kps;
      leftScore += k[KP.leftShoulder].score + k[KP.leftHip].score + k[KP.leftKnee].score;
      rightScore += k[KP.rightShoulder].score + k[KP.rightHip].score + k[KP.rightKnee].score;
    });
    return rightScore > leftScore ? 'right' : 'left';
  }

  return { KP, loadModel, setSeamRange, clearSeamRange, isInSeam, estimateFrame, pickSide };
})();
