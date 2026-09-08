// charts.js
// Thin wrapper around Chart.js so main.js doesn't need to know rendering details.

const Charts = (() => {
  let velChart = null, forceChart = null, impulseChart = null;

  function renderVelocity(canvasId, labels, vx) {
    if (velChart) velChart.destroy();
    velChart = new Chart(document.getElementById(canvasId), {
      type: 'line',
      data: { labels, datasets: [{ label: 'Horizontal CoM velocity (m/s)', data: vx, borderColor: '#7fd1ff', pointRadius: 0 }] },
      options: { scales: { x: { title: { display: true, text: 'time (s)' } }, y: { title: { display: true, text: 'v (m/s)' } } } }
    });
  }

  function renderForce(canvasId, labels, Fprop) {
    if (forceChart) forceChart.destroy();
    forceChart = new Chart(document.getElementById(canvasId), {
      type: 'line',
      data: { labels, datasets: [{ label: 'Reconstructed net propulsive force (N)', data: Fprop, borderColor: '#3fb950', pointRadius: 0 }] },
      options: { scales: { x: { title: { display: true, text: 'time (s)' } }, y: { title: { display: true, text: 'F (N)' } } } }
    });
  }

  function renderImpulses(canvasId, impulses) {
    if (impulseChart) impulseChart.destroy();
    impulseChart = new Chart(document.getElementById(canvasId), {
      type: 'bar',
      data: {
        labels: impulses.map((_, i) => `Phase ${i + 1}`),
        datasets: [{ label: 'Impulse per phase (N·s)', data: impulses.map(p => p.impulse), backgroundColor: '#f0883e' }]
      },
      options: { scales: { y: { title: { display: true, text: 'Impulse (N·s)' } } } }
    });
  }

  return { renderVelocity, renderForce, renderImpulses };
})();
