// flowSolver.js
// A real 2D incompressible Navier-Stokes solver using Jos Stam's "stable fluids" method:
//   1. Advect velocity field backward along streamlines (semi-Lagrangian advection).
//   2. Add forces / diffusion.
//   3. Project onto a divergence-free field by solving the pressure Poisson equation
//      (div(u)=0 constraint) via Jacobi iteration, then subtract the pressure gradient.
// A user-drawn obstacle (streamlined vs hunched cross-section) sits inside the grid and
// velocity is zeroed inside it each step, producing visible flow separation / vortex
// shedding behind blunt shapes -- this is the live "physics made visible" demo.

const FlowSolver = (() => {
  let N, dt, diff, visc;
  let u, v, uPrev, vPrev, dens, densPrev;
  let obstacle; // Uint8Array, 1 = solid

  function index(i, j) { return i + (N + 2) * j; }

  function init(size, options = {}) {
    N = size;
    dt = options.dt || 0.1;
    diff = options.diff || 0.0001;
    visc = options.visc || 0.0001;
    const total = (N + 2) * (N + 2);
    u = new Float32Array(total); v = new Float32Array(total);
    uPrev = new Float32Array(total); vPrev = new Float32Array(total);
    dens = new Float32Array(total); densPrev = new Float32Array(total);
    obstacle = new Uint8Array(total);
  }

  function setObstacle(mask) { obstacle = mask; } // Uint8Array same size as grid

  function setBoundary(b, x) {
    for (let i = 1; i <= N; i++) {
      x[index(0, i)] = b === 1 ? -x[index(1, i)] : x[index(1, i)];
      x[index(N + 1, i)] = b === 1 ? -x[index(N, i)] : x[index(N, i)];
      x[index(i, 0)] = b === 2 ? -x[index(i, 1)] : x[index(i, 1)];
      x[index(i, N + 1)] = b === 2 ? -x[index(i, N)] : x[index(i, N)];
    }
    x[index(0, 0)] = 0.5 * (x[index(1, 0)] + x[index(0, 1)]);
    x[index(0, N + 1)] = 0.5 * (x[index(1, N + 1)] + x[index(0, N)]);
    x[index(N + 1, 0)] = 0.5 * (x[index(N, 0)] + x[index(N + 1, 1)]);
    x[index(N + 1, N + 1)] = 0.5 * (x[index(N, N + 1)] + x[index(N + 1, N)]);
    // Zero velocity inside the obstacle
    for (let i = 0; i <= N + 1; i++) for (let j = 0; j <= N + 1; j++) {
      if (obstacle[index(i, j)]) x[index(i, j)] = 0;
    }
  }

  function linearSolve(b, x, x0, a, c, iterations = 20) {
    for (let k = 0; k < iterations; k++) {
      for (let i = 1; i <= N; i++) {
        for (let j = 1; j <= N; j++) {
          x[index(i, j)] = (x0[index(i, j)] + a * (
            x[index(i - 1, j)] + x[index(i + 1, j)] + x[index(i, j - 1)] + x[index(i, j + 1)]
          )) / c;
        }
      }
      setBoundary(b, x);
    }
  }

  function diffuseField(b, x, x0) {
    const a = dt * diff * N * N;
    linearSolve(b, x, x0, a, 1 + 4 * a);
  }

  // Semi-Lagrangian advection: for each cell, trace backward along velocity and
  // bilinearly interpolate the field there. Unconditionally stable regardless of dt,
  // which is the key trick that makes real-time interactive fluid sim possible.
  function advect(b, d, d0, uField, vField) {
    const dt0 = dt * N;
    for (let i = 1; i <= N; i++) {
      for (let j = 1; j <= N; j++) {
        let x = i - dt0 * uField[index(i, j)];
        let y = j - dt0 * vField[index(i, j)];
        x = Math.min(Math.max(x, 0.5), N + 0.5);
        y = Math.min(Math.max(y, 0.5), N + 0.5);
        const i0 = Math.floor(x), i1 = i0 + 1;
        const j0 = Math.floor(y), j1 = j0 + 1;
        const s1 = x - i0, s0 = 1 - s1, t1 = y - j0, t0 = 1 - t1;
        d[index(i, j)] =
          s0 * (t0 * d0[index(i0, j0)] + t1 * d0[index(i0, j1)]) +
          s1 * (t0 * d0[index(i1, j0)] + t1 * d0[index(i1, j1)]);
      }
    }
    setBoundary(b, d);
  }

  // Pressure projection: solves the Poisson equation for pressure so that subtracting
  // its gradient from the velocity field makes it divergence-free (incompressible).
  function project(uField, vField, p, divergence) {
    const h = 1.0 / N;
    for (let i = 1; i <= N; i++) {
      for (let j = 1; j <= N; j++) {
        divergence[index(i, j)] = -0.5 * h * (
          uField[index(i + 1, j)] - uField[index(i - 1, j)] +
          vField[index(i, j + 1)] - vField[index(i, j - 1)]
        );
        p[index(i, j)] = 0;
      }
    }
    setBoundary(0, divergence); setBoundary(0, p);
    linearSolve(0, p, divergence, 1, 4);

    for (let i = 1; i <= N; i++) {
      for (let j = 1; j <= N; j++) {
        uField[index(i, j)] -= 0.5 * (p[index(i + 1, j)] - p[index(i - 1, j)]) / h;
        vField[index(i, j)] -= 0.5 * (p[index(i, j + 1)] - p[index(i, j - 1)]) / h;
      }
    }
    setBoundary(1, uField); setBoundary(2, vField);
  }

  function step(inflowU = 1.0) {
    // constant inflow on the left edge simulates flow past the obstacle
    for (let j = 1; j <= N; j++) { u[index(1, j)] = inflowU; }

    [u, uPrev] = [uPrev, u]; [v, vPrev] = [vPrev, v];
    diffuseField(1, u, uPrev); diffuseField(2, v, vPrev);
    project(u, v, uPrev, vPrev);
    [u, uPrev] = [uPrev, u]; [v, vPrev] = [vPrev, v];
    advect(1, u, uPrev, uPrev, vPrev); advect(2, v, vPrev, uPrev, vPrev);
    project(u, v, uPrev, vPrev);

    [dens, densPrev] = [densPrev, dens];
    diffuseField(0, dens, densPrev);
    [dens, densPrev] = [densPrev, dens];
    advect(0, dens, densPrev, u, v);
    // continuously seed dye near the inflow so streaklines stay visible
    for (let j = 1; j <= N; j++) dens[index(1, j)] = 1.0;
  }

  function getState() { return { u, v, dens, N }; }

  return { init, setObstacle, step, getState, index };
})();
