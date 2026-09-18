# Fluid Sim — Overview

A 2D SPH (smoothed particle hydrodynamics) fluid simulation in Unity 6 / URP, running on macOS via Metal. It began as a CPU learning exercise and became a GPU compute pipeline. Twenty-eight tasks are done or nearly done; the 3D port has not started.

This file is the one-page mental model. `WALKTHROUGH.md` is the long form.

---

## What it is

A fluid is **N particles**. Each carries a mass, a position and a velocity. There is no mesh and no grid of cells — a particle only ever knows about the particles within a smoothing radius `h` of it. Smoothing kernels turn those local neighbourhoods into continuous fields (density, pressure), and those fields into forces.

That is the whole idea. Everything in this repo is either that maths, or plumbing to make it fast.

---

## The step

Every simulation step runs the same eight things, in this order. The order is not arbitrary; each line needs the one above it.

```
 1. rebuild the grid        positions -> who is near whom
 2. density                 neighbours -> rho, rho_near
 3. pressure                rho -> p, p_near
 4. pressure force          p, positions -> force        (main + near channels)
 5. viscosity force         velocities -> force +=        (XSPH)
 6. reduce extremes         force, velocity -> max|v|, max|a|   (for the time step)
 7. integrate               force -> new velocity, position     (+ wall clamp)
 8. render                  positions -> pixels
```

Steps 1–7 happen on the GPU. Step 8 binds the GPU's own particle buffer, so nothing is copied to the CPU between them.

---

## The two solvers

There are **two complete implementations** of steps 1–7, and keeping both is deliberate.

| | CPU | GPU |
|---|---|---|
| Where | `SPH2D` + `SPHJobs` + `SPHGrid` + `SPHSolver` | `SPHCompute` + `SPH2D.compute` |
| Parallelism | Burst + C# Job System, ~8 cores | ~1000 GPU threads |
| Neighbour search | uniform grid, ~N/4.5 cells | count sort into ~N/4.8 buckets |
| State layout | SoA — one array per field | AoS — one 40-byte `Particle` |
| Role | reference, and fallback | the one that actually runs |

The CPU version exists because it is the thing the GPU version is *checked against*. Every GPU task since T-026 was verified by running both on identical input and diffing the results.

---

## File map

Everything 2D lives under a `2D/` folder. `3D/` is empty and reserved.

**Scripts — `Assets/Scripts/2D/`**

| File | Lines | What it owns |
|---|---|---|
| `Particle2D.cs` | 22 | the 40-byte per-particle struct — the data contract |
| `ParticleState.cs` | 67 | SoA state (7 arrays) for the CPU path |
| `SPHMath.cs` | 173 | kernels + equations of state. Pure maths, no particles |
| `SPHSolver.cs` | 186 | `SolverParams` + the O(N) reductions |
| `SPHJobs.cs` | 327 | the 7 Burst `IJobParallelFor` passes |
| `SPHGrid.cs` | 465 | CPU uniform grid + CSR neighbour lists |
| `SPH2D.cs` | 803 | MonoBehaviour: spawn, parameters, orchestration, gizmos |
| `SPHCompute.cs` | 1148 | GPU buffers, kernels, dispatch, readback, self-tests |
| `SPHComputeSimulation.cs` | 192 | the GPU frame loop: accumulator, substeps, adaptive `dt` |
| `ParticleRenderer2D.cs` | 735 | instanced drawing, colour ramp, impostor sizing |

**Shaders — `Assets/Shaders/2D/`**

| File | Lines | What it does |
|---|---|---|
| `SPH2D.compute` | 625 | 13 kernels: the whole GPU solver |
| `Particle2D.shader` | 227 | two passes — visible impostors, and a depth prepass |
| `FluidSurface.shader` | 205 | bilateral blur + composite — **WIP, not working** |

The 13 compute kernels, in dispatch order: `ComputeCellKeys`, `ClearBuckets`, `CountBuckets`, `PrefixSumBuckets`, `ScatterParticles` (the grid), then `ComputeDensity`, `ComputePressure`, `ComputePressureForce`, `ComputeViscosity`, `ClearExtremes`, `ReduceExtremes`, `Integrate`, plus `ProbeRoundTrip` (a plumbing smoke test).

---

## The two pressure channels

This is the least obvious part and the most important. There are **two** pressure terms, not one:

| | density kernel | EOS | gradient shape | can be attractive? |
|---|---|---|---|---|
| **main** | Spiky Pow2 | `k(rho - rho0)` | linear `(h - r)` | **yes** — goes negative |
| **near** | Spiky Pow3 | `kNear * rhoNear` (no rest offset) | quadratic `(h - r)^2` | **no**, never |

Why both: a single-channel solver has to choose. Clamp the pressure to be non-negative and the fluid can't hold itself together (it sprays). Allow negative pressure and particles pair up and collapse. Two channels let each handle one job — the near term repels at short range, the main term supplies the two-way restoring force that makes a real equilibrium possible.

The gradients **must** differ in shape. If both were quadratic, the near term would be a constant multiple of the main one and could not suppress pairing.

Consequence: the fluid settles ~1% *below* rest density. That is the designed equilibrium, not a bug.

---

## Memory layout — the decision that shapes everything

**Derived quantities are not stored in the particle struct.** Density, pressure and force live in their own buffers, on both CPU and GPU.

Why: the density pass reads *every* particle but writes only its own. If density lived in the particle struct, thread `i` would be writing a field thread `j` is reading. The race is benign — the value read isn't one the pass uses — and rejecting benign races is the point: correctness shouldn't rest on a coincidence.

The payoff is invisible but large: each pass gets one array to write and a set to read, which is exactly what both Burst vectorisation and GPU parallelism want.

---

## Performance

Measured on an Apple M5, headless.

| N | brute force | gridded | speedup |
|---|---|---|---|
| 400 | 1.29 ms | 0.54 ms | 2.4× |
| 5,000 | 6.57 ms | 0.63 ms | 10.4× |
| 10,000 | 29.25 ms | **1.05 ms** | **27.8×** |
| 50,000 | — | 3.65 ms | — |

At the scene's configuration (10×10 box, 0.3×0.8 spawn region, 5,000 particles) a step costs **0.18–0.57 ms**. For comparison the CPU Burst solver costs 15.7 ms at the same count — about **25× slower**.

Memory is not a constraint: roughly 130 bytes per particle, so 5,000 is ~0.7 MB and 100,000 would be ~13 MB.

**The limit is step count, not solve speed.** `h` shrinks as `1/sqrt(N)`, so the CFL time step shrinks with it. At 10,000 particles you need ~2,000 steps per second of simulated fluid, and that — not the arithmetic — is what runs out.

---

## The invariants

Things that must not silently change. Each is documented in `TASKS.md` with where it lives and which task could break it.

1. **Boundary particles complete the kernel support near walls** and push back by pressure mirroring. Without them, particles weld to walls.
2. **Viscosity is the only dissipative term.** Remove it and the fluid never settles.
3. **Clamping and the near channel are mutually exclusive.** The near term is a permanent outward push; only a *negative* main pressure balances it.
4. **The two gradients must differ in shape** (linear vs quadratic).
5. **Every force term carries `particleMass`.** Density is mass-scaled, so omitting it makes acceleration scale as `1/mass` — and mass is ~240, so the error is large and silent.
6. **Viscosity is XSPH, not a Müller Laplacian.** A non-negative weight cannot add energy; a Laplacian can.
7. **The time step is adaptive** (CFL from `sqrt(stiffness)`, max speed and max acceleration).
8. **Density uses Spiky Pow2, not Poly6.** Poly6 was retired from density and now serves as the viscosity weight.
9. **`h` is derived from spacing** (`h = 2.2 × spacing`), never hand-set.

---

## The trap list

Bugs that cost real time. All fixed, all worth knowing.

| Symptom | Cause |
|---|---|
| Fluid welded to walls (GPU) | Gradient functions have **no `r >= h` early-out** — safe on the CPU only because the neighbour grid guarantees `r < h` first |
| One particle at the centre of the screen | Job field was an **uncreated `NativeArray`**; it threw every frame, so the render buffer was never filled and all N instances drew at the origin |
| Black screen | `ParticleRenderer2D` was never attached; camera far clip sat exactly on the particle plane (`ndcZ = 1`) |
| Everything one flat colour | Colour ramp topped out at speed 6 while the fluid ran at 7–38, so 60–84% of particles clamped to the last stop |
| Everything a solid mesh | Impostor diameter was 1.7× the particle spacing, so discs fused |
| Silent 5–10× slowdown | 12 Burst errors; the grid ran as managed code because `NativeArray` was passed by value to a direct call |
| Texture silently unbound | `ColourMap` was an HLSL uniform but not declared in the shader's `Properties` |
| Scene reference resolved to `null` | A `ComputeShader` asset uses `fileID: 7200000`, not `4800000` as `Shader` assets do |
| Pass drawn twice / wrong pass | `Graphics.Blit` defaults to pass `-1` (multiple passes); `DrawMesh` cannot name a pass other than 0 |
| `submeshIndex out of range` | `DrawMeshInstancedIndirect`'s second argument is the **submesh**, not the shader pass |

A theme runs through most of these: **the failure was silent**. Wrong colour, wrong size, wrong pass, unbound texture, managed fallback — none of them error, they just quietly do the wrong thing.

---

## Configuration

The scene runs: `boxSize` 10×10, `particleCount` 5,000, spawn region 0.3×0.8 of the box, `stiffness` 2000, `restDensity` 1000, `viscosity` 50, `useBoundaryParticles` on, `autoVelocityMax` on, `autoParticleRadius` on.

Derived at spawn: `spacing` 0.0693, `h` 0.1524, `particleMass` 6.40, 1,172 boundary particles, a 68×68 grid, a CFL step of 0.5–0.7 ms, and ~33 substeps per 60 Hz frame.

Useful facts for tuning:
- **The physics constants are correct at any particle count.** `h/spacing` is constant and the SPH sums are consistent discretisations, so `stiffness`, `nearPressureMultiplier`, `viscosity` and `restDensity` do not need rescaling. (The viscosity term looks like it should scale as `1/N`; it does not, because the `dv` factor is odd about the centre and only the Laplacian survives.)
- **Anything measured per particle *does* need care** — that is why `particleRadius` is derived from spacing and the colour range follows the simulation's own peak speed.
- **`maxSubSteps` must exceed what a frame needs** (~33 at 5,000), or the fluid silently falls behind.
- **`minTimeStep` is a stability ceiling**: `dt = 0.0493/sqrt(N)`, so above ~60,000 particles it clamps and the CFL guarantee is lost.

---

## Where it is now

**Working and verified:** the whole GPU solver (density, both pressure channels, both force channels, XSPH viscosity, integration, boundaries), the grid, the adaptive step, instanced rendering straight from the solver's buffer, relative colour mapping, and 5,000-particle configuration.

**Not working:** the screen-space surface (`FluidSurface.shader`). The depth prepass is verified; the bilateral blur and composite are not — a background pixel reading 512 before the blur reads 0 after. `useScreenSpaceSurface` is off by default.

**Consequence:** T-033's "surface looks continuous rather than individual dots" is unmet. Particles can be *made* to look continuous by raising the impostor radius past ~1.13× spacing, but that trades away the particle look.

**Not started:** Phase 5, the 3D port. `Assets/Scripts/3D/` and `Assets/Shaders/3D/` hold only a README describing what carries over.

---

## Reading order

- `TASKS.md` — the task list, the invariants table, the GPU port decisions, and a note on every task where the original brief turned out to be wrong.
- `understanding/T-0XX.md` — one per task, pairing real code with questions about *why*. They are written as exercises, not answers.
- `WALKTHROUGH.md` — the long form of this file: the physics, the data flow, and every decision with its reasoning.
