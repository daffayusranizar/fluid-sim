# Fluid Sim — Walkthrough

The long form. `OVERVIEW.md` is the one-page version; this is the whole thing: what SPH actually computes, why each piece is shaped the way it is, how the two solvers differ, and every decision that had to be made along the way.

Read it top to bottom if you want the full picture, or jump to a section. Section 2 is the physics; sections 4–8 are the engineering; sections 9–11 are the accumulated judgement.

---

## 1. The problem SPH solves

A fluid obeys the Navier–Stokes equations. Solving them on a fixed grid (an *Eulerian* description) means tracking fields — velocity, pressure, density — at every point in space, and the grid has to be fine enough to resolve the smallest feature, which moves.

SPH takes the other option: a *Lagrangian* description. The fluid **is** a set of particles that move with the flow. Instead of asking "what is the velocity at this point?", it asks "what is the velocity of this particle, given its neighbours?" Any continuous field is reconstructed by summing over nearby particles, weighted by a smoothing kernel:

```
A(x) ≈ Σ_j  m_j / ρ_j  ·  A_j  ·  W(|x − x_j|, h)
```

`W` is the kernel, `h` the smoothing radius, `r = |x_i − x_j|`. The kernel is non-zero only inside `h`, so each particle only interacts with a handful of neighbours.

Two properties make this work:
- `∫ W dA = 1` — so a constant field reconstructs exactly.
- `W → 0` as `r → h` — so the sum is bounded and local.

Why particles are attractive here: free surfaces, splashing and fragmentation are trivial (there is nothing to track but the particles themselves), whereas on a grid they require interface tracking. The cost is that neighbour finding is the central performance problem — which is most of what this repo is about.

**This project is 2D.** 2D is not 3D with one number removed: the kernel normalisations differ (area vs volume), so every constant changes even though the shape of every equation does not. `TASKS.md` is explicit that the two are separate milestones, and `Assets/Scripts/3D/` is reserved and empty.

---

## 2. The maths, piece by piece

All of it lives in `SPHMath.cs` (CPU) and is transcribed into `SPH2D.compute` (GPU). That file knows nothing about particles or neighbours — it is pure kernel and equation-of-state maths, which is what makes it reusable and easy to check.

### 2.1 The kernels

Three kernels, and the *shape* of each is chosen for a specific job.

| Kernel | Formula (2D) | Normalisation | Used for |
|---|---|---|---|
| Spiky Pow2 | `const · (h − r)²` | `6 / (π h⁴)` | main density |
| Spiky Pow3 | `const · (h − r)³` | `10 / (π h⁵)` | near density |
| Poly6 | `const · (h² − r²)³` | `4 / (π h⁸)` | viscosity weight |

Two things to notice.

**The constants are 2D, and they are precomputed.** `π h⁴` is a `pow` call, and evaluating it per particle *pair* would dominate the cost. So each constant is computed once per step into `SolverParams` and passed in.

**Poly6 no longer computes density.** It did originally, and the name in older tasks reflects that. It was retired when the second pressure channel arrived, because a *point* value of Poly6 goes to zero smoothly at `r = 0` — which is exactly wrong for a kernel that needs a gradient (see the next section). Poly6 survives as the viscosity weight, where its non-negativity is the whole point.

The gradient shapes matter enough to state separately:

```
main gradient:  12/(π h⁴) · (h − r)      linear  →  can be negative past h
near gradient:  30/(π h⁵) · (h − r)²     quadratic →  always ≥ 0
```

**Neither gradient has an `r >= h` early-out.** The value kernels do (`if (r >= h) return 0`), the gradients do not. On the CPU that is safe, because the caller only ever reaches them through the neighbour grid, which has already guaranteed `r < h`. In a brute-force GPU loop it is not safe — `(h − r)` goes *negative* past `h`, silently producing an attractive force from a distant particle. This bit the GPU port and is recorded in T-028; the fix is an explicit `r < h` test at every call site, including the boundary loop.

### 2.2 Density

```
ρ_i = Σ_j m · W_main(r_ij)          main channel
ρn_i = Σ_j m · W_near(r_ij)         near channel
```

Two densities, from two kernels, over the same neighbours. Plus a boundary contribution:

```
ρ_i += Σ_b ρ₀ · V_b · W_main(r_ib)
```

Note `m` in the fluid term and `ρ₀ · V_b` in the boundary term — boundary particles stand in for a volume of solid at rest density, so they contribute as if they were fluid at rest.

**Why the boundary term matters more than it looks.** Near a wall, roughly half of a particle's kernel support lies inside the solid. Without the term, density is badly under-estimated, pressure follows it down, and particles weld themselves to the wall. That was T-020's bug, and it is the single most likely thing to be dropped by omission when porting.

Density is mass-scaled. That single fact has a consequence that shows up again in section 2.4.

### 2.3 Pressure — two channels

This is the part of the design that is hardest to guess and most important to get right.

```
main:  p_i = k · (ρ_i − ρ₀)              Tait equation of state, can go negative
near:  pn_i = kNear · ρn_i               no rest-density offset, never negative
```

A single-channel solver has to choose one of two bad options:

| Choice | Consequence |
|---|---|
| clamp `p ≥ 0` | fluid cannot cohere; free surfaces spray into droplets |
| allow `p < 0` | particles attract, pair up, and the surface collapses into clumps |

The two-channel scheme takes both. The near term is **always repulsive** and acts only at short range, which stops pairing. The main term is free to go negative, which gives a genuine two-way restoring force — push the fluid together and it pushes back, pull it apart and it pulls back. That is what lets it reach an actual equilibrium instead of being clamped into a one-way push.

**The gradients must differ in shape.** The near/main force ratio is about 2.5× at `r = 0`, falling to 0 at `r = h`. If both channels used the quadratic gradient, the near term would be a constant multiple of the main one and would raise the stiffness without ever suppressing short-range pairing — it would look like it was working while doing nothing useful.

**Consequence: the fluid settles ~1% below rest density.** At equilibrium the near channel is still pushing outward, and only a slightly negative main pressure balances it. That is the design, not drift. Which also explains the next point.

**Clamping and the near channel are mutually exclusive.** Clamping forbids negative main pressure, so nothing can balance the near push, and the fluid expands without limit. The failure presents as *particles flying apart*, which points at the force code — but the cause is a scalar flag interacting with a pressure offset. `SPH2D` warns on the combination.

The clamp is therefore a **flag** (`clampPressurePositive`, default off), not a hardcoded `max(0, p)`, and must be ported as a flag.

### 2.4 Forces

Two force channels, both between fluid particles:

```
F_i += Σ_j  dir_ij · m · ∇W_main(r_ij) · (p_i + p_j)/2 / ρ_j
F_i += Σ_j  dir_ij · m · ∇W_near(r_ij) · (pn_i + pn_j)/2 / ρn_j
```

where `dir_ij = (x_i − x_j) / r` points from `j` toward `i`, so a positive term pushes `i` away from `j`.

Pressure is **symmetrised** — the average of the two particles' values — which keeps the pair force equal and opposite. The divisor is the *neighbour's* density, which is what makes the discretisation consistent.

Then the wall, by **pressure mirroring**:

```
F_i += Σ_b  dir_ib · V_b · max(0, p_i) · ∇W_main(r_ib)
```

The wall copies the particle's own pressure rather than having a pressure of its own. Two details:

- The clamp here is `max(0, p_i)` **unconditionally** — independent of `clampPressurePositive`. A particle's own pressure is often negative near a wall (density below rest), and a mirrored *negative* pressure would make the wall **attractive**. The wall may push but never pull.
- There is no `m` and no density divisor; `V_b` replaces both.

**Every force term carries `particleMass`.** This looks redundant and is not. Density is mass-scaled (`ρ = Σ m W`), so acceleration is `F/ρ ∝ m·(…) / (m·ΣW)` — the mass cancels and acceleration is mass-independent. Drop the `m` from the force and it no longer cancels: acceleration becomes proportional to `1/m`. In this project mass is ~240 (at 400 particles) or ~6.4 (at 5000), so the error is both large and silent. This is the kind of bug that produces fluid that "looks a bit wrong" rather than anything diagnosable.

### 2.5 Viscosity

```
F_i += μ · Σ_j (v_j − v_i) · W_poly6(r_ij)
```

This is **XSPH**, not Müller's Laplacian, and the difference matters.

Müller's formulation differentiates the kernel twice: `∇²W`, whose sign can flip. A term that can change sign can *add* energy — a viscosity that accelerates. So Müller's version needs a dedicated kernel constructed to be non-negative.

XSPH sidesteps the whole problem: it uses the kernel as a **weight** on the velocity difference. A weighted average toward your neighbours' velocities cannot add energy, no matter how tightly particles are packed, because a non-negative weight has no sign to get wrong. It also needs no square root.

Viscosity is the **only dissipative term** in the solver. Remove it and the fluid sloshes forever.

### 2.6 Integration and the time step

Semi-implicit Euler:

```
a = F/ρ + g
v ← v + a·dt
x ← x + v·dt
```

Velocity is updated first and the new velocity is used for the position. Explicit Euler (using the old velocity) injects energy and explodes at these stiffnesses.

The step size is **adaptive**, from a CFL condition:

```
dt_cfl = C · h / (max‖v‖ + c_s)          c_s = √k   (sound speed from the EOS)
dt_acc = C · √(h / max‖a‖)
dt     = min(dt_cfl, dt_acc)
```

The first limit says information must not cross more than one smoothing length per step. The second says a particle must not be flung across its own neighbourhood. A stiffer fluid raises `c_s` and forces a smaller step — the central trade-off of weakly compressible SPH.

This needs `max‖v‖` and `max‖a‖` over all particles, which is a CPU-side quantity by definition. T-029 had to reconcile that with "zero CPU readback"; see section 6.3.

Then the wall clamp:

```
if x beyond the box:  x = wall;  v *= −collisionDamping
```

This is only a **penetration safety net**, not the wall model — boundary particles do the physics. Its job is to guarantee nothing escapes if a step overshoots.

### 2.7 Boundaries

Boundary particles are static samples inside the solid, just outside each wall. They:

- complete the kernel support near the wall (the density term),
- push back when the fluid compresses against them (the mirroring term).

They never move, never integrate, and need one scalar (`boundaryVolume` = `spacing²`) rather than a full state. That is why they cost "one buffer and one constant" on the GPU rather than a parallel subsystem.

They are indexed in the **same** spatial structure as the fluid particles, packed after them — `[0, N)` fluid, `[N, N+B)` boundary. That mirrors the CPU grid's scheme, and it matters for performance: leaving boundary particles out keeps the wall loop at `O(N·B)`, and since `B ∝ perimeter/spacing ∝ √N`, that is `O(N^1.5)` — barely visible when the fluid query is `O(N²)`, and the dominant term once it is not.

---

## 3. The step, in full

```
                          ┌───────────────────────────────┐
                          │ positions                     │
                          └───────────────┬───────────────┘
                                          ▼
   ┌──────────────────────────────────────────────────────────────────┐
   │ 1. spatial structure                                             │
   │    CPU: uniform grid + CSR neighbour lists                       │
   │    GPU: cell keys → count sort → bucket offsets + sorted indices │
   └──────────────┬───────────────────────────────────────────────────┘
                  ▼
   2. density            neighbours → ρ, ρn
                  ▼
   3. pressure           ρ → p, pn
                  ▼
   4. pressure force     p, positions → F          (main + near + wall)
                  ▼
   5. viscosity force    velocities → F +=         (XSPH)
                  ▼
   6. reduce extremes    F, v → max‖v‖, max‖a‖     → dt
                  ▼
   7. integrate          F → v, x                  (+ wall clamp)
                  ▼
   8. render             x, v → pixels             (binds the same buffer)
```

Steps 1–7 are one "step". The driver runs as many per frame as the frame budget allows, each with its own `dt`.

Ordering is load-bearing at three points:
- pressure needs density,
- viscosity **reads `F` and adds to it**, so it must follow the pressure force,
- integration needs the finished force, and the reduction must read the *pre-integration* state to match what the CPU measures.

---

## 4. Memory layout, and why it shapes everything

There are two layouts, and the difference is not cosmetic.

**CPU: structure of arrays (SoA)** — `ParticleState` holds seven separate arrays.

**GPU: array of structs (AoS)** — one 40-byte `Particle2D`, matching the shader's struct field for field.

The CPU is SoA for three reasons, in order of importance:

1. **No data race.** Each pass writes exactly one array and reads others. With an array of structs, writing `particles[i]` rewrites position and velocity too, while another thread reads `particles[j].position` as a neighbour. The race is benign — the values rewritten are the same ones — and that is precisely why it should be removed: correctness would rest on a coincidence.
2. **No aliasing.** When one array is both read and written, Burst must assume the pointers may overlap, which blocks vectorisation. Separate arrays remove the ambiguity.
3. **It is where the GPU was heading anyway.**

The GPU is AoS because a compute shader wants one structured buffer with a fixed stride, and because that same 40-byte struct is what the renderer uploads. The buffer the simulation writes is the buffer the renderer reads — no repacking.

**But derived quantities are separate buffers on both sides.** Density, near-density, pressure, near-pressure and force are not in `Particle2D`. The reason is the same in both cases: the density pass reads every particle and writes only its own. Putting density in the particle struct would have each thread writing a field another thread reads. The race is benign — density is not a value the density pass uses — and again, that is the argument against it.

The cost is a few extra buffers. The benefit is that every pass has exactly one output, which is what both Burst and the GPU scheduler want.

---

## 5. The CPU solver

Kept as the **reference implementation** and as a fallback. Files: `SPH2D` (state, spawn, orchestration), `SPHGrid` (grid), `SPHJobs` (passes), `SPHSolver` (params + reductions).

**Grid.** A uniform grid with direct index arithmetic — `floor((x − origin) / cellSize)` — laid out as a count sort: count per cell, prefix sum to get offsets, scatter into a packed array. Neighbours come out as two CSR lists (fluid, boundary) where particle `i` owns the range `[start[i], start[i+1])`.

Cell size is **exactly `h`**, and that is a correctness requirement, not a tuning choice: the 3×3 query is only guaranteed complete if cell size ≥ `h`. The domain is the container plus `h` of margin, so boundary particles land *inside* the grid rather than being clamped into edge cells.

Out-of-grid cells are **skipped, never clamped**. Clamping would make `KeyForCell` return the edge cell for several deltas, so an edge particle would scan the same bucket up to four times and double-count its occupants.

**Passes.** Seven `IJobParallelFor` jobs: reset forces, density, pressure, pressure force, viscosity, integrate, and repack for rendering. Dependencies are expressed as `JobHandle` chains rather than an implied order.

`ResetForces` is **redundant** — the pressure force pass assigns rather than accumulates — and is deliberately not ported to the GPU, because carrying a no-op across would obscure that.

**Burst.** The grid ops and the reductions are `[BurstCompile]` static methods called directly from managed code. Burst rejects struct parameters passed by value in a direct-call thunk, which is why the write parameters are `ref` and the read ones `in`, and why `SolverParams`'s `bool` fields carry `[MarshalAs(UnmanagedType.U1)]`. Getting this wrong does not error at runtime — it silently falls back to managed code, which is how a 5–10× slowdown hid for a whole task.

---

## 6. The GPU solver

Files: `SPHCompute` (buffers, dispatch, readback), `SPHComputeSimulation` (the frame loop), `SPH2D.compute` (13 kernels).

### 6.1 Ping-pong state

Two particle buffers. Integration reads the current one and writes the other, then the index swaps. A single buffer would have every thread reading a slot another thread is overwriting.

The swap is an index flip, not a copy.

### 6.2 Dispatch order and dependencies

The step is five dispatches plus the grid's five:

```
grid:   keys → clear → histogram → prefix sum → scatter
solve:  density+pressure → force+viscosity → reduce → integrate
```

`ComputeDensity` and `ComputePressure` are separate kernels, and the force and viscosity are separate, even though fusing them would work. The split is what makes "does the GPU match the CPU" a *per-stage* question: if density is already wrong, a plausible-looking pressure is meaningless.

Ordering is guaranteed by Unity recording dispatches on one queue with UAV barriers between passes that touch the same buffer — the GPU equivalent of the CPU's `JobHandle` chain.

### 6.3 The adaptive step without readback

This was the one genuine architectural fork. "Zero CPU readback" conflicts with the CFL step, which needs `max‖v‖` and `max‖a‖`.

Three options: a GPU reduction with an async readback consumed a frame late; a GPU-resident `dt`; or a fixed conservative `dt`. The third would have silently deleted T-021's adaptive behaviour, so it was rejected. The first was chosen:

- a `ReduceExtremes` kernel does an integer `InterlockedMax` on the float **bit patterns**, which is valid only because both quantities are magnitudes, where a non-negative IEEE bit pattern is monotonic as an unsigned int;
- the result is read back asynchronously, one request per frame, skipped while one is in flight so readbacks cannot pile up.

The one-frame lag costs nothing real, because the CPU already sized `dt` from the previous step's state.

The readback is **poll-based**, not callback-based — a callback only fires while the player loop is pumping, which makes it untestable outside play mode.

### 6.4 The grid

Same count sort as the CPU, in five kernels, producing `_BucketStart[bucketCount+1]` and `_SortedIndices[total]`.

The prefix sum is a single workgroup walking tiles in order, carrying a running total, with a Hillis–Steele scan inside each tile. One workgroup suffices because `bucketCount ≈ N/4.8` (~2,300 at N=10,000); a multi-workgroup scan would pay off an order of magnitude higher.

**Direct index versus hash.** T-030's brief mandated a prime-multiplier spatial hash. But a hash exists to represent *unbounded* domains and pays for that with collisions, and this domain is a bounded box. Measured at N=400:

```
DirectIndex   441 buckets   94 distinct cells   94 occupied   max occupancy 7
SpatialHash   801 buckets   94 distinct cells   89 occupied   max occupancy 11
```

The direct index used 45% less memory with zero collisions. And a bigger hash table does not reduce candidates per query: with cell size `= h` a cell holds ~4.8 particles either way, so a 3×3 scan sees the same ~43 candidates and the hash only adds false positives. Both are implemented behind one function; the direct index is the default, and the hash is kept because it is what an unbounded domain would need.

### 6.5 The verification discipline

Every GPU task since T-026 follows the same pattern: **run both implementations on identical input and diff the results.** Not "it looks right" — compare numbers.

```
T-027  density      match to 2.6e-7 relative, both EOS branches
T-028  forces       match to 9.0e-6, viscosity exercised, integration to 2.3e-6
T-031  10 CPU/GPU steps   maxPosErr 9.5e-7
T-032  render       6411 lit pixels, spread measured
```

Two lessons came out of it, both about **tests that pass while proving nothing**:

- The viscosity check first compared `0` against `0`, because spawn leaves every velocity at zero and XSPH blends velocity *differences*. The test had to perturb velocities before it meant anything.
- The first benchmark reported ~0.01 ms/step at every particle count including 20,000. Compute dispatches are asynchronous, so it was timing CPU *submission*. A blocking readback of a buffer the last `Integrate` wrote forces completion.

And when a test had produced a false green once, it was mutation-checked: the `r < h` guard was deliberately removed, force went to `1e9` against the CPU's `1e6`, and the test went red. A test only ever seen green is not evidence.

---

## 7. Why `O(N²)` came first

The GPU kernels were brute force for four tasks (T-026 to T-029): every thread looped every particle and relied on the kernel returning zero beyond `h`.

That is `O(N²)` and obviously wrong for performance. It was still the right order of work, because the alternative was debugging a count sort, a prefix scan and a hashing scheme *at the same time* as the first correctness of the kernels. Brute force produces the same numbers — the kernel is zero past `h` anyway — so it made a stable reference to port against.

The payoff shows in section 12: gridding the three heavy kernels was worth **27.8× at 10,000 particles** — by far the largest single win in the project.

---

## 8. Rendering

One instanced draw call, reading the solver's own particle buffer.

### 8.1 Binding versus uploading

The renderer originally packed the CPU state and uploaded it with `SetData` every frame. At 50,000 particles that is 2 MB per frame of pure copying, and worse, it forces the simulation state to exist on the CPU at all. T-032 changed it to bind the solver's `ComputeBuffer` directly, so a GPU simulation renders with no readback and no upload. The CPU path is kept and is the automatic fallback (when no `SPHCompute` is initialised, the renderer packs from `SPH2D` instead), so the two solvers can be compared side by side.

### 8.2 The spherical impostor

Each particle is a quad, but the fragment reconstructs the front face of a **hemisphere** rather than a flat disc:

```
h = sqrt(max(0, 1 − r²))
worldFront = (centre + local·discRadius, −h·discRadius)
```

That gives two things a flat quad cannot: a **normal** (so a particle reads as a lit sphere) and a **depth that varies across the face**.

The depth is the load-bearing one. The quad used to be built at `z = 0` with `ZWrite Off`, so *every fragment of every particle had the same depth*. A bilateral blur over a constant field, and normals reconstructed from screen-space derivatives of it, would both see a plane. Screen-space smoothing has nothing to work with until the impostor has a curved depth profile — which is why that had to land before the blur.

The world-space disc radius is **half** the `particleRadius` parameter, because the quad's vertices span ±0.5 before scaling. That is easy to get wrong in both directions.

### 8.3 Colour

Colour comes from a 1D gradient texture sampled by speed. Two things had to be fixed:

**The range must be relative, not absolute.** With a fixed top speed of 6 against a fluid that runs at 7–12 median with peaks of 30–75, `t = saturate(speed / 6)` pinned 60–84% of particles to the gradient's final stop. A six-colour ramp rendered as one flat colour. The fix is to derive the range from the simulation's own peak speed — which the extremes reduction already computes every frame for the CFL step, so it is free — with a padding of 0.5 (the peak is outlier-dominated) and a floor (without one, a fluid at rest collapses the range to zero and puts everything at the *top* stop).

**A three-stop blue→green→red lerp goes muddy.** Green→red passes through olive and blue→green through teal, so the midpoints desaturate. Going round the wheel — blue, cyan, green, yellow, orange, red — keeps every stop saturated. That is what makes speed legible in fine detail rather than only at the ends.

### 8.4 Impostor size is a ratio

Disc **diameter** relative to particle spacing decides whether the fluid reads as particles or as a sheet:

```
below ~1.13    gaps between discs    → separate particles
~1.13          discs just tile       → one continuous sheet
above that     overlap into a mass   → a solid blob
```

A fixed world-space radius means different things at different counts: `0.12` was 0.49× spacing at 400 particles (dots) and 1.70× at 5,000 (a mesh). It is now derived from the spacing, which is recoverable from `h` because `h = 2.2 × spacing`.

This is the *same class of bug* as the colour range: a hand-set absolute value whose meaning silently changes when the configuration does. Both were fixed the same way — derive from something the solver already knows.

### 8.5 The screen-space surface — WIP, not working

The intended pipeline, in design order:

```
depth prepass (min-blend, RFloat)  →  bilateral blur  →  normals from blurred depth
                                   →  shade  →  full-screen composite
```

The depth prepass **is verified**: background reads exactly the clear value, the surface reads the right view depth, and the depth varies across the face. The blur and composite are **not**: after the horizontal blur a background pixel that reads 512.000 in the source reads 0.000, and the far-pixel count falls from 37624 to 9495 while the maximum stays 512 — a partial write the filter logic cannot produce. The bug is in the `RFloat` render target plus `Graphics.Blit` path, below the filter maths.

`useScreenSpaceSurface` is off by default. The consequence is that T-033's "surface looks continuous rather than individual dots" is unmet.

---

## 9. The invariants

Things that must not silently change. Recorded in `TASKS.md` with where each lives and which task could break it. They exist because Phases 2–4 are *performance* work and were not licence to change physics.

| Invariant | Why it matters | At risk from |
|---|---|---|
| Boundary particles complete kernel support near walls and push back by mirroring | Without them, particles weld to walls | grid must index them; must be ported to GPU |
| Viscosity is the only dissipative term | Without it the fluid never settles | any port that drops it |
| Clamping and the near channel are mutually exclusive | Clamping removes the only thing that can balance the near push; the fluid expands without limit | porting one without the other |
| Two pressure channels, with *different* gradient shapes | Same shape = near term cannot suppress pairing | porting one channel |
| Every force term carries `particleMass` | Density is mass-scaled; omitting it makes acceleration scale as `1/mass` | transcription to HLSL |
| Viscosity is XSPH, not a Laplacian | A Laplacian can add energy; a non-negative weight cannot | following the old task brief |
| Adaptive CFL step | A fixed step deletes T-021's work and can go unstable | "zero readback" |
| Density uses Spiky Pow2, not Poly6 | Poly6's point value vanishes at `r = 0`, which is wrong where a gradient is needed | following the old task brief |
| `h` derived from spacing | Keeps neighbour count sane at any particle count | any phase that changes spawn |

**A theme worth naming:** several of these were nearly broken by following the task's own written brief, which was written before the design changed. The invariants table, not the prose, is the specification.

---

## 10. Decisions on record

`TASKS.md` carries a **GPU Port Decisions** table, because these are choices rather than accidents.

| # | Decision | Chosen | Reason |
|---|---|---|---|
| 1 | Boundary particles in the grid (T-023) | shared grid | cell membership is static; a second grid duplicates the traversal, and excluding them breaks wall behaviour |
| 2 | Boundary particles on GPU (T-026) | static buffer | dropping them reintroduces wall welding; cost is one buffer and one scalar |
| 3 | Boundary push on GPU (T-028) | pressure mirroring | matches the CPU and is diffed against it, so the two cannot drift |
| 4 | Adaptive step under zero readback (T-029) | GPU reduction + async readback | keeps the CFL step; the lag is free because `dt` was already one step old |

Others recorded in-line in the relevant tasks:

- **Direct index over spatial hash** (T-030) — measured; see section 6.4.
- **ResetForces not ported** (T-028) — redundant on the CPU, and carrying a no-op across obscures that.
- **Priority of `r < h` tests** — the gradients are unguarded, so every call site tests (T-028).
- **Relative colour range and derived impostor radius** (T-033) — absolute values silently change meaning when the configuration does.

---

## 11. The trap list

Every one of these cost real debugging time. The pattern in almost all of them is **silent failure**: nothing errors, the result is just quietly wrong.

| Symptom | Cause | Lesson |
|---|---|---|
| Fluid welds to walls on the GPU | Gradient functions have no `r >= h` early-out; safe on the CPU only because the grid guarantees `r < h` | A ported function inherits its *caller's* guarantees — check them when the caller changes |
| One particle at the centre of the screen | A job field was an uncreated `NativeArray`. It threw every frame, so the render buffer was never filled and all N instances drew at the origin | A per-frame exception in a subsystem reads as a *rendering* bug elsewhere. Check the log first |
| Black screen | The renderer was never attached to the scene; and the camera's far clip sat exactly on the particle plane (`ndcZ = 1`) | Scene wiring is not code. Verify the scene, not just the scripts |
| Everything one flat colour | Ramp topped out at 6 while the fluid ran at 7–38 | An absolute parameter whose meaning depends on the data |
| Everything a solid mesh | Impostor diameter 1.7× the spacing | Same class: absolute size against a data-dependent space |
| 5–10× slowdown, no error | 12 Burst errors; the grid ran as managed code because `NativeArray` was passed by value to a direct call | Compile-time fallbacks are invisible. Count your errors |
| Texture silently unbound | `ColourMap` was an HLSL uniform but not in the shader's `Properties` block | A reimport drops what was never declared |
| Scene reference resolved to `null` | A `ComputeShader` uses `fileID: 7200000`, not `4800000` | Wrong ids deserialise to null, silently |
| Pass ran twice / wrong pass | `Graphics.Blit` defaults to pass `-1` (multiple passes); `DrawMesh` cannot name a pass other than 0 | Defaulted parameters hide intent |
| `submeshIndex out of range` | `DrawMeshInstancedIndirect`'s 2nd argument is the submesh, not the shader pass | Check the signature, not the parameter order you expected |
| Shadow scene values, stale enum fields | The scene had not been saved since T-018; fields that shared names with current ones silently overrode tuned defaults | A scene is a data file that rots |

---

## 12. Performance, measured

Apple M5, headless, Metal.

### Solver step

| N | brute force | gridded | speedup |
|---|---|---|---|
| 400 | 1.29 ms | 0.54 ms | 2.4× |
| 2,000 | 3.18 ms | 0.64 ms | 5.0× |
| 5,000 | 6.57 ms | 0.63 ms | 10.4× |
| 10,000 | 29.25 ms | **1.05 ms** | **27.8×** |
| 20,000 | — | 1.75 ms | — |
| 50,000 | — | 3.65 ms | — |

At the scene's configuration (10×10 box, 0.3×0.8 spawn region, 5,000 particles) a step costs **0.18–0.57 ms**, falling as the fluid settles because the CFL step grows with it.

The CPU Burst solver costs **15.7 ms** at 5,000 — the GPU is ~25× faster at equal count.

### Where the limits actually are

**Memory is not one.** ~130 bytes/particle: 5,000 is ~0.7 MB, 100,000 would be ~13 MB.

**Step count is.** `h` falls as `1/√N`, so `dt` falls with it, and the number of steps needed per second of simulated fluid rises as `√N`. At 10,000 particles you need ~2,000 steps per simulated second. That is why 5,000 runs at roughly real time and 10,000 at about half speed, even though the *solve* is only 1 ms.

**A hard ceiling exists** at `minTimeStep`: `dt = 0.0493/√N`, so past ~60,000 particles `dt` clamps and the CFL guarantee is lost. Lower `minTimeStep` to move it.

### What is still on the table

The 3×3 neighbourhood scan runs **three times** per step — in density, force and viscosity. Caching or fusing it would cut that substantially. Not attempted: it is an optimisation, not a correctness task.

---

## 13. Configuration and tuning

What the scene runs, and what is derived from it.

**Set:** `boxSize` 10×10, `particleCount` 5,000, `spawnRegionSize` (0.3, 0.8), `spawnRegionCenter` (−1, 0), `restDensity` 1000, `stiffness` 2000, `viscosity` 50, `nearPressureMultiplier` 20, `maxSubSteps` 64, `cflFactor` 0.25.

**Derived at spawn:** `spacing` 0.0693, `h` 0.1524, `particleMass` 6.40, `boundaryVolume` 0.0048, 1,172 boundary particles, a 68×68 CPU grid (4,624 buckets), a CFL step of 0.5–0.7 ms, ~33 substeps per 60 Hz frame.

**The physics constants are correct at any particle count**, and it is worth knowing why rather than assuming it: `h = 2.2 × spacing`, so `h/spacing` is constant, and the SPH sums are consistent discretisations. `stiffness`, `nearPressureMultiplier`, `viscosity`, `restDensity` and `cflFactor` do not need rescaling.

The viscosity term looks like the exception and is not. XSPH is a smoothing operator rather than a consistent derivative, so `Σ(Δv)·W` seems like it should scale as `1/spacing²` — which would mean dropping `viscosity` from 50 to ~4 at 5,000 particles. It does not, because `Δv` is odd about the centre: that leading term cancels, and the surviving term is

```
Σ_j (v_j − v_i)·W_ij  →  h²·∇²v / (20·spacing²)
```

with `h²/spacing² = 4.84` constant.

**Knobs that matter most:**

- `maxSubSteps` must exceed what a frame needs (~33 at 5,000) or the fluid silently falls behind.
- `autoParticleRadius` and `autoVelocityMax` — leave both on; they are what keep appearance independent of count.
- `particleRadiusInSpacing` ~0.8 for particles, ~1.13 for a continuous sheet.
- `stiffness` is the main physical trade: higher means more incompressible and a smaller step.
- `debugLogs` prints `steps` and `dt` per frame, which is the real indicator of whether the solver is keeping up.

---

## 14. State, and the 3D port

**Done and verified:** the GPU solver end to end (density, both pressure channels, both force channels, XSPH viscosity, integration, boundaries), the count-sort grid, the adaptive step with async readback, instanced rendering from the solver's own buffer, relative colour mapping, derived impostor sizing, and a 5,000-particle configuration.

**Done but unverified:** nothing important — verification has been per-task throughout.

**Not working:** the screen-space surface. Depth prepass verified; blur and composite not.

**Not started:** Phase 5.

When the 3D port begins, four things are worth carrying over rather than rediscovering:

1. **Normalisation changes everywhere.** 2D normalises over area (`h⁴`, `h⁸`), 3D over volume (`h⁶`). Every constant in `SPHMath` changes while the shape of every equation does not.
2. **The two traps**: the gradient functions' missing `r >= h` early-out, and the unconditionally-clamped mirrored boundary pressure.
3. **The invariants table**, which is the specification — several of its entries were nearly broken by following a task brief written before the design settled.
4. **Copy, don't abstract.** A dimension-parameterised `SPHMathBase` would make both solvers harder to read than the duplication costs.

And do not port `FluidSurface.shader` until it works in 2D.
