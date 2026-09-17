# Development Tasks

Private task tracking for the fluid simulation project.

T-001 to T-007 were the research/planning phase and are already complete. Those tasks led to the decision to build a 2D SPH fluid simulation in Unity on macOS. Everything below is the actual implementation, starting at T-008.

Critical rule: 2D first, 3D later. Master the 2D solver completely before starting any 3D work. The 2D and 3D implementations are separate milestones. Do not mix them.

How to use: Each task follows the pattern Study → Implement → Validate. Check off tasks as you complete them. Don't skip ahead — each phase builds on the previous one.

---

## Phase 0: Prerequisites & Setup (T-008 to T-014)

These tasks must be completed before writing any SPH code.

### Phase Summary
You cannot implement SPH if you don't yet understand the Unity basics, the math you'll need, or how to debug particle systems. This phase builds that foundation.

- [] T-008 — Unity Project Structure & Workflow
- [ ] T-009 — MonoBehaviour Lifecycle & the Update Loop
- [ ] T-010 — Vector Math Refresher for Physics
- [ ] T-011 — Debug Visualization with Gizmos
- [ ] T-012 — What Is SPH? (Theory Study)
- [ ] T-013 — Smoothing Kernels Theory
- [ ] T-014 — SPH Force Model Theory

Task Code: T-008
* Quest : Understand how a Unity project is organized and how to navigate between Scene view, Game view, Project window, and Inspector. Know where scripts, scenes, and assets live.
* Guide : Open SampleScene.unity. Create an empty GameObject. Attach SPH2D.cs to it. Use the Inspector to modify boxSize, numToSpawn, and gravity. Press Play and observe how values chlange behavior. Read Unity's manual on Project Structure.
* Done : You can create GameObjects, attach components, modify public fields in the Inspector, and understand what happens when you press Play vs. entering Play mode.

Task Code: T-009
* Quest : Understand the MonoBehaviour execution order: Awake(), OnEnable(), Start(), Update(), LateUpdate(), FixedUpdate(), OnDisable(), OnDestroy().
* Guide : Add Debug.Log statements to each lifecycle method in SPH2D.cs. Run the scene and observe the order in the Console. Experiment with moving code from Update() to FixedUpdate() and observe how physics behavior changes. Study Unity's Script Execution Order.
* Done : You can explain when each lifecycle method runs, why physics code lives in FixedUpdate(), and why Awake() is used for initialization.

Task Code: T-010
* Quest : Refresh the vector math needed for SPH: vector addition, subtraction, dot product, cross product (2D), magnitude, normalization, and distance between two points.
* Guide : In SPH2D.cs, write helper methods that compute: distance between two particles, dot product of two velocity vectors, and the normalized direction from particle A to B. Verify your math matches Unity's built-in Vector2 methods (Distance, Dot, normalized). Study the math behind Vector2 operations.
* Done : You can manually compute the distance, dot product, and direction between two 2D points and verify your results match Unity's built-in methods.

Task Code: T-011
* Quest : Learn to use OnDrawGizmos() and Gizmos class to visualize debug information in the Scene view.
* Guide : In SPH2D.cs, add Gizmos that draw: (1) a line from each particle to the center of the box, (2) a colored sphere for each particle colored by its speed (red = fast, green = slow), (3) a text label showing particle count. Study Unity's Gizmos documentation.
* Done : You can use Gizmos to visualize any per-particle data in the Scene view without creating GameObjects.

Task Code: T-012
* Quest : Understand what SPH is, why it exists, and how it approximates fluid behavior with particles.
* Guide : Read about the Navier-Stokes equations and why they're hard to solve directly. Study the concept of Lagrangian vs. Eulerian fluid description. Understand why SPH uses particles instead of a grid. Watch introductory videos on SPH fluid simulation. Take notes in your own words.
* Done : You can explain in plain English: what problem SPH solves, why particles are used, and what "smoothed particle hydrodynamics" means.

Task Code: T-013
* Quest : Learn the three SPH smoothing kernels: Poly6, Spiky, and Viscosity. Understand what each one does and why they have different formulas.
* Guide : Study the kernel formulas from your README. Write each kernel function in C# as a standalone static method (input: float r, float h; output: float). Test them by plotting their values from r = 0 to r = h. Verify that Poly6 is smooth at r = 0, Spiky has a non-zero gradient at r = 0, and Viscosity is a linear ramp.
* Done : You can implement each 2D kernel in C#, explain what it's used for, and show a graph or debug output of its shape.

Task Code: T-014
* Quest : Understand the SPH force model: how density is computed, how pressure is derived from density, and how pressure + viscosity forces are applied to particles.
* Guide : Study the full SPH loop: (1) find neighbors, (2) compute density using Poly6, (3) compute pressure using Tait EOS, (4) compute pressure force using Spiky gradient, (5) compute viscosity force using Viscosity Laplacian, (6) integrate forces into velocity and position. Draw the data flow on paper. Understand which quantities are computed per-particle and which require pairwise iteration.
* Done : You can draw the complete SPH data flow diagram and explain what each step computes and why.

---

## Phase 1: First Working SPH — Core Loop (T-015 to T-022)

This is where you write your first actual SPH solver. Start simple. Use brute-force O(N²) neighbor search. Get something working before optimizing.

### Phase Summary
Build a minimal but complete SPH solver on the CPU. At the end of this phase, you should see particles behaving like a fluid, not just independent dots under gravity.

- [ ] T-015 — Brute-Force Neighbor Search
- [ ] T-016 — Density Calculation with Poly6 Kernel
- [ ] T-017 — Pressure from Density (Tait Equation of State)
- [ ] T-018 — Pressure Force with Spiky Gradient
- [ ] T-019 — Viscosity Force with Laplacian Kernel
- [ ] T-020 — Boundary Collision Handling
- [ ] T-021 — Stable Time Integration
- [ ] T-022 — First Visual Validation

Task Code: T-015
* Quest : Implement a brute-force neighbor search that finds all particle pairs within the smoothing radius h.
* Guide : Add a smoothingRadius field to SPH2D. In Update(), before doing anything else, loop over every pair of particles (i, j) where i != j. If distance(i, j) < smoothingRadius, record j as a neighbor of i. Store neighbors in a List<int>[] or similar structure. Study why brute-force is O(N²) and why that's okay for N < 200 while learning.
* Done : You can print the neighbor count for each particle and see that nearby particles are detected. With 100 particles, the simulation still runs at 60 FPS.

Task Code: T-016
* Quest : Compute particle density using the Poly6 kernel.
* Guide : For each particle i, sum the Poly6 kernel contribution from all its neighbors: ρ_i = Σ m_j * W_poly6(|r_i - r_j|, h). Use the 2D Poly6 formula from your README. Add a restDensity field. Store the result in particles[i].density. Study why density is the foundation of SPH — every other force depends on it.
* Done : Particles have non-zero densities. Rest-density particles (isolated or in a flat surface) settle near restDensity. You can visualize density by coloring Gizmos.

Task Code: T-017
* Quest : Compute pressure from density using the Tait Equation of State.
* Guide : For each particle, compute pressure = stiffness * (density - restDensity). If density is below restDensity, pressure should be zero or negative depending on your EOS variant. Add stiffness and restDensity as public fields. Study the Tait EOS and why it creates a stiff pressure response — this is what makes SPH fluids feel "incompressible."
* Done : Particles under compression (high density) develop positive pressure. You can visualize pressure as a color gradient on Gizmos.

Task Code: T-018
* Quest : Apply pressure forces between neighboring particles using the Spiky kernel gradient.
* Guide : For each particle i, sum the Spiky gradient contributions from all neighbors: F_pressure_i = -Σ m_j * (P_i + P_j) / (2 * ρ_j) * ∇W_spiky(r_ij, h). The gradient points from neighbor j toward particle i. Add the result to particles[i].force. Study why the Spiky kernel is used for pressure (its gradient doesn't vanish at r = 0, preventing particle clumping).
* Done : Particles push away from each other when compressed. A column of particles spreads out horizontally instead of stacking like a solid. Fluid begins to behave like a fluid.

Task Code: T-019
* Quest : Apply viscosity forces between neighboring particles using the Viscosity kernel Laplacian.
* Guide : For each particle i, sum viscosity contributions: F_visc_i = μ * Σ m_j * (v_j - v_i) / ρ_j * ∇²W_visc(r_ij, h). Add viscosity as a public field. Study how viscosity diffuses velocity — it makes neighboring particles move at similar speeds, creating smooth flow instead of chaotic motion.
* Done : Fluid motion is smoother. Particles don't jitter as much. A falling stream of particles stays coherent instead of exploding into chaos.

Task Code: T-020
* Quest : Implement boundary collision handling so particles stay inside the container.
* Guide : For each particle, check if it's outside the box bounds defined by boxSize. If so, push it back inside and apply a velocity reflection with damping. Study different boundary models: simple position clamping, velocity reflection, and penalty forces. Understand why simple position clamping can cause energy injection.
* Done : Particles bounce off walls and floor. They settle at the bottom of the container. No particles escape the box.

Task Code: T-021
* Quest : Implement stable time integration using Semi-Implicit Euler with substeps.
* Guide : Use the integration order: (1) compute all forces, (2) update velocity v += (F/m) * dt, (3) update position x += v * dt. Add a maxSubSteps field. If Time.deltaTime is too large, split the frame into multiple smaller substeps. Study why explicit Euler explodes with stiff forces and why Semi-Implicit Euler is more stable for SPH.
* Done : The simulation remains stable even with high stiffness values. No particle explosions or runaway velocities.

Task Code: T-022
* Quest : Validate that your SPH solver produces visually plausible fluid behavior.
* Guide : Spawn a column of particles on one side of the container (dam break setup). Observe: does the column collapse under gravity? Does the fluid flow across the floor? Does it splash against the opposite wall? Does it settle into a calm pool? Adjust parameters (stiffness, viscosity, particleSpacing, dt) until behavior looks reasonable. Document what works and what doesn't.
* Done : You can run a dam break scenario and see fluid-like behavior: collapse, flow, splash, settle. You have a working 2D SPH solver.

---

## Phase 1 Invariants — Do Not Silently Regress These

Phase 1 produced several behaviours that the later phases must preserve. Phases 2–4 are *performance* work; they are not licence to change physics. Each item below is easy to drop by omission, because the optimisation tasks never mention it.

| Invariant | Where it lives now | Where it is at risk |
|---|---|---|
| **Boundary particles** complete the kernel support near walls and push back via pressure mirroring (`p_b = p_i`). This is what stops particles welding to walls. | `SpawnBoundaryParticles()`, `ComputeDensity()`, `ComputePressureForce()` | T-023 (grid must index them), T-026–T-028 (must be ported) |
| **Viscosity** is the only dissipative term. Without it the fluid sloshes forever. | `ComputeViscosityForce()` | T-028 (must be ported to HLSL) |
| **Pressure clamping and the near channel are mutually exclusive.** The near pressure has no rest-density offset, so it is a permanent outward push that only a *negative* main pressure can balance. Clamping forbids negative pressure, so both together mean the fluid expands without limit — which presents as particles flying apart, not as anything clamp-shaped. `SPH2D` now warns on this combination. | `WarnIfClampConflictsWithNearChannel()` | T-027 (both channels must be ported together) |
| **Two pressure channels: main (SpikyPow2, linear gradient, can go negative) and near (SpikyPow3, quadratic gradient, never negative).** The gradients must differ in shape or the near channel cannot suppress short-range pairing. The fluid settles ~1% *below* `restDensity` by design. | `ComputeDensity()`, `ComputePressureForce()` | T-027, T-028 |
| **Every force term carries `particleMass`.** Density is mass-scaled, so a force without it makes acceleration proportional to `1/mass` instead of mass-independent. This project's mass is ~240, so the error is large and silent. | `ComputePressureForce()`, `ComputeViscosityForce()` | T-028 (must carry through to HLSL) |
| **Viscosity is XSPH, not Müller's Laplacian.** Poly6 is used as a non-negative weight, so the blend cannot add energy. | `ComputeViscosityForce()` | T-028 (must be ported) |
| **Adaptive CFL step size** derived from `c_s = √stiffness`, max velocity and max acceleration. | `ComputeStableTimeStep()` | T-029 (conflicts with "zero CPU readback") |
| **Auto-calibrated particle mass** so mean spawn density matches `restDensity`. It measures whichever density kernel is in use, so it absorbs kernel changes automatically. | `CalibrateParticleMass()` | Any phase that changes spawn or resolution |
| **Density uses SpikyPow2, not Poly6.** Poly6 was retired from density when the second pressure channel was added, and now serves as the viscosity weight instead. | `ComputeDensity()`, `SPHMath` | T-027 |
| **Smoothing length derived from spacing** (`h = 2.2 × spacing`), not hand-set. | `SpawnParticles()` | Any phase that changes particle count |

**Rule:** if a later task must change one of these, record the decision and the reason in that task. Do not let it change by omission.

---

## Phase 2: CPU Optimization (T-023 to T-025)

Your brute-force solver works. Now make it faster while keeping the same behavior.

> **"The same behavior" is load-bearing.** Phase 2 is a pure performance refactor. In particular, boundary particles must remain part of the neighbour structure — see the Phase 1 Invariants above.

### Phase Summary
Replace the O(N²) neighbor search with an O(N) spatial grid. Use Unity's Burst compiler and Job System to parallelize the computation.

- [ ] T-023 — Spatial Grid Neighbor Search
- [ ] T-024 — Unity Burst Compiler Integration
- [ ] T-025 — Unity Job System Parallelization

Task Code: T-023
* Quest : Replace brute-force neighbor search with a uniform spatial grid.
* Guide : Divide the simulation domain into cells of size h (the smoothing radius). For each particle, compute its cell coordinate (floor(x/h), floor(y/h)). Only check neighbors in the 9 surrounding cells. Study spatial grid theory: why it's O(N), how cell size relates to h, and what happens at cell boundaries.
* Note : A uniform grid and spatial **hashing** are not the same thing. This task uses a uniform grid with **direct index arithmetic**, because the domain is a bounded box: no collisions, O(1) lookup. Hashing exists to represent *unbounded* domains and pays for that with collisions — the prime-multiplier hash belongs in T-030, where the GPU path does need it. Measured on a bounded domain, spatial hashing was the slowest of five methods tested (Ihmsen et al. 2011, Table 2).
* Note : Cell size must be exactly h. Measured (Ihmsen et al. 2011, Table 4): halving the cell size to 0.5h tests fewer pairs (15.5M vs 25M) but takes *longer* (39.6ms vs 26.6ms), because more cells means more memory lookups. Smaller is not better.
* Constraint : The grid must index **boundary particles as well as fluid particles**. If it only indexes fluid particles, wall behaviour regresses *here*, silently breaking Phase 2's "same behavior" promise. Boundary particles are static, so their cell membership never changes and can be computed once at spawn — decide up front whether they share the fluid grid or get their own static grid.
* Done : Neighbor search is O(N) instead of O(N²). You can increase particle count to 500+ and still run at 60 FPS. Wall behaviour is unchanged from T-022.

Task Code: T-024
* Quest : Integrate Unity's Burst compiler to speed up the CPU SPH loop.
* Guide : Identify the hot loops in your SPH code (neighbor search, density/pressure, force computation). Annotate them with [BurstCompile]. Move the math into a separate static class or method that Burst can compile. Study Burst's constraints: no managed objects, no boxing, use NativeArray instead of List<T>. Test with the Burst Inspector to see speedups.
* Done : The SPH loop runs 5-10x faster with Burst enabled. You can profile the difference in the Profiler window.

Task Code: T-025
* Quest : Parallelize the SPH computation using Unity's C# Job System.
* Guide : Refactor per-particle computations (density, pressure, force accumulation) into IJobParallelFor jobs. Use NativeArray for all particle data. Schedule jobs with proper dependencies. Study the Job System's safety system ([NativeDisableParallelForRestriction], NativeMultiHashMap for neighbor storage). Compare single-threaded vs. parallel performance.
* Done : The SPH solver uses all available CPU cores. Particle count can reach 5,000+ at interactive frame rates.

---

## Phase 3: GPU Acceleration (T-026 to T-029)

The CPU solver is fast enough for moderate particle counts. Now move the heavy math to the GPU.

### Phase Summary
Use Unity Compute Shaders to run SPH kernels on the GPU. This works on macOS via the Metal backend.

**Decide before starting.** The CPU solver depends on boundary particles (T-020) and an adaptive CFL step size (T-021). Neither appears in the tasks below, so both will be dropped by default unless handled deliberately. See **GPU Port Decisions** at the end of this phase.

- [ ] T-026 — Compute Shader Setup & Data Flow
- [ ] T-027 — GPU Density & Pressure Kernels
- [ ] T-028 — GPU Force Computation & Integration
- [ ] T-029 — CPU-GPU Pipeline Integration

Task Code: T-026
* Quest : Set up a Compute Shader pipeline. Create SPH2D.compute, define GPU buffers for particle data, and dispatch a simple kernel from C#.
* Guide : Study Unity's Compute Shader introduction. Create a compute shader with a single kernel that reads and writes particle positions. In C#, allocate ComputeBuffer objects, set them via SetBuffer(), and dispatch with Dispatch(). Verify data round-trips correctly between CPU and GPU.
* Constraint : Particle buffers must be able to hold boundary particles too. They are static — uploaded once, never updated, never integrated — and `boundaryVolume` is a single scalar. So this costs one buffer and one shader constant, not a whole new subsystem.
* Done : You can execute a compute shader from C# and read back modified particle data. The basic CPU-GPU communication pipeline works.

Task Code: T-027
* Quest : Implement density and pressure computation entirely in the compute shader.
* Guide : Write HLSL functions for the 2D Poly6 kernel and Tait EOS. Implement a kernel that loops over all particles and computes density + pressure for each. Store results in RWStructuredBuffer<float> for densities and pressures. Study HLSL syntax differences from C# (float2, [numthreads], thread IDs).
* Constraint : The density kernel must include the boundary contribution `restDensity * boundaryVolume * W`, exactly as `ComputeDensity()` does on the CPU. Without it, density near walls is under-estimated and the T-020 wall-welding bug returns on the GPU. Also port the pressure clamp (`p = max(0, p)`) — it is deliberate, not a leftover.
* Done : Density and pressure are computed on the GPU. Values match your CPU implementation for the same input data.

Task Code: T-028
* Quest : Implement pressure and viscosity force computation in the compute shader.
* Guide : Write HLSL functions for the Spiky gradient and Viscosity Laplacian. Create a force computation kernel that reads positions, densities, and pressures, then writes force vectors to a RWStructuredBuffer<float2>. Study how to structure multi-pass compute shader pipelines.
* Constraint : Two things must be carried over. (1) The boundary push via pressure mirroring — on the CPU this collapses to `V_b * p_i * |gradW_spiky|` with the direction pointing inward. (2) The viscosity term — it is the only dissipative force in the solver, so omitting it produces a fluid that never settles.
* Done : All SPH forces are computed on the GPU. The visual result matches the CPU solver.

Task Code: T-029
* Quest : Integrate the GPU compute pipeline with your C# simulation loop with zero CPU readback during simulation.
* Guide : Keep particle state entirely on the GPU between frames. Only read back data for visualization if needed. Use ComputeBuffer.CopyCount() or similar for metadata. Study the trade-offs of keeping data on GPU vs. CPU. On macOS, verify the Metal backend compiles and runs your compute shaders.
* Constraint : "Zero CPU readback" conflicts with T-021's adaptive step size, which needs `max‖v‖` and `max‖a‖` from the GPU. Choose deliberately: **(a)** GPU parallel reduction plus one tiny async readback consumed a frame later (no pipeline stall), **(b)** keep `dt` GPU-resident so the CPU never sees it, or **(c)** fall back to a fixed conservative `dt` and accept the lost adaptivity. Do not let the step silently become fixed.
* Done : The full SPH simulation runs on GPU. CPU only handles dispatch and rendering. You can reach 10,000+ particles at interactive frame rates.

---

### GPU Port Decisions

Record the outcome of each decision as you make it. The point is that these are **choices**, not accidents.

| # | Decision | Options | Chosen | Reason |
|---|---|---|---|---|
| 1 | Boundary particles in the spatial grid (T-023) | shared grid / separate static grid / exclude | | |
| 2 | Boundary particles on GPU (T-026) | port to a static buffer / drop and rely on the clamp | | |
| 3 | Boundary push on GPU (T-028) | pressure mirroring / penalty force / none | | |
| 4 | Adaptive step under zero readback (T-029) | async readback / GPU-resident dt / fixed dt | | |

**Recommended baseline:** port the boundary particles — they are static, so they are cheap — and keep the step adaptive via a GPU reduction. Dropping boundary particles does not merely reduce fidelity; it reintroduces the exact wall-welding bug that T-020 fixed, in the phase where it is hardest to diagnose.

---

## Phase 4: GPU Spatial Hashing & Rendering (T-030 to T-033)

O(N²) brute-force on GPU still limits particle count. Add spatial hashing for O(N) neighbor queries, then make it look good.

### Phase Summary
Implement GPU spatial hashing for efficient neighbor search. Add instanced rendering so particles look like a fluid, not debug spheres.

- [ ] T-030 — 2D Spatial Hash Key Generation in HLSL
- [ ] T-031 — GPU Count Sort & Prefix Scan
- [ ] T-032 — GPU Instanced Particle Rendering
- [ ] T-033 — Surface Smoothing / Better Visualization

Task Code: T-030
* Quest : Implement 2D spatial hashing in the compute shader to map particle positions to grid cells.
* Guide : In HLSL, compute cell coordinates int2 cell = floor(pos / h). Hash to a 1D key using prime multiplication: uint hash = (cell.x * 73856093u ^ cell.y * 19349663u) % tableSize. Study spatial hashing theory: why prime numbers reduce collisions, how cell size relates to h, and the difference between hashing and a uniform grid.
* Done : Particles are correctly mapped to spatial hash cells. No particles are lost or duplicated in the hash table.

Task Code: T-031
* Quest : Implement GPU count sort and prefix scan to build the spatial grid.
* Guide : Implement a 3-pass GPU sort: (1) clear and count particles per cell using atomics, (2) compute prefix sums to get cell start offsets, (3) scatter particles into sorted arrays. Study Blelloch scan and why prefix sums are needed for O(N) grid construction.
* Done : The spatial grid is built correctly on GPU. Downstream kernels can query _CellStart and _CellEnd for any hash bucket.

Task Code: T-032
* Quest : Render particles as smooth disks using GPU instancing instead of Gizmos.
* Guide : Create a shader that renders quad instances at particle positions. Pass the GPU position buffer to the vertex shader. Scale each instance by particle radius. Study Graphics.RenderPrimitivesInstanced or DrawMeshInstancedIndirect. On macOS, verify Metal instancing works correctly.
* Done : Particles render as smooth filled circles. The simulation looks like a fluid body, not debug wireframes. 50,000+ particles render at interactive frame rates.

Task Code: T-033
* Quest : Improve fluid visualization with color mapping and screen-space smoothing.
* Guide : Color particles based on velocity magnitude or pressure in the fragment shader. Add a simple screen-space post-process to smooth particle edges. Study how screen-space fluid rendering works: depth buffer, bilateral filtering, and normal reconstruction.
* Done : Fluid has visible color variation (e.g., fast = red, slow = blue). The surface looks continuous rather than like individual dots.

---

## Phase 5: 3D Conversion (T-034 to T-037)

Do not start this phase until all 2D phases (1–4) are complete and validated. This is a separate milestone: port the working 2D solver to 3D.

### Phase Summary
Take everything you built in 2D and extend it to 3D. The math is similar but the dimensional constants change. Treat this like building a new project based on your 2D experience.

- [ ] T-034 — 3D Data Layout & Container
- [ ] T-035 — 3D SPH Kernels & Compute Shader
- [ ] T-036 — 3D Spatial Hashing & 27-Cell Neighbor Search
- [ ] T-037 — Interactive Controls & UI

Task Code: T-034
* Quest : Extend all data structures from 2D (float2) to 3D (float3) to prepare for volumetric simulation.
* Guide : Study how 2D SPH maps to 3D: the math stays the same but normalization constants change. Update C# structs, ComputeBuffers, and HLSL types. Change the container from a 2D box to a 3D cube. Study the dimensional analysis of SPH kernels.
* Done : All data structures support 3D coordinates. The container is a 3D volume. Particle spawning works in 3D.

Task Code: T-035
* Quest : Port SPH kernels to 3D and validate against 2D behavior.
* Guide : Update Poly6, Spiky, and Viscosity kernels to 3D formulas. Create SPH3D.compute or parameterize the existing shader for 2D/3D. Study how kernel normalization changes with dimension (2D uses area, 3D uses volume).
* Done : 3D SPH produces correct fluid behavior. Particles fall, stack, and flow in 3D space.

Task Code: T-036
* Quest : Implement 3D spatial hashing with 27-cell neighbor queries.
* Guide : Extend the 2D spatial hash to 3D. Hash int3 cell coordinates. Query all 27 neighboring cells instead of 9. Study how the prime hash function extends to 3D and why 27 cells cover the full neighborhood in 3D.
* Done : 3D neighbor search is O(N). The 3D simulation runs at interactive frame rates with 50,000+ particles.

Task Code: T-037
* Quest : Add user controls for simulation parameters and runtime interaction.
* Guide : Create editor UI with OnValidate() or IMGUI/UIToolkit to adjust parameters at runtime. Add keyboard/mouse controls to spawn particles, reset the simulation, or apply external forces. Study Unity's input system for editor and runtime input.
* Done : You can tweak parameters and see results in real-time. The simulation is interactive and fun to experiment with.

---

## Notes for This Project

- No NVIDIA tools needed. This project uses Unity's cross-platform compute shader pipeline. On macOS, Unity compiles to Metal automatically.
- Start simple. A working O(N²) solver with 100 particles is better than a broken O(N) solver with 10,000 particles.
- Visualize everything. If you can't see it, you can't debug it. Use Gizmos extensively in the early phases.
- One task at a time. Complete each task fully before moving to the next. The phases are deliberately sequential.
- T-001 to T-007 are done. Those were the research and planning tasks. This file starts tracking implementation from T-008 onward.
