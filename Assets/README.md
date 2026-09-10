# 2D SPH Fluid Simulation

A Unity-based Smoothed Particle Hydrodynamics (SPH) fluid simulation built as a learning project. The goal is to progress from a basic CPU particle prototype to a full GPU-accelerated 3D fluid engine with volumetric rendering.

## Current State

- Basic 2D particle system with gravity and wall collisions
- Gizmo-based visualization in Scene view
- Structured for future SPH solver integration

## How to Run

1. Open the project in Unity
2. Open `Assets/Scenes/SampleScene.unity`
3. Attach `SPH2D.cs` to a GameObject
4. Press **Play**

## Underlying Physics

This project implements **Weakly Compressible SPH (WCSPH)**. The core ideas:

- **Lagrangian discretization:** The fluid is represented by moving particles that carry physical properties.
- **SPH interpolation:** Field values at a particle are approximated by summing contributions from neighbors weighted by a smoothing kernel \\(W(r, h)\\):
  \\[A(\mathbf{r}_i) = \sum_{j} m_j \frac{A_j}{\rho_j} W(\mathbf{r}_i - \mathbf{r}_j, h)\\]
- **Equation of State:** Pressure is derived from local density deviation via the Tait equation:
  \\[P_i = k (\rho_i - \rho_0)\\]
- **Kernels used:**
  - **Poly6** — density estimation
  - **Spiky** — pressure forces (non-vanishing gradient at \\(r \to 0\\))
  - **Viscosity** — velocity smoothing / internal friction

## Architecture Plan

The simulation is being developed in phases:

1. **CPU prototype** — correctness and benchmarking
2. **Burst / Job System** — parallel CPU performance
3. **Compute shaders** — GPU SPH evaluation
4. **Spatial hashing** — \\(O(N)\\) neighbor search
5. **3D extension** — full volumetric simulation
6. **Rendering** — surface and volumetric output

Key technical targets:

- **GPU spatial hashing** with count sort + Blelloch prefix scan for \\(O(N)\\) neighbor queries
- **Structure of Arrays (SoA)** memory layout for GPU cache efficiency
- **Zero CPU readback** during simulation — all physics remain on VRAM

## Tech Stack

- Unity 6 / URP
- C# + Burst Compiler
- HLSL Compute Shaders
- C# Job System

## Status

Active learning project. See `TASKS.md` for the private development roadmap.
