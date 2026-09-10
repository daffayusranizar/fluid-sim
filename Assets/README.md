# 2D SPH Fluid Simulation

A Unity-based Smoothed Particle Hydrodynamics (SPH) fluid simulation built as a step-by-step learning project. The goal is to progress from "I don't know what SPH is" to a working GPU-accelerated fluid simulator, with each phase building on the last.

## Who this is for

- **Platform:** macOS (no NVIDIA-specific tooling)
- **Experience:** Basic C#/C++ syntax familiarity, beginner Unity user, no prior fluid simulation experience
- **Goal:** Deep understanding through implementation, not just following a tutorial

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

The simulation is developed in phases. Each phase has two parts: **study the theory**, then **implement the code**.

1. **Foundation** — Unity setup, math prerequisites, debugging tools
2. **First SPH** — brute-force CPU solver with all core forces
3. **Validation** — benchmark scenarios and visual debugging
4. **CPU Optimization** — spatial hashing, Burst, Jobs
5. **GPU Acceleration** — compute shaders (Metal-compatible)
6. **Rendering** — make it look like fluid, not just dots
7. **Extension** — 3D, interaction, polish

Key technical targets:

- **GPU compute shaders** via Unity's platform-agnostic pipeline (works on macOS Metal)
- **Structure of Arrays (SoA)** memory layout for cache efficiency
- **Spatial hashing** for O(N) neighbor search
- **Zero CPU readback** during simulation — all physics on GPU

## Tech Stack

- Unity 6 / URP
- C# + Burst Compiler
- HLSL Compute Shaders (Metal backend on macOS)
- C# Job System

## Status

Active learning project. See `TASKS.md` for the private development roadmap.
