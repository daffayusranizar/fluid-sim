# Fluid Sim

A 2D fluid simulation in Unity, built from scratch with **SPH** (smoothed-particle
hydrodynamics). No physics engine, no asset — every force in it comes from the maths
in this repo.

It runs **8,000 particles** at interactive frame rates and you can stir it with the
mouse.

It is written twice on purpose:

- a **CPU solver** — Burst-compiled jobs (`SPH2D`, `SPHJobs`, `SPHGrid`)
- a **GPU solver** — a compute shader (`SPHCompute`, `SPH2D.compute`)

Same physics, two implementations. That is not redundancy: the fastest way to tell a
maths bug from a code bug is to run the same scenario on both and compare.

---

## Run it

1. Open the project in **Unity 6** (developed on `6000.6.0f1`).
2. Open `Assets/Scenes/SampleScene.unity`.
3. Press **Play**.

The scene uses the GPU solver. 8,000 particles spawn as a tall column, then collapse
into the box.

## Controls

| Action | What happens |
|---|---|
| Move the mouse | A disc follows the cursor. Intangible — it does not touch the fluid. |
| **Click** | The disc turns solid and pushes water aside. A poke. |
| **Hold and drag** | The disc scoops. Water is carried along rather than just shoved. |
| **Space** | Freeze and resume. Nothing is lost. |

The disc is drawn in the **Scene view only**, so keep the Scene and Game views both
open if you want to see where it is and what size it is.

## Tuning

All of these are live-editable while playing. **Play-mode edits are discarded when
you stop** — to keep them, use the component's ⋮ menu → *Copy Component*, stop, then
*Paste Component Values*.

`SPH2D`

| Field | Effect |
|---|---|
| `particleCount` | More particles, finer detail, smaller particles. Needs re-entering Play. |
| `smoothingLengthInSpacing` | Support radius in particle spacings — the biggest single lever on behaviour. ~2.2 gives ~15 neighbours. |
| `stiffness` | Resistance to compression. Higher is stiffer and forces a smaller time step. |
| `viscosity` | How thick it is. |
| `maxSubSteps` | Must exceed what one frame needs, or the fluid quietly runs slow. |

`ParticleRenderer2D`

| Field | Effect |
|---|---|
| `colourSource` | `Speed` or `Density`. |
| `particleRadiusInSpacing` | Splat size. Display only. 1.0 leaves gaps between particles; past ~1.13 they overlap and it stops reading as particles. |
| `densityHalfRange` | What density the ends of the colour ramp mean, as a fraction of rest density. |

`SPHInteractor2D`

| Field | Effect |
|---|---|
| `radiusInSpacings` | How big the scoop is. |
| `maxPointerSpeed` | Fastest the disc may move. Raise it to stir harder — and lower it first if the fluid ever goes unstable. |
| `radiusGrowthTime` | How long a tap takes to inflate. Lower is a harder tap. |

## Switching to the CPU solver

Untick `SPHComputeSimulation` **before** pressing Play. Both solvers render through
the same shader.

(Unticking it *during* Play freezes the fluid: the GPU loop stops, but the CPU solver
was already told to stand down at startup.)

## Export a macOS app

In the Editor: **Fluid → Build macOS**. Or headless, with the Editor closed:

```bash
U=/Applications/Unity/Hub/Editor/6000.6.0f1/Unity.app/Contents/MacOS/Unity
"$U" -batchmode -quit -projectPath . -executeMethod FluidBuild.BuildMacOS
```

The bundle lands in `Builds/FluidSim.app`. Apple silicon, windowed.

## What it does not do

- **The screen-space surface does not work.** `FluidSurface.shader` is implemented but
  its blur and composite are wrong, so `useScreenSpaceSurface` is off. You can make the
  fluid *look* continuous by raising `particleRadiusInSpacing` past ~1.13, but that
  trades away the particle look rather than reproducing it.
- **No 3D.** `Assets/Scripts/3D/` holds a README only.

## Where to look next

| | |
|---|---|
| `docs/OVERVIEW.md` | The design, the decisions, and the traps that cost real time. **Start here.** |
| `docs/WALKTHROUGH.md` | The maths, from the kernels upward. |
| `docs/TASKS.md` | Every task with notes on what was measured and what broke. |
| `Assets/Scripts/2D/SPHMath.cs` | The pure maths. Everything else is plumbing around it. |

## Layout

```
Assets/
  Scenes/SampleScene.unity   the only scene
  Scripts/2D/                both solvers
  Shaders/2D/                SPH2D.compute, Particle2D.shader
  Editor/BuildMacOS.cs       the macOS export
  Scripts/3D/                empty; the port has not started
docs/                        design, maths, task log
```

## Requirements

Unity 6 with URP, Burst, Collections, Mathematics, and the Input System package.
Active Input Handling is set to **Input System Package (New)**.
