# T-018 — Gap Filler: Pressure Force

Plain-English answers to every template question, with analogies. Sources are listed at the bottom.

---

# Part 1 — Snippet by snippet

## Snippet 1 — Force storage and per-frame reset

### Why is `force` a `Vector2` but `pressure` a `float`?

**Analogy:** pressure is like the reading on a barometer — a single number, no direction. Force is like a shove — it has a size *and* a direction.

- `pressure`: "how squeezed am I?" → one number
- `force`: "which way am I being pushed, and how hard?" → needs x and y

You cannot express "pushed up and to the left" with a single number. So `force` is a vector.

### Why must `force` be reset every step?

**Analogy:** imagine a whiteboard where you tally how much everyone in the room owes you. Each time you redo the tally, you must **wipe the board first**. If you keep adding to yesterday's total, the number grows forever.

The force is an **accumulated sum** over neighbors:

```csharp
particles[i].force = Vector2.zero;      // wipe the board
for each neighbor j:
    particles[i].force += contribution;  // add each neighbor's push
```

Forget the reset and the force compounds every frame. Particles accelerate faster and faster until the simulation explodes.

### Does the reset need its own loop?

No. Resetting inside the same loop is correct because you only ever **read** other particles' `position`, `pressure`, and `density` — never their `force`. You only write `force[i]` while iterating `i`, and you never read `force[j]` for a neighbor `j`. So there is no dependency between iterations and the order does not matter.

---

## Snippet 2 — Per-particle vs pairwise

### Why does `ComputePressure()` have one loop while `ComputePressureForce()` has a nested loop?

**Analogy:** a person's **body temperature** is a property of that person alone — you can take it without asking anyone else. But whether they are **crowded** depends on who else is nearby.

| Quantity | Depends on | Loop |
|---|---|---|
| `pressure[i]` | only `density[i]` | single loop |
| `force[i]` | every neighbor `j` | nested loop |

Pressure is a **local state**: `p = k(ρ − ρ₀)`. Only `ρ_i` is needed, and `ρ_i` was already computed.

Force is an **interaction**: the pressure at `i` alone tells you nothing about which way to push. You must look at each neighbor and ask "how much do you push me?" This is what the nested loop represents — **pairwise interaction**.

---

## Snippet 3 — The pressure force formula

### Why use the kernel **gradient** instead of the pressure value?

**Analogy:** standing on a hill, your altitude (pressure) tells you nothing about which way you will roll. What matters is the **slope** (gradient). Water flows downhill, not "toward low numbers".

Using SPH:

```
∇A(r) = Σ_j m_j (A_j / ρ_j) ∇W(r − r_j, h)
```

The derivative **moves onto the kernel**. That is why forces need `∇W` while density only needs `W`. Müller notes that stability, accuracy, and speed all hinge on the kernel choice for exactly this reason.

### Why divide by `particles[n].density` (the neighbor's density)?

**Analogy:** each particle represents a fixed blob of mass. `m_j / ρ_j` converts that mass into a **volume** — the amount of space particle `j` actually occupies. The pressure force is distributed over that volume.

Using the neighbor's own density means each neighbour contributes in proportion to its own size. Müller's derivation writes it exactly this way (Eqn 10), and the reference 2D implementation also uses `pj.rho`.

### Why does `particleMass` appear?

In SPH, every particle stands in for a small parcel of fluid. The interpolation rule is

```
A(r) ≈ Σ_j m_j (A_j / ρ_j) W(r − r_j, h)
```

`m_j` is the **interpolation weight** — it says how much this neighbor's parcel counts. Without mass, the sum has no notion of "how much fluid is over there".

### Why `(p_i + p_j)` rather than just `p_j`?

This is Müller's symmetrization, and it is a real fix, not a stylistic choice.

**The problem:** the naive SPH form is

```
F_i = −Σ_j m_j (p_j / ρ_j) ∇W
```

Particle `i` uses only `j`'s pressure, and `j` uses only `i`'s. Since `p_i ≠ p_j` in general, the two forces have **different magnitudes**. That violates Newton's third law, and momentum is not conserved — the fluid can spontaneously drift or spin.

**Analogy:** two people pushing each other. If one pushes harder than the other, the pair accelerates from nothing. Momentum conservation requires equal and opposite.

**The fix:** average the two pressures so both particles use the same number:

```
(p_i + p_j) / 2
```

Now the pair forces are equal and opposite. Müller: *"The so computed pressure force is symmetric because it uses the arithmetic mean of the pressures of interacting particles."*

### What is `(h − r)²` at `r = 0` and at `r = h`?

| Distance | `(h − r)²` | What it means |
|---|---|---|
| `r = 0` | `h²` — **maximum** | Exactly when two particles overlap, the push is strongest. This is the anti-clumping property. |
| `r = h` | `0` | The force fades smoothly to zero at the edge of support. No sudden jolt when a particle enters or leaves the neighborhood. |

Both matter:
- `r = 0` non-zero → particles cannot sit on top of each other
- `r = h` zero → no force discontinuity, which would inject energy and destabilise the fluid

The second property is why Müller chose a kernel that is *"zero with vanishing derivatives at the boundary"* — that's "conducive to stability".

---

## Snippet 4 — The sign

### Is `dW/dr` positive or negative for Spiky?

The Spiky kernel is a decreasing function of distance:

```
W_spiky(r) = 10/(πh⁵) · (h − r)³
```

As `r` grows toward `h`, `W` shrinks to zero. So its slope is **negative**:

```
dW/dr = −30/(πh⁵) · (h − r)²      ← always negative
```

### Which way must `f_i` point for positive pressure?

Work through it in steps.

**Step 1 — where does `∇W` point?**

```
∇W = (dW/dr) · (r_i − r_j)/r
```

- `(r_i − r_j)` points **from j to i**
- `dW/dr` is **negative**

Negative × (j→i) = **(i→j)**. So `∇W` points from `i` toward `j`.

**Step 2 — apply the minus sign in Müller's formula**

```
f_i = −Σ m (p_i + p_j)/(2ρ_j) · ∇W
```

With positive pressure the sum is positive, so the leading minus flips the direction:

```
∇W points i→j   ⇒   f_i points j→i
```

Particle `i` is pushed **away from** `j`. **Repulsive.** That is correct physics: squeeze a fluid and it pushes back.

### Why `dir = position[i] - position[n]` with a positive sign?

Because `(r_i − r_j)` **already points away from `j`**. And `|dW/dr| = 30/(πh⁵)(h − r)²` is positive, so the code uses the magnitude and a positive sign:

```csharp
Vector2 dir = rVec / r;          // rVec = i - n, so dir points away from n
particles[i].force += dir * contribution;   // contribution > 0 for positive pressure
```

The code is doing the same algebra as the derivation, just written more compactly.

### What if the sign were flipped?

With the sign flipped, positive pressure would **pull** particles together and negative pressure would push them apart. The fluid would invert its behaviour:

- Compressed regions would collapse further — exactly the clumping you observed
- Sparse regions would blow apart
- The simulation would look like it has negative surface tension

### Why the reference implementation looked correct but wasn't

The reference wrote `-rij.normalized()` where `rij = p_j − p_i`, combined with a **negative** `SPIKY_GRAD` constant. Two negatives cancel, so the force ends up pointing **toward** `j` — attractive.

It appeared to work only because in that demo `REST_DENS = 300` while the *actual* density is around 0.02. So `(p_i + p_j)` was **always hugely negative**, and a third sign flip cancelled everything out into repulsion.

Your auto-calibrated `particleMass` puts pressure near zero with **both** signs, so the inversion became visible — and harmful.

---

## Snippet 5 — Clamping the equation of state

### What behaviour does clamping remove?

Negative pressure. Unclamped:

```
p = k(ρ − ρ₀)
p < 0  whenever  ρ < ρ₀   (any under-dense region)
```

Negative pressure is physically **tension** — it pulls particles together. Clamping forces `p ≥ 0`, so the fluid can only push.

### Why does one-directional force prevent a true equilibrium?

**Analogy:** a ratchet, or a door that only opens one way.

For a system to settle, it must be able to correct *both kinds* of error:

| Situation | Force needed | Clamped? |
|---|---|---|
| too dense | push apart | ✅ yes |
| too sparse | pull together | ❌ removed |

With no pull, an under-dense region never gets corrected. The fluid simply expands until pressure switches off, then coasts. There is no point where "net force = 0" it can rest at — only a point where "all forces happen to be zero and nothing stops the drift".

The Ghost SPH paper describes exactly this failure mode for the free surface: once density dips, *"the equation of state then causes particles to unnaturally cluster in a shell around the liquid, rebalancing density but causing a strong surface-tension-like artifact."*

### What is the trade-off of turning clamping off?

| | Clamped (on) | Unclamped (off) |
|---|---|---|
| Equilibrium | none — only expansion | true restoring force exists |
| Artifacts | no cohesion, surface spreads freely | cohesion, surface tension, **sticking/stringing** |
| Stability | stable | can suffer **tensile instability** |

Negative pressure in SPH is well known to cause the **tensile instability** — particles clumping into pairs and strings. There is a whole paper dedicated to it (Monaghan 2000), which is why many practical implementations clamp or add extra terms.

There is also a documented middle ground: the Ghost SPH authors note that when the boundary layer is under-resolved, *"reverting to freely separating boundary conditions by disallowing negative pressures may provide more plausible results."*

---

## Snippet 6 — Forces become motion

### Why divide `force` by `density` for acceleration? Units?

Müller Eqn (8):

```
a_i = f_i / ρ_i
```

The quantity your code accumulates is a **force density** (force per unit volume), not a plain force. Dividing by density turns it into acceleration.

| Quantity | Units |
|---|---|
| accumulated `force` | force density — N/m³ |
| `density` | kg/m² (2D) |
| `force / density` | m/s² — acceleration ✅ |

**Analogy:** two people can push you with the same force, but the heavier one is harder to move. Same force, different acceleration, because mass differs. Density plays that role here.

### Why add gravity separately instead of accumulating it into `force`?

Gravity is already an **acceleration** (m/s²). To store it in `force` you would first have to multiply by density (`ρ·g`) to convert it into a force density, then divide by density again to get back to acceleration.

Adding it directly to the acceleration skips that round trip. It is mathematically identical but simpler, and it does not require density at all.

### Why is a conservative force a problem?

**Analogy:** a frictionless pendulum. It converts potential energy into kinetic energy and back, forever. It never stops swinging, because nothing removes energy.

The pressure force is `−∇p`, derived from the fluid's internal energy. Like a spring, it **stores and releases** energy but never destroys it. So a fluid driven only by pressure will slosh forever.

To settle, the system needs something that **turns kinetic energy into heat**. In SPH, that is **viscosity** — which is exactly T-019.

The Ghost SPH paper states it plainly: *"Some form of artificial viscosity is necessary to stabilize the inviscid equations."*

---

# Part 2 — Final reflection

## 1. Why is the Spiky gradient non-zero at `r = 0`?

Compare the two kernels' gradients:

| Kernel | `∇W` near `r = 0` | Behaviour when particles overlap |
|---|---|---|
| **Poly6** | `∝ r(h² − r²)²` → **0** | No push at all → particles clump |
| **Spiky** | `∝ (h − r)²` → **h²** | Strong push → particles separate |

**Analogy:** Poly6 is a soft pillow. Squeeze it as hard as you like and, right at the centre, it stops resisting. Spiky is a spike. The closer you get to the tip, the harder it pushes back.

Müller describes the failure precisely:

> *"If this kernel is used for the computation of the pressure forces, particles tend to build clusters under high pressure. As particles get very close to each other, the repulsion force vanishes because the gradient of the kernel approaches zero at the center."*

Desbrun solved it with a kernel that has *"a non vanishing gradient near the center"* — the Spiky kernel.

**In one line:** with Poly6, the exact situation you need the force for (particles touching) is the one situation where the force disappears.

---

## 2. Why does an isolated particle feel no pressure force?

The force is a **sum over neighbors**:

```csharp
for (int j = 0; j < neighbors[i].Count; j++) { ... }
```

No neighbors → the loop body never runs → `force` stays at `Vector2.zero`.

**Analogy:** a single person alone in a field cannot be "crowded". Crowding is a relationship, not a property. No one nearby means no crowding and no push.

Note this is about the *loop*, not about the pressure value. Even if the isolated particle had a large pressure number, there is nobody to push against.

There is also a guard in the code:
```csharp
if (particles[n].density <= 0.0001f) continue;
```
This skips neighbors whose own density is near zero, avoiding a division by zero.

---

## 3. The three prerequisites for a stable equilibrium

| # | Prerequisite | Purpose | Your code |
|---|---|---|---|
| 1 | **Two-way restoring force** | push when dense, pull when sparse → a balance point exists | ❌ clamped to one-way |
| 2 | **Dissipation (viscosity or damping)** | drain kinetic energy so oscillation dies out | ❌ not implemented yet (T-019) |
| 3 | **Correct density near boundaries** | so "rest density" is well-defined everywhere | ❌ boundary deficiency (T-020) |

Your simulation satisfies **none** of them yet. That is not a bug in the pressure force — the force is now correct. These are three *separate* missing pieces, two of which are literally the next two tasks.

**Analogy for each:**

1. **Two-way force** — a car with no brakes, only an accelerator. It can speed up but never hold a steady speed.
2. **Dissipation** — a pendulum in a vacuum. It swings forever because nothing takes energy away.
3. **Boundary density** — taking a headcount while standing in a doorway. Everyone behind you is invisible, so you conclude the room is half empty.

---

## 4. Why is a correct repulsive force not enough at the walls?

Because **density is wrong at the wall**, and the force is computed *from* density.

```
ρ_i = Σ_j m_j W(|r_i − r_j|, h)
```

**Analogy:** imagine you are counting how crowded your street is. You count every house within one block. But if you live at the very edge of town, there are **no houses on one whole side** — so your count says "half as crowded as downtown", purely because you ran out of houses to count.

A particle sitting on the floor has **half its kernel support outside the domain**. There are no particles below the wall, so roughly half the contributions to `ρ_i` are simply missing.

The chain of consequences:

```
half the neighbors missing
   → density badly underestimated
   → ρ < ρ₀
   → p = k(ρ − ρ₀) is negative
   → clamped to 0
   → NO repulsion at all
   → the layers above press down and nothing pushes back
   → particles weld to the floor
```

The Ghost SPH paper describes the same effect at the free surface, where the empty air side of the neighbourhood causes *"a much lower density estimate"*, leading to *"a stiff shell of particles"* and a fluid that *"non-uniformly shrinks"*.

**So the fix cannot live in the force calculation.** The force is already doing the right thing with the density it is given. You have to fix the **density estimate** so it knows about the solid material beyond the wall. That is T-020, and the standard solutions are:

| Approach | How it works |
|---|---|
| **Ghost / boundary particles** | Sample extra particles *inside* the wall, mirrored from the fluid. They appear in the density sum, so the truncated neighbourhood is refilled. |
| **Density renormalization** | Precompute how much of the kernel is cut off by the wall and divide the summed density by that fraction. |
| **Penalty / repulsive wall forces** | Add an explicit force pushing particles away from the wall, and stop relying on pressure alone to do it. |

---

# Part 3 — Sources

1. **Müller, Charypar & Gross (2003)** — *Particle-Based Fluid Simulation for Interactive Applications*, SIGGRAPH/Eurographics SCA.
   - Eqn (8): `a_i = f_i / ρ_i`
   - Eqn (9): naive (asymmetric) pressure force
   - Eqn (10): symmetrized force with `(p_i + p_j)/2`
   - Eqn (21): Spiky kernel, and the explicit warning about Poly6 causing clustering
   - https://matthias-research.github.io/pages/publications/sca03.pdf

2. **Schechter & Bridson (2012)** — *Ghost SPH for Animating Water*, SIGGRAPH.
   - Why summed-mass density fails at boundaries
   - Ghost particles for free surface and solid boundaries
   - "Some form of artificial viscosity is necessary to stabilize the inviscid equations"
   - https://www.cs.ubc.ca/~rbridson/docs/schechter-siggraph2012-ghostsph.pdf

3. **Monaghan (2000)** — *SPH without a Tensile Instability*, Journal of Computational Physics.
   - The clustering artifact caused by negative pressure
   - https://dl.acm.org/doi/10.1006/jcph.2000.6439

4. **Schuermann — `mueller-sph`** — reference 2D implementation and the widely-copied `SPIKY_GRAD`/`POLY6` constants.
   - https://github.com/lucas-schuermann/mueller-sph

5. **Illinois CS418** — *Smoothed Particle Hydrodynamics* notes.
   - The 2D normalisation constants
   - https://courses.grainger.illinois.edu/CS418/fa2025/text/sph.html
