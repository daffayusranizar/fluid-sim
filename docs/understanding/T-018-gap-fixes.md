# T-018 — Your Specific Gaps, Fixed

Nine targeted fixes with plain English and analogies. Every claim is sourced at the bottom.

---

## G1 — Why must force be reset every step?

Your answer: *"so we don't accumulate force from neighbors that are not relevant anymore."*

That's part of it, but it misses the **real mechanism**. The real reason is arithmetic, not relevance.

### The key fact

`force` is a **running sum**, not a value you overwrite:

```csharp
particles[i].force = Vector2.zero;   // start at 0
for each neighbor j:
    particles[i].force += contribution;   // ADD to it
```

The `+=` is the important part. It adds to whatever is already there.

### Analogy: a bank account you never zero

Imagine a bank account. Every day you deposit that day's income, but you never start a fresh balance — you keep adding to the running total.

- Day 1: balance = 10
- Day 2: balance = 30
- Day 3: balance = 60
- Day 10: balance = 550

The number is meaningless — it's the sum of every day so far, not today's income.

That's what happens if you skip the reset. Frame 1's force stays, then frame 2's force gets added on top, then frame 3's, and so on. After 60 frames the particle thinks it's being pushed 60× harder than it really is.

### What goes wrong

```
frame 1: force = F
frame 2: force = 2F
frame 3: force = 3F
...
frame 60: force = 60F
```

Acceleration grows every frame → velocity grows even faster → the simulation explodes within a second or two.

### So which is the "right" reason?

| Reason | Correct? |
|---|---|
| Neighbors changed, so old contributions are stale | true, but secondary |
| A sum must start from zero, or it compounds forever | ✅ **the real reason** |

The reset isn't about relevance. It's that you're **recomputing a sum**, and a sum has to start at zero.

---

## G2 — Why does `particleMass` appear?

Your answer: *"we cannot compute force without mass."*

That's circular — it restates the question. Here's what mass actually *does*.

### The key fact: SPH interpolation

Everything in SPH is built on this rule:

```
A(r_i) ≈ Σ_j  m_j · (A_j / ρ_j) · W(|r_i − r_j|, h)
```

Mass appears as the **weight** in that sum. It answers: *"how much fluid is out there at neighbor j's location?"*

### Analogy: scoop sizes in a soup recipe

Imagine measuring how salty a soup is. You taste contributions from nearby ingredients:

- a big scoop of stock → counts a lot
- a pinch of salt → counts a little

Mass is the **scoop size**. Each particle represents a parcel of fluid, and `m_j` says how big that parcel is. Without it, every particle would count equally — a particle representing 1 kg would count the same as one representing 1000 kg.

### Why the pairing `m_j / ρ_j`

`m_j / ρ_j` = mass ÷ density = **volume**. So each neighbor contributes:

```
(volume of neighbor j) × (its value per unit volume) × (kernel weight)
```

Müller's Eqn (1) states this directly: *"The particle mass and density appear because each particle `i` represents a certain volume `V_i = m_i / ρ_i`."*

### Short version

Mass is the **interpolation weight** — it tells the sum how much matter each neighbor represents.

---

## G3 — What does `(h − r)²` do at `r = h`?

You answered `r = 0` (correct) but skipped `r = h`.

### The table

| Distance | `(h − r)²` | Meaning |
|---|---|---|
| `r = 0` | `h²` — maximum | Strongest push when particles overlap → prevents clumping |
| `r = h` | **`0`** | Force fades smoothly to **zero** at the edge of support |

### Why `r = h → 0` matters

**Analogy:** imagine two drivers on a road. One *gradually* eases off the brake as they approach a junction. The other slams the brake on at the exact junction line.

The first is smooth and stable. The second jolts every car behind, and those jolts add up.

If the force were still non-zero at `r = h`, then the instant a particle crossed the support boundary:

```
frame n:   particle is inside h  → force = some value
frame n+1: particle is outside h → force = 0
```

That's a **sudden jump**. Jumps inject energy into the system, and injected energy shows up as jitter and instability.

### Why SPH kernels are built this way

Müller states the design principle explicitly:

> *"kernels that are zero with vanishing derivatives at the boundary are conducive to stability."*

The Spiky kernel satisfies this: both `W` and its gradient reach zero exactly at `r = h`, smoothly.

### Short version

- `r = 0` non-zero → **prevents overlap**
- `r = h` zero → **prevents jolts** when particles enter/leave the neighbourhood

Both properties matter. You need both.

---

## G4 — The sign (your biggest gap)

You marked two sub-questions as `gap`, and two of your answers landed in the wrong slots. Let's do the whole thing slowly with concrete numbers.

### Setup: two particles

```
particle i  at  (0, 0)
particle j  at  (1, 0)
h = 2, so r = 1
both have positive pressure (they're compressed)
```

**What should happen physically?** They're squeezed together, so they should push each other apart. Particle `i` should fly in the **−x** direction (left, away from `j`).

Let's check whether the formula agrees.

---

### Step 1 — `dW/dr` for Spiky: negative

The Spiky kernel decreases with distance:

```
W(r) = 10/(πh⁵) · (h − r)³
```

As `r` grows, `(h − r)` shrinks, so `W` shrinks. It's a **downhill slope**.

```
dW/dr = −30/(πh⁵) · (h − r)²      ← negative, always
```

**Answer: negative.**

**Analogy:** the kernel is a hill. As you walk outward (increasing `r`), you're always walking downhill. Downhill = negative slope.

---

### Step 2 — Where does `∇W` point?

```
∇W = (dW/dr) · (r_i − r_j)/r
```

Work out each piece with our numbers:

```
r_i − r_j = (0,0) − (1,0) = (−1, 0)      points from j to i (leftward)
dW/dr     = negative
```

Multiply:

```
negative × (pointing left) = (pointing right)
```

So **`∇W` points from `i` toward `j`** (in the +x direction).

**Analogy:** `∇W` points "uphill" — toward the center of the kernel, where `W` is biggest. Particle `j` sits in that direction, so `∇W` points at `j`.

---

### Step 3 — Where does the force point?

Müller's formula (Eqn 10):

```
f_i = −Σ_j m_j · (p_i + p_j)/(2ρ_j) · ∇W
```

Take our numbers. Positive pressure ⇒ `(p_i + p_j)` is positive. Every other factor is also positive. So the sum is **positive**.

Now apply the leading minus:

```
f_i = −(positive) × (∇W, pointing right)
    = (pointing LEFT)
```

**`f_i` points left — away from `j`.** ✅

**Answer: away from `j`. Repulsive.** That matches the physics.

---

### Step 4 — Why does the code look different?

The code is the same algebra, just pre-arranged:

```csharp
Vector2 rVec = particles[i].position - particles[n].position;  // j -> i
Vector2 dir = rVec / r;                                        // unit, points away from j
particles[i].force += dir * contribution;                      // positive sign
```

Remember two facts:

| Fact | Value |
|---|---|
| `(r_i − r_j)` direction | away from `j` |
| `\|dW/dr\|` | positive (we flipped the sign and named it `spikyGradMag`) |

So the code's `dir * contribution` is:

```
(points away from j) × (positive number)  =  points away from j ✅
```

Same answer as Step 3, written more compactly. The code took `dW/dr`'s negative sign and folded it into `∇W`'s direction — two flips that cancel.

**Answer:** because `position[i] − position[n]` already points away from `j`, and `spikyGradMag` is already the positive magnitude. Using `position[n] − position[i]` with a negative sign is the *same thing written differently* — it's not wrong, just more confusing.

---

### Step 5 — What if the sign were flipped?

**Analogy:** a crowd where, the more tightly you pack people, the harder they **hug** each other.

With the sign inverted, positive pressure becomes attraction:

| Density | Correct behaviour | Flipped behaviour |
|---|---|---|
| compressed | push apart → spreads out | **pull together** → collapses |
| sparse | (clamped) nothing | **push apart** → explodes |

So the fluid would do the exact opposite of fluid behaviour:
- Compressed pockets would **suck themselves into dense clumps**
- Sparse regions would **blow apart**
- Nothing would ever look like water

**This is exactly the bug you hit.** Compressed regions were pulling together instead of pushing apart, which is why particles stuck and stacked.

---

## G5 — Why does a one-way force prevent equilibrium?

Your answer mixed in the next question. Here's the clean version.

### What "equilibrium" means

A system is in equilibrium when **the net force is zero at a point where small errors get corrected**.

Think of a marble in a bowl. Nudge it and it rolls back — that's a **stable** equilibrium.

Now think of a marble on a flat floor with a ratchet. You can push it right, but nothing pushes it back left. There's no "resting place" it returns to.

### Your clamped pressure is the ratchet

| Density | Force | Can the error be corrected? |
|---|---|---|
| `ρ > ρ₀` (too dense) | push apart | ✅ yes |
| `ρ < ρ₀` (too sparse) | **nothing** | ❌ no |

### Analogy: a car with only an accelerator

Imagine a car with no brakes:

- You can always go faster
- You can never deliberately slow to a target speed
- You can coast, but "coasting" isn't equilibrium — it's just the absence of force

With clamping, the fluid is exactly this car. It **expands**, pressure drops, and once it's under-dense everywhere there's simply *no force anymore*. Nothing pulled it back. Nothing stopped it at a specific density.

### Contrast: what a real equilibrium looks like

| Situation | Unclamped force | Effect |
|---|---|---|
| too dense | push apart | reduces density toward ρ₀ |
| too sparse | pull together | increases density toward ρ₀ |
| exactly ρ₀ | zero | **rests here** ✅ |

Both errors get corrected, so there's a **basin** the fluid settles into. That's what clamping destroys.

### Short version

**Clamping removes the "brake" side of the force. Without it, errors in one direction can never be corrected, so there is no balance point — only a state where forces happen to be off.**

---

## G6 — What is the trade-off of turning clamping OFF?

You answered the previous question here instead. Here's the actual trade-off.

### What you gain

A true **two-way restoring force**:

```
too dense → push apart
too sparse → pull together
```

Which means the fluid can finally **reach equilibrium**, hold its shape, and behave cohesively (droplets, surface tension, connected streams).

The Ghost SPH authors describe this as *"real liquid behaviour"* — liquid that *"properly coheres"* rather than flying apart.

### What you lose

Negative pressure brings back two documented problems.

**1. Sticking and stringing at surfaces**

At a free surface or near a wall, density is under-estimated (see FR4). Under-estimated density = negative pressure = tension. The surface particles get pulled into each other and form clumps and strings.

This is a **boundary bug revealing itself through the unclamped EOS**. The ghost paper describes it as particles *"unnaturally cluster in a shell around the liquid"*.

**2. Tensile instability**

This is serious enough to have its own paper (Monaghan 2000, *SPH without a Tensile Instability*). The abstract states plainly:

> *"The tensile instability in smoothed particle hydrodynamics results in a clustering of SPH particles. The clustering is particularly noticeable in materials which have an equation of state which can give negative pressures."*

So it's not a bug in your code — it's a known, named, documented property of negative-pressure SPH under tension.

### Summary table

| | Clamped ON | Clamped OFF |
|---|---|---|
| Equilibrium | ❌ none | ✅ exists |
| Cohesion / shape holding | ❌ fluid spreads freely | ✅ holds together |
| Sticking / stringing | ✅ avoided | ❌ can appear |
| Tensile instability | ✅ avoided | ❌ possible |
| Stability | ✅ robust | ⚠️ needs care |

### The middle path

The Ghost SPH paper notes a pragmatic compromise — when resolution near boundaries is too coarse, *"reverting to freely separating boundary conditions by disallowing negative pressures may provide more plausible results."*

In other words: **clamping is a legitimate engineering choice when you can't afford proper boundary handling.** It trades physical correctness for stability.

---

## G7 — Why divide force by density to get acceleration?

Your answer described what `force` *is* (a force density) but didn't finish the sentence.

### The rule

Müller Eqn (8):

```
a_i = f_i / ρ_i
```

Divide a force density by density → **acceleration**.

### Why

**Analogy:** pushing a shopping trolley and a loaded truck with the same force. The trolley shoots forward; the truck barely moves. Same push, very different acceleration — because mass differs.

Density is how much "stuff" is packed into a region. It plays the mass role here. A denser region is harder to accelerate.

### The units, explicitly

| Quantity | Unit |
|---|---|
| accumulated `force` (force density) | N/m³ |
| `density` | kg/m² |
| `force / density` | (N/m³) ÷ (kg/m²) = **m/s²** ✅ |

That's acceleration — exactly what the next line needs:

```csharp
particles[i].velocity += acceleration * dt;
```

### Why `density` and not `particleMass`

Because the accumulated quantity is a **force density**, not a force. Müller's discretisation gives force density, so density is the correct divisor. If you had accumulated a plain force instead, you'd divide by mass. The two must match.

### Short version

**`force` here is per-unit-volume, so dividing by density converts it to per-unit-mass — which is acceleration. Units: m/s².**

---

## G8 — Why does an isolated particle feel no pressure force?

Your answer: *"because it doesn't have pressure."*

That's **wrong**. An isolated particle absolutely *has* a pressure value:

```
density = 0  →  p = k(0 − ρ₀) = −k·ρ₀     (a large negative number, or 0 when clamped)
```

The value exists. The reason there's no force is **structural**, not numerical.

### The real reason

Look at where the force comes from:

```csharp
for (int j = 0; j < neighbors[i].Count; j++)
{
    // ...accumulate force from neighbor j
}
```

The force lives **entirely inside the neighbour loop**. If `neighbors[i]` is empty, the loop body never runs once, and `force` stays exactly `Vector2.zero`.

### Analogy: being crowded

A person alone in the middle of a field cannot be "crowded". Crowding isn't a property of that person — it's a **relationship** between them and other people. No one nearby means the relationship doesn't exist, so there's nothing to measure.

Same with pressure force:
- Pressure is a **property** ("how squeezed am I?") — a lone particle can have this
- Force is a **relationship** ("who is pushing me?") — a lone particle has no one to push against

### Analogy 2: pushing a door

Having "pressure" is like having strong arms. To actually exert force you still need a **door to push against**. A particle with high pressure and zero neighbours is a strong person in an empty room — nothing to push.

### Summary

| | Correct? |
|---|---|
| "It has no pressure" | ❌ it does have pressure |
| "It has no neighbours, and the force is computed entirely from neighbours" | ✅ this is the reason |

---

## G9 — Which equilibrium prerequisites does the code satisfy?

You correctly named all three ✅. What was missing is the verdict: **the code satisfies none of them yet.**

### The three, and the status

| # | Prerequisite | What it does | Status | Task |
|---|---|---|---|---|
| 1 | Two-way restoring force | push when dense, pull when sparse → creates a balance point | ❌ clamped to one-way | — |
| 2 | Dissipation (viscosity) | drains kinetic energy so oscillation dies out | ❌ force is conservative | **T-019** |
| 3 | Correct density at boundaries | so `ρ₀` is meaningful everywhere | ❌ kernel support truncated | **T-020** |

### Analogy: why you need all three

Think of a car that must come to a stop in a parking space:

| Component | Role | Without it |
|---|---|---|
| **Brakes and accelerator** | two-way control | you can only speed up, never settle at a speed → prerequisite 1 |
| **Suspension / shock absorbers** | remove bounce | the car bounces forever instead of settling → prerequisite 2 |
| **Flat, level ground** | a meaningful reference | the "parking space" is on a slope and shifts → prerequisite 3 |

You have a car with an accelerator, no shock absorbers, parked on a slope. It will not settle — and none of that is a problem with the *engine* (your pressure force, which is now correct).

### Why this matters for how you judge your own work

Your pressure force is **correct**. The fluid not settling is **not evidence of a bug in T-018**. Three separate pieces are missing, and two of them are the very next tasks in your list:

```
T-018 pressure force        ✅ done and correct
T-019 viscosity             ← provides dissipation (prerequisite 2)
T-020 boundary handling     ← provides correct boundary density (prerequisite 3)
```

Prerequisite 1 (two-way force) is a *choice* — the `clampPressurePositive` flag — and it becomes viable once T-020 fixes the boundary density.

---

# Sources

1. **Müller, Charypar & Gross (2003)** — *Particle-Based Fluid Simulation for Interactive Applications*, SIGGRAPH/Eurographics SCA 2003.
   - Eqn (1) — density/interpolation; particle volume `V_i = m_i/ρ_i` (G2)
   - Eqn (8) — `a_i = f_i/ρ_i` (G7)
   - Eqn (9) — asymmetric pressure force (G4)
   - Eqn (10) — symmetrized force with `(p_i + p_j)/2` (G4)
   - Eqn (21) — Spiky kernel (G3, G4)
   - Explicit warning on Poly6 clustering and on kernels with vanishing derivatives at the boundary
   - https://matthias-research.github.io/pages/publications/sca03.pdf

2. **Schechter & Bridson (2012)** — *Ghost SPH for Animating Water*, SIGGRAPH 2012.
   - Free-surface and solid-boundary density deficiency (FR4)
   - Unnatural clustering of particles in a shell at the surface (G6)
   - The pragmatic note about disallowing negative pressures when resolution is low (G6)
   - *"Some form of artificial viscosity is necessary to stabilize the inviscid equations"* (G9)
   - https://www.cs.ubc.ca/~rbridson/docs/schechter-siggraph2012-ghostsph.pdf

3. **Monaghan (2000)** — *SPH without a Tensile Instability*, Journal of Computational Physics.
   - Clustering caused specifically by an equation of state that permits negative pressure (G6)
   - https://dl.acm.org/doi/10.1006/jcph.2000.6439

4. **Schuermann — `mueller-sph`** — reference 2D implementation; the source of the widely-copied `SPIKY_GRAD` constant and sign layout (G4).
   - https://github.com/lucas-schuermann/mueller-sph

5. **Illinois CS418** — *Smoothed Particle Hydrodynamics* notes.
   - 2D normalisation constants; `∇W_spiky` coefficient in 2D (G3, G4)
   - https://courses.grainger.illinois.edu/CS418/fa2025/text/sph.html
