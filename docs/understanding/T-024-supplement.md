# T-024 Supplement — Borrowed Mechanisms (Double Density Relaxation and XSPH)

This is not a TASKS.md task. It records work done mid-Phase-2 after comparing against
a reference implementation, and it revises concepts from T-016 through T-019. Those
four notes each carry a "Later amendment" section; this is the understanding check for
the new mechanisms.

---

## Snippet 1 — Two pressure channels

```csharp
// density pass
density     += p.particleMass * SPHMath.SpikyPow2Kernel(r, h, p.spikyPow2Const);
nearDensity += p.particleMass * SPHMath.SpikyPow3Kernel(r, h, p.spikyPow3Const);
```

```csharp
// pressure pass
particle.pressure     = SPHMath.Pressure(density, restDensity, stiffness, clamp);
particle.nearPressure = SPHMath.NearPressure(nearDensity, nearPressureMultiplier);
```

```csharp
[BurstCompile]
public static float NearPressure(float nearDensity, float multiplier)
{
    return multiplier * nearDensity;
}
```

**Explain:**
- `NearPressure` has **no rest-density offset**, unlike `Pressure`. What does that make
  true about its sign, always? Why is that the entire point of the channel?
- What would break if `NearPressure` were written as `multiplier * (nearDensity - restDensity)`
  instead?
- Why are there two density estimates rather than one? What does a single estimate
  force you to choose between?
- Why is there only **one** boundary-particle contribution, and to the main density?
  What would adding boundary particles to `nearDensity` do to wall behaviour?

---

## Snippet 2 — The two gradients must differ

```csharp
force += dir * (p.particleMass * SPHMath.SpikyPow2GradientMagnitude(r, h, p.spikyPow2GradConst)
                * sharedPressure / other.density);

force += dir * (p.particleMass * SPHMath.SpikyPow3GradientMagnitude(r, h, p.spikyPow3GradConst)
                * sharedNearPressure / other.nearDensity);
```

| `r` | near / main |
|---|---|
| `0.00 h` | 2.50 |
| `0.50 h` | 1.25 |
| `0.75 h` | 0.63 |
| `1.00 h` | 0.00 |

**Explain:**
- The main gradient is linear in `(h − r)` and the near gradient is quadratic. Show why
  that ordering is required, not merely convenient.
- What would happen if **both** channels used the quadratic gradient — the same shape,
  differing only by a constant multiplier? Would the near channel still prevent pairing?
- Why must the near/main ratio fall to zero at `r = h`, rather than just becoming small?
- Which kernel feeds which gradient, and why does that pairing have to match? (The
  answer is the reason the density kernel changed from Poly6.)

---

## Snippet 3 — Why clamping and the near channel are mutually exclusive

```csharp
if (clampPressurePositive && nearPressureMultiplier > 0f)
{
    Debug.LogWarning("... the fluid will expand without limit ...");
}
```

**Explain:**
- Trace the chain: near pressure is always positive → main pressure must be able to go
  negative → clamping forbids that → what happens to the fluid?
- The fluid worked fine for several tasks **with clamping on**. Name what changed that
  made clamping stop being harmless.
- The fluid now settles about 1% **below** `restDensity`. Explain why that is the
  designed equilibrium and not an error. Which channel pushes it there, and which
  holds it?
- Why is the failure mode of this conflict so hard to read from the symptom? What does
  "particles flying apart" suggest, and why is that misleading?

---

## Snippet 4 — XSPH viscosity

```csharp
float r2 = math.distancesq(other.position, posI);
viscForce += (other.velocity - velI) * SPHMath.Poly6Kernel(r2, p.smoothingLengthSq, p.poly6Const);
```

**Explain:**
- Müller needed a *dedicated kernel* to guarantee his viscosity never added energy. A
  weight needs no such thing. State the property that makes the difference, and why it
  is automatic for a weight.
- What three things did this replacement remove from the hot loop?
- Why is it no longer necessary to compute a square root per neighbour?
- The original T-019 note argued the direction "lives in `(v_j − v_i)`, so there is no
  unit vector to get backwards". Does that argument still hold? Why is it *more* true now?

---

## Snippet 5 — Every force term carries `particleMass`

```csharp
force += dir * (p.particleMass
                * SPHMath.SpikyPow2GradientMagnitude(r, h, p.spikyPow2GradConst)
                * sharedPressure / other.density);
```

**Explain:**
- Density is mass-scaled (`density += particleMass * W`). Given `p = k(ρ − ρ₀)`, show
  that omitting `particleMass` from the force makes the resulting **acceleration**
  proportional to `1/mass` instead of being mass-independent.
- At this project's settings `particleMass ≈ 240`. What does that mean for how wrong the
  fluid behaves if the term is omitted?
- Why does the same omission cause *no* problem in the reference implementation this came
  from? What is its effective mass?
- State the general rule this gives for copying a force expression between two SPH
  implementations.

---

## Snippet 6 — Scale transfer

The constants were taken from an implementation whose particle mass and density are both
of order 1. This project's are roughly 240 and 1000.

**Explain:**
- Before reusing a pressure or force constant from another solver, what has to be checked?
- The near channel initially appeared to do nothing at all. What was the cause, and how
  would a reader have detected it *without* running the simulation?
- The same mistake appeared twice in this work — once as a missing mass factor and once
  as a magnitude that was orders of magnitude out. What single piece of information would
  have caught both?

---

## Final reflection

1. State the two prerequisites for the near channel to work. What happens if either is
   missing?
2. The scheme was adopted wholesale rather than partially, which was deliberate. Why is
   taking half of a two-channel pressure model more dangerous than taking half of a
   single-channel one?
3. This work revised four earlier tasks. Which parts of T-016/T-017/T-018/T-019 survived
   unchanged, and what does that say about which concepts were load-bearing?
4. The symptom was "particles flying apart", which points at forces. The cause was a
   scalar flag interacting with a pressure offset. What does that suggest about how to
   debug a simulation where the symptom and the cause live at different levels?
