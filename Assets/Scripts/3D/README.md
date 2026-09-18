# 3D scripts

Empty until Phase 5. The 2D solver lives in `../2D/` and is the reference.

TASKS.md is explicit that these are two milestones, not one codebase with a dimensional switch: *"Do not mix them."* Concretely, that means:

- Nothing in `../2D/` should gain a `bool is3D` or a `float3` path. When the 2D solver needs to change, it changes for 2D reasons.
- The 3D port starts as a copy of the 2D files, then diverges. Kernel
  normalisation is the obvious difference: 2D normalises over area (`h^4`, `h^8`)
  and 3D over volume (`h^6`), so every constant in `SPHMath` changes even though
  the shape of every equation stays the same.
- Shared ideas should be copied, not abstracted. A `SPHMathBase` with dimension parameters would make both solvers harder to read than the duplication costs.

Also worth carrying over rather than rediscovering: the phase-1 invariants table
in TASKS.md, the GPU port decisions, and the two traps recorded in T-027 and
T-028 (the gradient functions have no `r >= h` early-out, and the mirrored
boundary pressure is clamped unconditionally).
