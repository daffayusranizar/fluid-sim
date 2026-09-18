# 3D shaders

Empty until Phase 5. The 2D shaders live in `../2D/`.

What will need a 3D counterpart rather than a port:

- `SPH2D.compute` -> `SPH3D.compute`: the `Particle` struct gains a third component on position, velocity and force (40 bytes -> 48), the kernels change normalisation, and the neighbour search becomes 27 cells instead of 9.
- `Particle2D.shader`: the spherical impostor stays spherical, but the quad faces the camera instead of lying in the XY plane, so the normal reconstruction and the depth offset change.
- `FluidSurface.shader`: the screen-space composite is dimension-agnostic in principle, but it is currently WIP and unverified in 2D, so do not port it until it works there.
