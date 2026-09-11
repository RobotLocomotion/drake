# C++ contact-pipeline optimizations: home_office_desk_tidy

Instead of changing the scene or solver configuration, this series
optimizes Drake's collision/contact C++ itself. Physics configuration is
unchanged (10 ms steps, `hydroelastic_with_fallback`, default `lagged`
approximation, stock scene).

All measurements: `harness.py --duration 30` via the bazel target added in
the first commit of this series, single thread, local gcc-13 `-c opt` build,
48-core desktop, 5-run medians (~±0.5 s run noise). Every commit finishes
with the identical final contact set (89 hydroelastic + 5 point-pair).

**Baseline (this machine): 36.6 s wall (0.82x realtime); collision-geometry
query (`ComputeContactSurfacesWithFallback`) ≈ 50% of total.**

**Result: 27.1 s wall (1.11x realtime) — 26% faster end-to-end.**

## Why the scene is expensive

The broadphase produces ~3,760 AABB-overlap candidate pairs per step while
only ~94 pairs are actually in contact. World-aligned AABBs of oriented
convex-decomposition pieces overlap promiscuously (dozens of pieces per
body, bodies stacked and shelved). The wins below are therefore about
(a) making per-candidate rejection radically cheaper and (b) not recomputing
per-tet/per-tri/per-body quantities once per candidate.

## The commit series

Each commit documents its own 5-run-median benchmark numbers; cumulative:

| change | wall (s) | vs prev |
|---|---|---|
| (baseline; benchmark target only) | 36.6 | — |
| compliant-rigid intersection: per-element caches + sphere reject | 35.3 | −3.6% |
| compliant-compliant intersection: per-tet caches + scratch + hash fix | 34.3 | −2.8% |
| CollisionFilter: single-probe blocked set + fast pair hash | 33.3 | −2.9% |
| rigid-rigid point-contact OBB prefilter | 30.1 | −9.6% |
| per-body spatial Jacobians in contact-pair assembly | 27.1 | −10.0% |

Highlights:

- **Rigid-rigid OBB prefilter** (`proximity_engine.cc`): ~2,700 of the
  ~3,760 candidates per step are rigid-rigid pairs (only ~5 in contact);
  each went straight to fcl GJK/MPR. One OBB-OBB test on the pair's
  hydroelastic surface-mesh bounding volumes now proves most disjoint.
  Restricted to fcl `GEOM_BOX`/`GEOM_CONVEX`, whose rigid meshes bound the
  narrowphase shape exactly (tessellated round primitives under-approximate
  and keep the unconditional fcl query).
- **Per-element caching** (`mesh_intersection`, `field_intersection`):
  tet face half-spaces, normalized gradients, transformed vertices, and
  per-tet field values were rebuilt for every (tet, tri) / (tet, tet)
  candidate; now computed once per element per query. Bounding-sphere
  tests reject non-touching candidates before clipping. Scratch buffers
  remove the per-candidate heap allocations, and the BFS visited-set hash
  no longer XORs the two indices (which collides for every pair with equal
  XOR).
- **Contact-pair assembly** (`discrete_update_manager.cc`, the one
  non-geometry change): two full-width (3×nv, nv=144 here) translational
  Jacobians were computed per contact face and immediately sliced to ~6
  tree columns. Each body's 6×nv spatial Jacobian is now computed once per
  step and rigidly shifted to each face centroid on the relevant columns
  only.

## Tried and rejected (measured slower or neutral here)

- **fcl `findExtremeVertex` thread-local visited buffer**: replacing the
  per-call `std::vector<int8_t>` allocation with a thread-local buffer
  measured *slower* (+1.1 s at its stack position, consistently across two
  measurement series) — general-dynamic TLS access in a shared library
  evidently costs more per support call than a tcache-hot small allocation.
- **Hydroelastic-calculator root-OBB culls**: duplicated the root test the
  intersectors' own BVH traversals perform; +0.6 to +1.0 s at its position
  once the other culls were in.
- **BVTT traversal changes** (descend-larger heuristic, single-compose OBB
  tests, thread-local stack, inlined callbacks): −0.2 s at its position —
  within run noise, so not kept in this series despite being mechanically
  sound; worth revisiting on traversal-heavier scenes.

## Numerics

None of the changes alter what is computed, but caching changes *how*
(cached vs recomputed values are identical only in exact arithmetic).
Trajectories therefore differ from baseline at floating-point noise level,
as with any refactor of these code paths. The steady state is unchanged:
same contact counts every run, settled scene at 30 s.

## Leftovers

- The remaining compliant-rigid cost is dominated by BVH descent and
  clipping for *genuinely near* pairs; temporal-coherence caching across
  steps is the next big lever but requires mutable per-pair state in a
  const query path.
- Per-body (rather than per-piece) broadphase grouping would cut the
  ~3,760 candidates/step directly but is an architectural change to
  ProximityEngine.
- SAP solve (~31% of baseline) was intentionally out of scope.
