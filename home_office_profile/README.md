# Home office contact benchmark

Self-contained, Drake-only benchmark of a cluttered home-office scene:
furniture welded to the world and 24 free manipulands settling on a desk
under gravity, ~90 contacts (hydroelastic + point-pair) at steady state.
Contact processing dominates the run time, which makes this a useful
profiling and regression target for Drake's collision/contact pipeline.

## Contents

- `harness.py` — builds the plant (10 ms steps,
  `hydroelastic_with_fallback`, remaining `MultibodyPlantConfig` fields at
  Drake defaults), parses the scene, advances flat-out, prints per-chunk
  realtime rate and final contact counts. Imports only pydrake.
- `packages/` — `scenesmith_scenes` + `scenesmith_models` package subsets
  with `package.xml` files. SDFs carry collision geometry and inertia only
  (no visuals or textures): 443 convex meshes, ~3 MB.

## Run

```bash
bazel build //home_office_profile:harness
bazel-bin/home_office_profile/harness --duration 30
```

or with any installed pydrake: `python3 harness.py --duration 30`.

## Known data quirks

- One scene body's inertia fails Drake's `CouldBePhysicallyValid()`
  triangle inequality (parse-time warning); it does not affect the run.

See [OPTIMIZATIONS.md](OPTIMIZATIONS.md) for measured baselines and the
optimization series this benchmark was built to evaluate.
