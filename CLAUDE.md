# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Drake ("dragon" in Middle English) is a C++ toolbox for model-based design and
verification for robotics, with Python bindings (`pydrake`). The major
subsystems are dynamical-systems modeling, multibody dynamics, mathematical
optimization, geometry/collision, and motion planning.

## Build & Test (Bazel)

Bazel is the primary build system. CMake is only a thin wrapper that invokes
Bazel for end-user installs — develop with Bazel directly. Bazel is launched
via Bazelisk; the pinned version is in `.bazelversion`.

```bash
bazel build //...                 # Build everything
bazel test //...                  # Build and test everything
bazel build common/...            # Build everything under common/ (recursive)
bazel build //common:polynomial   # Build a single target
bazel test //common:polynomial_test            # Run a single test
```

Target syntax: `//path/to/pkg:target`. `//` is the repo root, `...` recurses,
`:all` is all targets in one package (non-recursive). Relative paths work from
the shell's cwd (e.g. `bazel build ...` inside `common/`).

On Ubuntu the default compiler is GCC; the repo's `.bazelrc` sets
`--config=clang` by default here. OpenMP is on by default (disable with
`--config=without_openmp`).

### Useful test configs and flags

```bash
bazel test --config=debug //common:polynomial_test       # debug build
bazel test --config=asan --config=clang //common:polynomial_test  # AddressSanitizer
bazel test --config=memcheck //common:polynomial_test    # valgrind
bazel test --config=kcov //common:polynomial_test        # coverage (Ubuntu only)

# Filter to specific cases / see live output:
bazel test //multibody/plant:multibody_plant_test \
  --test_output=streamed --nocache_test_results \
  --test_arg=--gtest_filter='*SimpleModelCreation*'

# Python test filtering uses unittest names:
bazel test //bindings/pydrake:py/symbolic_test \
  --test_output=streamed --nocache_test_results \
  --test_arg=TestSymbolicVariable

# Pass args to a runnable target after a bare `--`:
bazel run //examples/acrobot:run_passive -- --help
```

### Proprietary solvers

Gurobi/MOSEK/SNOPT are opt-in behind flags, e.g.
`bazel test --@drake//tools/flags:with_mosek=True //solvers:mosek_solver_test`.
Gurobi and SNOPT require local install + license env vars; MOSEK is downloaded
automatically.

## Linting & Formatting

Lint runs automatically as part of `bazel test` (results cached per file). To
run only lint:

```bash
bazel test --config lint //...        # style checks only, no build/test
bazel test --config lint //common/...
```

Auto-formatters (each rewrites files in place):

```bash
bazel run //tools/lint:clang-format -- -i -style=file path/to/file.cc
bazel run -- //tools/lint:ruff format path/to/file.py     # Python (ruff)
bazel-bin/tools/lint/buildifier common/BUILD              # one BUILD file
bazel-bin/tools/lint/buildifier --all                     # all Bazel files
```

C++ follows the Drake C++ style guide (a fork of Google's); Python follows PEP 8
with an 80-char limit (not 79) and uses `ruff` for both check and format. Suppress
with `// NOLINT(category)` (cpplint) or `# noqa` (ruff). Add `tags = ["nolint"]`
to a BUILD rule to exempt vendored/third-party code.

## BUILD file conventions

Do **not** use raw Bazel rules for Drake code. Use the Drake macro wrappers
loaded from `//tools/skylark`, which apply Drake's compiler flags, lint, and
install logic:

- `drake_cc_library`, `drake_cc_binary`, `drake_cc_googletest`, `drake_cc_test`
- `drake_cc_package_library` — the per-directory aggregate library named after
  the package (e.g. `//common:common`) that bundles its sibling targets
- `drake_py_library`, `drake_py_binary`, `drake_py_unittest`, `drake_py_test`

Every BUILD file ends with `add_lint_tests()` from `//tools/lint:lint.bzl`.

## Architecture & dependency rules

Directories group functional components specifically to avoid circular
dependencies. The dependency ordering is roughly: `common` → `math` →
`solvers` → `systems`/`geometry`/`multibody` → `planning`/`manipulation` →
`examples`. For instance `systems` may depend on `solvers`, so `solvers` must
not depend on `systems`. `examples` is logically after all core libraries, and
`test` code may depend on anything (including `examples`) as long as it's
declared in the BUILD rule.

Key top-level directories:

- `common/` — foundational utilities: scalar types (`AutoDiffXd`, symbolic),
  `Value`/`AbstractValue`, RAII smart pointers, `Polynomial`.
- `math/`, `solvers/` — linear algebra helpers; `MathematicalProgram` and the
  solver wrappers (OSQP, Clarabel, IPOPT, Gurobi, MOSEK, SNOPT, …).
- `systems/` — the **Systems framework**: `System`/`Diagram`/`Context`,
  simulation, the core computational abstraction in Drake.
- `geometry/` — `SceneGraph`, proximity/collision queries, meshes, optimization
  over convex sets (GCS).
- `multibody/` — `MultibodyPlant`, parsing (URDF/SDFormat/MJCF), inverse
  dynamics, contact models.
- `planning/`, `manipulation/`, `perception/`, `visualization/` — higher-level
  capabilities built atop the above.
- `bindings/pydrake/` — pybind11/nanobind Python bindings, mirroring the C++
  module layout.
- `tools/` — Bazel macros (`skylark/`), lint (`lint/`), and the external
  dependency definitions (`workspace/`).
- `examples/`, `tutorials/` — runnable demos and Jupyter tutorials.

Scalar templating: most core classes are templated on a scalar type `T` that
may be `double`, `AutoDiffXd` (forward-mode autodiff), or
`symbolic::Expression`. New templated classes are typically explicitly
instantiated for these "default scalars" (see `common/default_scalars.h`).

## ICF contact solver (current focus)

The active work on this branch targets the **ICF (Irrotational Contact Fields)**
contact solver in `multibody/contact_solvers/icf/`. Read
`multibody/contact_solvers/icf/README.md` first — it has the full math
derivation and component overview.

**What ICF does.** It advances a `MultibodyPlant`'s state ($q$, $v$) by one
step, solving for next-step velocities $v_{n+1}$ as the minimizer of an
*unconstrained convex* cost $\ell(v;q_n,v_n,\delta t) = \tfrac12 v^TAv - r^Tv +
\ell_c(v)$, where $A = M(q_n) + \delta t K$ (mass matrix plus implicit
damping/rotor terms), $r = Av_n - \delta t\,k_0$, and $\ell_c$ is a convex
contact/constraint potential. The optimality condition reproduces the discrete
momentum balance with contact impulses $\gamma(v) $ where $\nabla\ell_c =
-J^T\gamma$. It's solved with Newton iterations + exact line search. See
[Castro et al. 2023](https://arxiv.org/abs/2312.03908) and
[Kurtz & Castro 2025](https://arxiv.org/abs/2511.08771).

**Core components** (all in `namespace drake::multibody::contact_solvers::icf`,
mostly `::internal`):

- `IcfModel` (`icf_model.h`) — the convex problem for a fixed $(q_n, v_n,
  \delta t)$; constant during a solve. Owns the constraint pools and an
  `IcfParameters` (`M0`, `D0`, `v0`, `k0`, `time_step`, …). Evaluates the cost,
  gradient, and Hessian.
- `IcfData` (`icf_data.h`) — all per-iterate state that changes with $v$ during
  the optimization (the current $v$, derived per-constraint quantities,
  scratch). Must be sized to a model via `model.ResizeData(&data)` before
  solving.
- `IcfSolver` (`icf_solver.h`) — runs the Newton loop:
  `SolveWithGuess(model, tolerance, data*)`. Holds `IcfSolverParameters` and
  `IcfSolverStats`. Key tunables: Hessian reuse across iterations/steps
  (`enable_hessian_reuse`, `hessian_reuse_target_iterations`), dense-vs-sparse
  algebra (`use_dense_algebra`, for debugging), and exact line-search settings.
- `IcfBuilder` (`icf_builder.h`) and `IcfExternalSystemsLinearizer` — the
  **only** components that know about `MultibodyPlant`; they construct the model
  from a plant/context. They reach into the plant's private API via
  `multibody/plant/multibody_plant_icf_attorney.h` (the attorney/friend idiom).

**Constraint pools.** Constraints of each type are stored in contiguous "pools"
for cache locality: `PatchConstraintsPool` (contact, grouped into patches — each
patch is two bodies A/B with one or more contact pairs, in world coordinates),
plus `Coupler`, `Gain`, `Limit`, and `Weld` pools. Each lives in `IcfModel`; the
matching `*DataPool` (per-iterate data) lives in `IcfData`. All pools are built
on `EigenPool` (`eigen_pool.h`), which is roughly a `std::vector<MatrixX<T>>`
with contiguous backing storage. When adding a constraint type, expect to touch
both the `*_pool` and `*_data_pool` pair.

**Performance model / invariants to preserve:** allocate once at startup (often
over-estimating sizes); reallocation is allowed but should be rare. Patch
grouping exists to reuse per-patch computation and cut FLOPs, and contact is
expressed directly in world coordinates to avoid intermediate frames. Keep these
properties in mind when modifying the solver.

**Public entry point.** ICF is `internal` and not used directly by end users.
It is driven by the `CenicIntegrator` (`multibody/cenic/cenic_integrator.h`), a
`systems::IntegratorBase` subclass providing variable-step, error-controlled
integration; it owns the `IcfBuilder`, `IcfSolver`, two `IcfModel`s, and an
`IcfData`, and passes an adaptive tolerance into `SolveWithGuess`.

**Build & test.** Targets live under `//multibody/contact_solvers/icf:...`, with
per-component tests (e.g. `icf_solver_test`, `icf_model_test`,
`patch_constraints_pool_test`, `two_spheres_test`). Run e.g.
`bazel test //multibody/contact_solvers/icf/...`. Integration-level behavior is
tested via `//multibody/cenic/...`.

## `dev` and `experimental` directories

Code under a `dev/` or `experimental/` subdirectory is exempt from Drake's
style/test-coverage requirements and from platform review. Stable code must
**not** `#include`/`import` from `dev` or `experimental`. `experimental` code is
installed/packaged (and must carry `experimental` in its C++ namespace / Python
module); `dev` code is neither installed nor packaged and cannot have Python
bindings. Their `BUILD.bazel` must live inside the dev/experimental directory
itself.

## Conventions worth knowing

- **Determinism:** never use `Eigen::Random`, libc `rand`, or other global RNG
  state — including in tests. Use a local generator seeded with a hard-coded
  value.
- **Doxygen comments** (`///` or `/** */`) only on published (public/protected)
  APIs; private and `.cc`-local code uses plain comments.
- **Naming:** `UpperCamelCase` types and (verb) methods; trivial getters/setters
  are `lower_snake_case`; member fields are `lower_snake_case_` with a trailing
  underscore; POD members get `{}` in-class initializers.
- **Pointers:** prefer `unique_ptr` and const-ref; `shared_ptr` usually signals
  unclear ownership.
- **Deprecation over removal** when changing public APIs: use `DRAKE_DEPRECATED`
  (C++) / the pydrake deprecation helpers rather than migrating all callers at
  once.
