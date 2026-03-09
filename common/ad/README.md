
Drake's AutoDiff implementation
-------------------------------

This directory is a work-in-progress for Drake's forthcoming replacement of
`Eigen::AutoDiffScalar<Eigen::VectorXd>`.

The `namespace ad { ... }` is chosen for it's compact size in debugging traces,
but is not otherwise exposed to users.

Users should continue to `#include <drake/common/autodiff.h>` and refer to
`drake::AutoDiffXd` in their code.

Once we are confident with our replacement implementation, we will switch that
alias to refer to this new code instead of Eigen's AutoDiffScalar.

In the meantime, you can opt-in using by passing the Bazel command line flag
`--@drake//tools/flags:use_eigen_legacy_autodiff=False`.
