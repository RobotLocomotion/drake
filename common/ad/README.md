
Drake's AutoDiff implementation
-------------------------------

This directory contains Drake's replacement of
`Eigen::AutoDiffScalar<Eigen::VectorXd>`.

Users should `#include <drake/common/autodiff.h>` and refer to
`drake::AutoDiffXd` in their code. (The `namespace ad { ... }` is chosen for
it's compact size in debugging traces, but is not otherwise exposed to users.)
