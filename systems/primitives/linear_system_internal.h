#pragma once

#include <optional>

#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {
namespace internal {

/* Returns the controllability matrix:  R = [B, AB, ..., A^{n-1}B].
@pre The coefficient matrices must obey the following dimensions:
 | Matrix  | Num Rows    | Num Columns |
 | A       | num states  | num states  |
 | B       | num states  | num inputs  | */
Eigen::MatrixXd ControllabilityMatrix(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B);

/* Returns true iff the controllability matrix is full row rank.
@pre The coefficient matrices must obey the following dimensions:
 | Matrix  | Num Rows    | Num Columns |
 | A       | num states  | num states  |
 | B       | num states  | num inputs  | */
bool IsControllable(const Eigen::Ref<const Eigen::MatrixXd>& A,
                    const Eigen::Ref<const Eigen::MatrixXd>& B,
                    std::optional<double> threshold);

/* Returns the observability matrix: O = [ C; CA; ...; CA^{n-1} ].
@pre The coefficient matrices must obey the following dimensions:
 | Matrix  | Num Rows    | Num Columns |
 | A       | num states  | num states  |
 | C       | num outputs | num states  | */
Eigen::MatrixXd ObservabilityMatrix(const Eigen::Ref<const Eigen::MatrixXd>& A,
                                    const Eigen::Ref<const Eigen::MatrixXd>& C);

/* Returns true iff the observability matrix is full column rank.
@pre The coefficient matrices must obey the following dimensions:
 | Matrix  | Num Rows    | Num Columns |
 | A       | num states  | num states  |
 | C       | num outputs | num states  | */
bool IsObservable(const Eigen::Ref<const Eigen::MatrixXd>& A,
                  const Eigen::Ref<const Eigen::MatrixXd>& C,
                  std::optional<double> threshold);

/* Returns true iff the system is stabilizable.
@pre The coefficient matrices must obey the following dimensions:
 | Matrix  | Num Rows    | Num Columns |
 | A       | num states  | num states  |
 | B       | num states  | num inputs  | */
bool IsStabilizable(const Eigen::Ref<const Eigen::MatrixXd>& A,
                    const Eigen::Ref<const Eigen::MatrixXd>& B,
                    bool continuous_time, std::optional<double> threshold);

/* Returns true iff the system is detectable.
@pre The coefficient matrices must obey the following dimensions:
 | Matrix  | Num Rows    | Num Columns |
 | A       | num states  | num states  |
 | C       | num outputs | num states  | */
bool IsDetectable(const Eigen::Ref<const Eigen::MatrixXd>& A,
                  const Eigen::Ref<const Eigen::MatrixXd>& C,
                  bool continuous_time, std::optional<double> threshold);

}  // namespace internal
}  // namespace systems
}  // namespace drake
