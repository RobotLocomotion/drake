#pragma once

#include <Eigen/Core>

namespace drake {
namespace systems {
/** Returns true if the linear dynamical system
 ẋ = A*x+B*u (or x[n+1] = A*x[n]+B*u[n])
 is controllable.
 */
bool Controllable(const Eigen::Ref<const Eigen::MatrixXd>& A,
                  const Eigen::Ref<const Eigen::MatrixXd>& B);

/** Returns true if the linear dynamical system
 ẋ = A*x+B*u (or x[n+1] = A*x[n]+B*u[n])
 y = C*x
 is observable.
 */
bool Observable(const Eigen::Ref<const Eigen::MatrixXd>& A,
                const Eigen::Ref<const Eigen::MatrixXd>& C);

/** Returns true if the linear dynamical system
 ẋ = A*x+B*u or (x[n+1] = A*x[n]+B*u[n])
 is stabilizable.

 @param continuous_time True if the system is in continuous time. False if the
 system is in discrete time.
*/
bool Stabilizable(const Eigen::Ref<const Eigen::MatrixXd>& A,
                  const Eigen::Ref<const Eigen::MatrixXd>& B,
                  bool continuous_time);

/** Returns true if the linear dynamical system
 ẋ = A*x+B*u (or x[n+1] = A*x[n]+B*u[n])
 y = C*x
 is detectable.

 @param continuous_time True if the system is in continuous time. False if the
 system is in discrete time.
*/
bool Detectable(const Eigen::Ref<const Eigen::MatrixXd>& A,
                const Eigen::Ref<const Eigen::MatrixXd>& C,
                bool continuous_time);

}  // namespace systems
}  // namespace drake
