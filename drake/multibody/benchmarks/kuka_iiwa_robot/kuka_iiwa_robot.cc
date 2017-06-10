#include "drake/multibody/benchmarks/kuka_iiwa_robot/kuka_iiwa_robot.h"

#include "drake/common/extract_double.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/MGKukaIIwaRobot.h"

namespace drake {
namespace multibody {
namespace benchmarks {

using Eigen::Map;
using Eigen::Matrix;

template <typename T>
std::tuple<Matrix3d, Vector3d, Vector3d, Vector3d>
KukaIIwaRobot<T>::CalcForwardKinematicsEndEffector(
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& v) const {

  static_assert(std::is_convertible<T, double>::value,
                "This class only supports T = double.");

  // Form a continuous state vector of joint angles and their time-derivatives.
  VectorX<double> state(q.size() + v.size());
  state << q, v;

  // Calculate forward kinematics of end-effector with MotionGenesis.
  using MotionGenesis::MGKukaIIwaRobot;
  MGKukaIIwaRobot mgKukaIIwaRobot;
  mgKukaIIwaRobot.SetVariablesFromArray(state.data());
  mgKukaIIwaRobot.CalculateOutput();

  // Convert MotionGenesis standard C++ matrix to an Eigen matrix.
  // Note: Standard C++ stores two-dimensional matrices in row-major form.
  // By default, Eigen stores two-dimensional matrices in column-major form.
  // After an Eigen matrix is constructed from the C++ matrix (interpreting the
  // C++ matrix in row-order), column-major R_NG is populated via operator=.
  const double *matrix33 = mgKukaIIwaRobot.R_NG[0];
  const Matrix3d R_NG = Matrix<double, 3, 3, Eigen::RowMajor>(matrix33);

  const Vector3d p_NoGo_N(mgKukaIIwaRobot.p_NoGo_N);
  const Vector3d w_NG_N(mgKukaIIwaRobot.w_NG_N);
  const Vector3d v_NGo_N(mgKukaIIwaRobot.v_NGo_N);

  return std::make_tuple(R_NG, p_NoGo_N, w_NG_N, v_NGo_N);
}

// Explicitly instantiates on the most common scalar types.
template class KukaIIwaRobot<double>;

}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
