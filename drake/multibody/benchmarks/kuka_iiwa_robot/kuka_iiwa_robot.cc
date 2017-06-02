#include "drake/multibody/benchmarks/kuka_iiwa_robot/kuka_iiwa_robot.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/MGKukaIIwaRobot.h"

#include "drake/common/drake_assert.h"
#include "drake/common/extract_double.h"

namespace drake {
namespace multibody {
namespace benchmarks {

using Eigen::Map;
using Eigen::Matrix;

// Abbrevate function for converting matrices from row-major to column-major.
typedef Matrix<double, 3, 3, Eigen::RowMajor> ColumnMajorVector3d;

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

  // Convert MotionGenesis data to Eigen classes.
  // Note: Standard C/C++ stores two-dimensional matrices in row-major form.
  // However, Eigen stores two-dimensional matrices in column-major form.
  const Matrix3d R_NG = ColumnMajorVector3d(mgKukaIIwaRobot.R_NG[0]);

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
