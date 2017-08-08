#include "drake/multibody/benchmarks/kuka_iiwa_robot/MG/MG_kuka_iiwa_robot.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {
namespace MG {

using Eigen::Map;
using Eigen::Matrix;

template<typename T>
void MGKukaIIwaRobot<T>::CalcMGOutput(
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& qDt,
    const Eigen::Ref<const VectorX<T>>& qDDt) const {
  // Form a continuous vector of joint angles and their time-derivatives.
  const Eigen::Index size = q.size() + qDt.size() + qDDt.size();
  VectorX<double> joint_angles_and_1st_2nd_derivatives(size);
  joint_angles_and_1st_2nd_derivatives << q, qDt, qDDt;

  // Calculate MotionGenesis output quantities.
  MG_kuka_auto_generated.SetVariablesFromArray(
        joint_angles_and_1st_2nd_derivatives.data());
  MG_kuka_auto_generated.CalculateOutput();
}


template<typename T>
std::tuple<Matrix3d, Vector3d, Vector3d, Vector3d, Vector3d, Vector3d>
MGKukaIIwaRobot<T>::CalcEndEffectorKinematics(
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& qDt,
    const Eigen::Ref<const VectorX<T>>& qDDt) const {
  // Calculate forward kinematics of end-effector with MotionGenesis.
  CalcMGOutput(q, qDt, qDDt);

  // Convert MotionGenesis standard C++ matrix to an Eigen matrix.
  // Note: Standard C++ stores two-dimensional matrices in row-major form.
  // By default, Eigen stores two-dimensional matrices in column-major form.
  // After an Eigen matrix is constructed from the C++ matrix (interpreting the
  // C++ matrix in row-order), column-major R_NG is populated via operator=.
  const double *matrix33 = MG_kuka_auto_generated.R_NG[0];
  const Matrix3d R_NG = Matrix<double, 3, 3, Eigen::RowMajor>(matrix33);

  const Vector3d p_NoGo_N(MG_kuka_auto_generated.p_NoGo_N);
  const Vector3d w_NG_N(MG_kuka_auto_generated.w_NG_N);
  const Vector3d v_NGo_N(MG_kuka_auto_generated.v_NGo_N);
  const Vector3d alpha_NG_N(MG_kuka_auto_generated.alpha_NG_N);
  const Vector3d a_NGo_N(MG_kuka_auto_generated.a_NGo_N);

  return std::make_tuple(R_NG, p_NoGo_N, w_NG_N, v_NGo_N, alpha_NG_N, a_NGo_N);
}


template<typename T>
std::tuple<SpatialForced, SpatialForced, SpatialForced, SpatialForced,
           SpatialForced, SpatialForced, SpatialForced>
MGKukaIIwaRobot<T>::CalcJointReactionForces(
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& qDt,
    const Eigen::Ref<const VectorX<T>>& qDDt) const {
  // Calculate joint reaction torque/forces with MotionGenesis.
  CalcMGOutput(q, qDt, qDDt);

  // Convert MotionGenesis standard C++ matrices to Eigen matrices.
  const SpatialForced F_A_Na(Vector3d(MG_kuka_auto_generated.tA),
                             Vector3d(MG_kuka_auto_generated.fA));
  const SpatialForced F_B_Ab(Vector3d(MG_kuka_auto_generated.tB),
                             Vector3d(MG_kuka_auto_generated.fB));
  const SpatialForced F_C_Bc(Vector3d(MG_kuka_auto_generated.tC),
                             Vector3d(MG_kuka_auto_generated.fC));
  const SpatialForced F_D_Cd(Vector3d(MG_kuka_auto_generated.tD),
                             Vector3d(MG_kuka_auto_generated.fD));
  const SpatialForced F_E_De(Vector3d(MG_kuka_auto_generated.tE),
                             Vector3d(MG_kuka_auto_generated.fE));
  const SpatialForced F_F_Ef(Vector3d(MG_kuka_auto_generated.tF),
                             Vector3d(MG_kuka_auto_generated.fF));
  const SpatialForced F_G_Fg(Vector3d(MG_kuka_auto_generated.tG),
                             Vector3d(MG_kuka_auto_generated.fG));

  return std::make_tuple(F_A_Na, F_B_Ab, F_C_Bc, F_D_Cd,
                         F_E_De, F_F_Ef, F_G_Fg);
}


template<typename T>
Vector7d MGKukaIIwaRobot<T>::CalcRevoluteMotorZTorques(
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& qDt,
    const Eigen::Ref<const VectorX<T>>& qDDt) const {
  // Calculate joint driving torques with MotionGenesis.
  CalcMGOutput(q, qDt, qDDt);

  const double tAz = MG_kuka_auto_generated.tAz;
  const double tBz = MG_kuka_auto_generated.tBz;
  const double tCz = MG_kuka_auto_generated.tCz;
  const double tDz = MG_kuka_auto_generated.tDz;
  const double tEz = MG_kuka_auto_generated.tEz;
  const double tFz = MG_kuka_auto_generated.tFz;
  const double tGz = MG_kuka_auto_generated.tGz;

  // Assemble the information to return.
  Vector7d motor_torques;
  motor_torques << tAz, tBz, tCz, tDz, tEz, tFz, tGz;
  return motor_torques;
}


// Explicitly instantiates on the most common scalar types.
template
class MGKukaIIwaRobot<double>;

}  // namespace MG
}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
