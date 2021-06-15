#include "drake/multibody/benchmarks/kuka_iiwa_robot/MG/MG_kuka_iiwa_robot.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {
namespace MG {

using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::Vector3d;
using SpatialForced = SpatialForce<double>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using test_utilities::SpatialKinematicsPVA;

template<typename T>
void MGKukaIIwaRobot<T>::PrepareMGOutput(
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& qDt,
    const Eigen::Ref<const VectorX<T>>& qDDt) const {
  // Form a continuous vector of joint angles and their time-derivatives.
  const Eigen::Index size = q.size() + qDt.size() + qDDt.size();
  VectorX<double> joint_angles_and_1st_2nd_derivatives(size);
  joint_angles_and_1st_2nd_derivatives << q, qDt, qDDt;

  // Calculate MotionGenesis output quantities.
  MG_kuka_auto_generated_.SetVariablesFromArray(
        joint_angles_and_1st_2nd_derivatives.data());
  MG_kuka_auto_generated_.CalculateOutput();
}


template<typename T>
SpatialKinematicsPVA<T> MGKukaIIwaRobot<T>::CalcEndEffectorKinematics(
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& qDt,
    const Eigen::Ref<const VectorX<T>>& qDDt) const {
  // Calculate forward kinematics of end-effector with MotionGenesis.
  PrepareMGOutput(q, qDt, qDDt);

  // Convert MotionGenesis standard C++ matrix to an Eigen matrix.
  // Note: Standard C++ stores two-dimensional matrices in row-major form.
  // By default, Eigen stores two-dimensional matrices in column-major form.
  // After an Eigen matrix is constructed from the C++ matrix (interpreting the
  // C++ matrix in row-order), column-major R_NG is populated via operator=.
  const double* matrix33 = MG_kuka_auto_generated_.R_NG[0];
  const Matrix3d R_NG33 = Matrix<double, 3, 3, Eigen::RowMajor>(matrix33);
  const math::RotationMatrixd R_NG(R_NG33);

  const Vector3d p_NoGo_N(MG_kuka_auto_generated_.p_NoGo_N);
  const Vector3d w_NG_N(MG_kuka_auto_generated_.w_NG_N);
  const Vector3d v_NGo_N(MG_kuka_auto_generated_.v_NGo_N);
  const Vector3d alpha_NG_N(MG_kuka_auto_generated_.alpha_NG_N);
  const Vector3d a_NGo_N(MG_kuka_auto_generated_.a_NGo_N);

  return SpatialKinematicsPVA<T>(R_NG, p_NoGo_N, w_NG_N, v_NGo_N, alpha_NG_N,
      a_NGo_N);
}


template<typename T>
std::tuple<SpatialForced, SpatialForced, SpatialForced, SpatialForced,
           SpatialForced, SpatialForced, SpatialForced>
MGKukaIIwaRobot<T>::CalcJointReactionForcesExpressedInMobilizer(
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& qDt,
    const Eigen::Ref<const VectorX<T>>& qDDt) const {
  // Calculate joint reaction torque/forces with MotionGenesis.
  PrepareMGOutput(q, qDt, qDDt);

  // Convert MotionGenesis standard C++ matrices to Eigen matrices.
  const SpatialForced F_A_Na(Vector3d(MG_kuka_auto_generated_.tA),
                             Vector3d(MG_kuka_auto_generated_.fA));
  const SpatialForced F_B_Ab(Vector3d(MG_kuka_auto_generated_.tB),
                             Vector3d(MG_kuka_auto_generated_.fB));
  const SpatialForced F_C_Bc(Vector3d(MG_kuka_auto_generated_.tC),
                             Vector3d(MG_kuka_auto_generated_.fC));
  const SpatialForced F_D_Cd(Vector3d(MG_kuka_auto_generated_.tD),
                             Vector3d(MG_kuka_auto_generated_.fD));
  const SpatialForced F_E_De(Vector3d(MG_kuka_auto_generated_.tE),
                             Vector3d(MG_kuka_auto_generated_.fE));
  const SpatialForced F_F_Ef(Vector3d(MG_kuka_auto_generated_.tF),
                             Vector3d(MG_kuka_auto_generated_.fF));
  const SpatialForced F_G_Fg(Vector3d(MG_kuka_auto_generated_.tG),
                             Vector3d(MG_kuka_auto_generated_.fG));

  return std::make_tuple(F_A_Na, F_B_Ab, F_C_Bc, F_D_Cd,
                         F_E_De, F_F_Ef, F_G_Fg);
}


template<typename T>
std::tuple<SpatialForced, SpatialForced, SpatialForced, SpatialForced,
           SpatialForced, SpatialForced, SpatialForced>
MGKukaIIwaRobot<T>::CalcJointReactionForcesExpressedInWorld(
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& qDt,
    const Eigen::Ref<const VectorX<T>>& qDDt) const {
  // Calculate joint reaction torque/forces with MotionGenesis.
  PrepareMGOutput(q, qDt, qDDt);

  // Convert MotionGenesis standard C++ matrices to Eigen matrices.
  const SpatialForced F_A_W(Vector6d(MG_kuka_auto_generated_.SpatialForce_A_N));
  const SpatialForced F_B_W(Vector6d(MG_kuka_auto_generated_.SpatialForce_B_N));
  const SpatialForced F_C_W(Vector6d(MG_kuka_auto_generated_.SpatialForce_C_N));
  const SpatialForced F_D_W(Vector6d(MG_kuka_auto_generated_.SpatialForce_D_N));
  const SpatialForced F_E_W(Vector6d(MG_kuka_auto_generated_.SpatialForce_E_N));
  const SpatialForced F_F_W(Vector6d(MG_kuka_auto_generated_.SpatialForce_F_N));
  const SpatialForced F_G_W(Vector6d(MG_kuka_auto_generated_.SpatialForce_G_N));
  return std::make_tuple(F_A_W, F_B_W, F_C_W, F_D_W, F_E_W, F_F_W, F_G_W);
}


template<typename T>
Vector7d MGKukaIIwaRobot<T>::CalcRevoluteMotorZTorques(
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& qDt,
    const Eigen::Ref<const VectorX<T>>& qDDt) const {
  // Calculate joint driving torques with MotionGenesis.
  PrepareMGOutput(q, qDt, qDDt);

  const double tAz = MG_kuka_auto_generated_.tAz;
  const double tBz = MG_kuka_auto_generated_.tBz;
  const double tCz = MG_kuka_auto_generated_.tCz;
  const double tDz = MG_kuka_auto_generated_.tDz;
  const double tEz = MG_kuka_auto_generated_.tEz;
  const double tFz = MG_kuka_auto_generated_.tFz;
  const double tGz = MG_kuka_auto_generated_.tGz;

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
