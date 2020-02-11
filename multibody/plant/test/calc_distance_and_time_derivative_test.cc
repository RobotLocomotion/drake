#include "drake/multibody/plant/calc_distance_and_time_derivative.h"

#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
void CheckDistanceAndTimeDerivative(
    const multibody::MultibodyPlant<double>& plant,
    multibody::FrameIndex sphere_frame_index,
    multibody::FrameIndex box_frame_index,
    const SortedPair<geometry::GeometryId>& box_sphere_pair,
    const Eigen::Vector3d& p_WB, const Eigen::Vector4d& quat_WB,
    const Eigen::Vector3d& p_WS, const Eigen::Vector4d& quat_WS,
    const Eigen::Vector3d& v_WB, const Eigen::Vector3d& omega_WB,
    const Eigen::Vector3d& v_WS, const Eigen::Vector3d& omega_WS,
    double distance_expected, double distance_time_derivative_expected,
    systems::Context<double>* plant_context) {
  Eigen::Matrix<double, 14, 1> q;
  Eigen::Matrix<double, 12, 1> v;
  const auto& sphere_body = plant.get_frame(sphere_frame_index).body();
  const auto& box_body = plant.get_frame(box_frame_index).body();
  q.segment<4>(box_body.floating_positions_start()) = quat_WB;
  q.segment<3>(box_body.floating_positions_start() + 4) = p_WB;
  q.segment<4>(sphere_body.floating_positions_start()) = quat_WS;
  q.segment<3>(sphere_body.floating_positions_start() + 4) = p_WS;
  v.segment<3>(box_body.floating_velocities_start() - 14) = omega_WB;
  v.segment<3>(box_body.floating_velocities_start() - 14 + 3) = v_WB;
  v.segment<3>(sphere_body.floating_velocities_start() - 14) = omega_WS;
  v.segment<3>(sphere_body.floating_velocities_start() - 14 + 3) = v_WS;
  Eigen::Matrix<double, 26, 1> q_v;
  q_v << q, v;
  plant.SetPositionsAndVelocities(plant_context, q_v);
  double distance{};
  double distance_time_derivative{};
  CalcDistanceAndTimeDerivative(plant, box_sphere_pair, *plant_context,
                                &distance, &distance_time_derivative);
  EXPECT_NEAR(distance, distance_expected, 1e-12);
  EXPECT_NEAR(distance_time_derivative, distance_time_derivative_expected,
              1e-12);
}

TEST_F(BoxSphereTest, CalcDistanceAndTimeDerivative) {
  // First test when the box and sphere are both static. The contact normal is
  // the +z vector in the world frame.
  Eigen::Vector3d p_WB(0, 0, 0);
  Eigen::Vector4d quat_WB(1, 0, 0, 0);
  Eigen::Vector3d p_WS(0, 0, 20);
  Eigen::Vector4d quat_WS(1, 0, 0, 0);
  Eigen::Vector3d v_WB(0, 0, 0);
  Eigen::Vector3d omega_WB(0, 0, 0);
  Eigen::Vector3d v_WS(0, 0, 0);
  Eigen::Vector3d omega_WS(0, 0, 0);
  CheckDistanceAndTimeDerivative(
      *plant_double_, sphere_frame_index_, box_frame_index_,
      {box_geometry_id_, sphere_geometry_id_}, p_WB, quat_WB, p_WS, quat_WS,
      v_WB, omega_WB, v_WS, omega_WS, 14, 0, plant_context_double_);

  // Box is static, the sphere is moving horizontally. The time derivative of
  // the signed distance should be 0.
  v_WS << 20, 10, 0;
  CheckDistanceAndTimeDerivative(
      *plant_double_, sphere_frame_index_, box_frame_index_,
      {box_geometry_id_, sphere_geometry_id_}, p_WB, quat_WB, p_WS, quat_WS,
      v_WB, omega_WB, v_WS, omega_WS, 14, 0, plant_context_double_);

  // Box is static, the sphere is moving towards the box. The time derivative
  // of the signed distance should be the vertical velocity of the sphere.
  v_WS << 20, 10, -2;
  CheckDistanceAndTimeDerivative(
      *plant_double_, sphere_frame_index_, box_frame_index_,
      {box_geometry_id_, sphere_geometry_id_}, p_WB, quat_WB, p_WS, quat_WS,
      v_WB, omega_WB, v_WS, omega_WS, 14, -2, plant_context_double_);

  // Box is static, the sphere is rotating and moving towards the box. The time
  // derivative of the signed distance should be the vertical velocity of the
  // sphere.
  omega_WS << 2, 3, 4;
  CheckDistanceAndTimeDerivative(
      *plant_double_, sphere_frame_index_, box_frame_index_,
      {box_geometry_id_, sphere_geometry_id_}, p_WB, quat_WB, p_WS, quat_WS,
      v_WB, omega_WB, v_WS, omega_WS, 14, -2, plant_context_double_);

  // Box is translating, the sphere is rotating and moving towards the box. The
  // time derivative of the signed distance should be the relative vertical
  // velocity.
  v_WB << 1, 2, 3;
  CheckDistanceAndTimeDerivative(
      *plant_double_, sphere_frame_index_, box_frame_index_,
      {box_geometry_id_, sphere_geometry_id_}, p_WB, quat_WB, p_WS, quat_WS,
      v_WB, omega_WB, v_WS, omega_WS, 14, -5, plant_context_double_);

  // Box is rotating about x axis, the sphere is static. The time derivative
  // of the signed distance is 0.
  v_WB.setZero();
  v_WS.setZero();
  omega_WS.setZero();
  omega_WB << 2, 0, 0;
  CheckDistanceAndTimeDerivative(
      *plant_double_, sphere_frame_index_, box_frame_index_,
      {box_geometry_id_, sphere_geometry_id_}, p_WB, quat_WB, p_WS, quat_WS,
      v_WB, omega_WB, v_WS, omega_WS, 14, 0, plant_context_double_);

  // The box is tilted about the x axis with angle theta, and rotating about
  // the x-axis with angular velocity theta_dot. The signed distance is
  // 20 * cos(theta) - 6, the time derivative of the signed distance is
  // -20 * sin(theta) * theta_dot.
  double theta = M_PI / 20;
  quat_WB << std::cos(theta / 2), std::sin(theta / 2), 0, 0;
  double thetadot = 0.5;
  omega_WB << thetadot, 0, 0;
  CheckDistanceAndTimeDerivative(
      *plant_double_, sphere_frame_index_, box_frame_index_,
      {box_geometry_id_, sphere_geometry_id_}, p_WB, quat_WB, p_WS, quat_WS,
      v_WB, omega_WB, v_WS, omega_WS, 20 * std::cos(theta) - 6,
      -20 * std::sin(theta) * thetadot, plant_context_double_);
}
}  // namespace multibody
}  // namespace drake
