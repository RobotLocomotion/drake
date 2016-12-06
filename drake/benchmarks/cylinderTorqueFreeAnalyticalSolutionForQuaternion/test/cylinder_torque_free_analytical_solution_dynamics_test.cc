//    File: cylinder_torque_free_analytical_solution_dynamics_test.cc
// Purpose: Compare Drake with analytical closed-form solution from Section
//          1.13 (pgs. 60-62) of Spacecraft Dynamics, by Kane & Levinson.
//          This closed-form solution includes both angular velocity and
//          Euler parameters for an axis-symmetric rigid body B, when the moment
//          of forces on B about Bcm (B's center of mass) is zero (torque-free).
//    Note: uniformSolidCylinder.urdf defines mass and inertia properties.
//-----------------------------------------------------------------------------
#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parser_urdf.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"

namespace drake {
namespace benchmarks {
namespace uniformSolidCylinderTorqueFreeAnalyticalSolution {

//-----------------------------------------------------------------------------
void CalculateExactSolution(const double t, const double wxyzInitial[],
                            double e[], double w[], double eDt[],
                            double wDt[]) {
  // Constant values of moments of inertia.
  const double I = 0.04;
  const double J = 0.02;

  // Initial values of wx, wy, wz.
  const double wx0 = wxyzInitial[0];
  const double wy0 = wxyzInitial[1];
  const double wz0 = wxyzInitial[2];

  // Analytical solution for Euler parameters (quaternion).
  const double p = sqrt(wx0 * wx0 + wy0 * wy0 + (wz0 * J / I) * (wz0 * J / I));
  const double s = (I - J) / I * wz0;
  const double coef = wz0 * (J / (I * p));
  const double spt2 = sin(p * t / 2);
  const double cpt2 = cos(p * t / 2);
  const double sst2 = sin(s * t / 2);
  const double cst2 = cos(s * t / 2);

  const double e1 = spt2 / p * (wx0 * cst2 + wy0 * sst2);
  const double e2 = spt2 / p * (-wx0 * sst2 + wy0 * cst2);
  const double e3 = coef * spt2 * cst2 + cpt2 * sst2;
  const double e0 = -coef * spt2 * sst2 + cpt2 * cst2;

  // Analytical solution for wx(t), wy(t), wz(t).
  const double wx = wx0 * cos(s * t) + wy0 * sin(s * t);
  const double wy = -wx0 * sin(s * t) + wy0 * cos(s * t);
  const double wz = wz0;

  // Analytical solution for time-derivatives of Euler parameters.
  const double e0Dt = -0.5 * e1 * wx - 0.5 * e2 * wy - 0.5 * e3 * wz;
  const double e1Dt = 0.5 * e0 * wx + 0.5 * e2 * wz - 0.5 * e3 * wy;
  const double e2Dt = 0.5 * e0 * wy + 0.5 * e3 * wx - 0.5 * e1 * wz;
  const double e3Dt = 0.5 * e0 * wz + 0.5 * e1 * wy - 0.5 * e2 * wx;

  // Analytical solution for time-derivatives of wx, wy, wz.
  const double wxDt = (1 - J / I) * wy * wz;
  const double wyDt = (-1 + J / I) * wx * wz;
  const double wzDt = 0.0;

  // Return results.
  e[0] = e0;
  eDt[0] = e0Dt;
  e[1] = e1;
  eDt[1] = e1Dt;
  e[2] = e2;
  eDt[2] = e2Dt;
  e[3] = e3;
  eDt[3] = e3Dt;
  w[0] = wx;
  wDt[0] = wxDt;
  w[1] = wy;
  wDt[1] = wyDt;
  w[2] = wz;
  wDt[2] = wzDt;
}

//-----------------------------------------------------------------------------
// GTEST_TEST: 1st arg is name of test, 2nd arg is name of sub-test.
GTEST_TEST(uniformSolidCylinderTorqueFree, testA) {
  // Create the path to the file containing geometry.
  const std::string path_to_urdf_file = GetDrakePath() +
                                        "/benchmarks/"
                                        "cylinderTorqueFreeAnalyticalSolutionFo"
                                        "rQuaternion/uniformSolidCylinder.urdf";

  // Create a default RigidBodyTree constructor.
  std::unique_ptr<RigidBodyTree<double>> tree =
      std::make_unique<RigidBodyTree<double>>();

  // Populate RigidBodyTree tree by providing path to .urdf file,
  // second argument is enum FloatingBaseType that specifies the joint
  // connecting the robot's base link to ground/world. enum values are:
  // kFixed         welded to ground,.
  // kRollPitchYaw  translates x,y,z and rotates 3D (by SpaceXYZ Euler angles).
  // kQuaternion    translates x,y,z and rotates 3D (by Euler parameters).
  const drake::multibody::joints::FloatingBaseType joint_type =
      drake::multibody::joints::kQuaternion;
  std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      path_to_urdf_file, joint_type, weld_to_frame, tree.get());

  // Create a RigidBodyPlant which takes ownership of tree.
  // Note: unique_ptr helps ensure tree is only managed/deleted once.
  drake::systems::RigidBodyPlant<double> rBPlant(std::move(tree));

  // Create a Context: Which is similar to Simbody's state and cache.
  std::unique_ptr<systems::Context<double>> contextB =
      rBPlant.CreateDefaultContext();

  // Get the state portion of the Context. State has weird/inconsistent order.
  // State for joints::kQuaternion   -- x,y,z, e0,e1,e2,e3, wx,wy,wz, x',y',z'.
  // State for joints::kRollPitchYaw -- x,y,z, q1,q2,q3, x',y',z', q1',q2',q3'
  // (where q1=Roll, q2=Pitch, q3=Yaw for SpaceXYZ rotation sequence).
  systems::VectorBase<double>& s =
      *(contextB->get_mutable_continuous_state_vector());

  // Create 3x1 Eigen column matrix for x, y, z.
  // Create 3x1 Eigen column matrix for x', y', z'.
  Eigen::Vector3d xyz(1.0, 2.0, 3.0);
  Eigen::Vector3d xyzDt(0.0, 0.0, 0.0);

  // Create 4x1 Eigen column matrix for quaternion e0, e1, e2, e3.
  // Create 3x1 Eigen column matrix for wx, wy, wz.
  Eigen::Vector4d quaternion(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector3d wxyzInitial(2.0, 4.0, 6.0);

  // Concatenate these 4 Eigen column matrices into one Eigen column matrix.
  // Note: The state has a weird order (see previous comment).
  Eigen::Matrix<double, 13, 1> eigenState;
  eigenState << xyz, quaternion, wxyzInitial, xyzDt;

  // Set state portion of Context using eigenState.
  s.SetFromVector(eigenState);

  // Construct enough space to hold time-derivative of state s.
  std::unique_ptr<systems::ContinuousState<double>> ds =
      rBPlant.AllocateTimeDerivatives();
  systems::ContinuousState<double>* sDt = ds.get();

  // Setup an empty input port - that serves no purpose but to frustrate me.
  const int numActuators = rBPlant.get_num_actuators();
  std::unique_ptr<systems::FreestandingInputPort> port =
      std::make_unique<systems::FreestandingInputPort>(
          std::make_unique<systems::BasicVector<double>>(numActuators));
  contextB->SetInputPort(0, std::move(port));

  // Evaluate the time-derivatives of the state.
  systems::Context<double>* rawContextB = contextB.get();
  rBPlant.EvalTimeDerivatives(*rawContextB, sDt);

  // Get the state and state time-derivatives for comparison.
  // State for joints::kQuaternion   -- x,y,z, e0,e1,e2,e3, wx,wy,wz, x',y',z'.
  Eigen::VectorXd stateEigen = s.CopyToVector();
  Eigen::Vector4d eDrakeEigen = stateEigen.segment<4>(3);
  Eigen::Vector3d wDrakeEigen = stateEigen.segment<3>(7);

  Eigen::VectorXd stateDtEigen = sDt->CopyToVector();
  Eigen::Vector4d eDtDrakeEigen = stateDtEigen.segment<4>(3);
  Eigen::Vector3d wDtDrakeEigen = stateDtEigen.segment<3>(7);

  // Calculate analytical solution.
  double eExact[4], wExact[3], eDtExact[4], wDtExact[3];
  const double* wxyzInitialDouble = wxyzInitial.data();
  const double t = 0;
  CalculateExactSolution(t, wxyzInitialDouble, eExact, wExact, eDtExact,
                         wDtExact);

  // To allow Eigen to compare results, create Eigen-versions of exact arrays.
  Eigen::Map<Eigen::Vector4d> eExactEigen(eExact);
  Eigen::Map<Eigen::Vector3d> wExactEigen(wExact);
  Eigen::Map<Eigen::Vector4d> eDtExactEigen(eDtExact);
  Eigen::Map<Eigen::Vector3d> wDtExactEigen(wDtExact);

  // Compare with exact results.
  EXPECT_TRUE(CompareMatrices(eDrakeEigen, eExactEigen, 1.0E-15));
  EXPECT_TRUE(CompareMatrices(wDrakeEigen, wExactEigen, 1.0E-15));
  EXPECT_TRUE(CompareMatrices(eDtDrakeEigen, eDtExactEigen, 1.0E-15));
  EXPECT_TRUE(CompareMatrices(wDtDrakeEigen, wDtExactEigen, 1.0E-15));
}

}  // namespace uniformSolidCylinderTorqueFreeAnalyticalSolution
}  // namespace benchmarks
}  // namespace drake
