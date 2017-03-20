#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/Acrobot/acrobot_plant.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

// Tests that the hand-derived dynamics (from the textbook) match the dynamics
// generated from the urdf via the RigidBodyPlant class.
GTEST_TEST(UrdfDynamicsTest, AllTests) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    GetDrakePath() + "/examples/Acrobot/Acrobot.urdf",
    multibody::joints::kFixed, tree.get());

  systems::RigidBodyPlant<double> rbp(std::move(tree));
  AcrobotPlant<double> p;

  auto context_rbp = rbp.CreateDefaultContext();
  auto context_p = p.CreateDefaultContext();

  auto& u_rbp = context_rbp->FixInputPort(0, Vector1d::Zero());
  auto& u_p = context_p->FixInputPort(0, Vector1d::Zero());

  Eigen::Vector4d x;
  Vector1d u;
  auto xdot_rbp = rbp.AllocateTimeDerivatives();
  auto xdot_p = p.AllocateTimeDerivatives();
  auto y_rbp = rbp.AllocateOutput(*context_rbp);
  auto y_p = p.AllocateOutput(*context_p);

  srand(42);
  for (int i = 0; i < 100; ++i) {
    x = Eigen::Vector4d::Random();
    u = Vector1d::Random();

    context_rbp->get_mutable_continuous_state_vector()->SetFromVector(x);
    context_p->get_mutable_continuous_state_vector()->SetFromVector(x);

    u_rbp.GetMutableVectorData<double>()->SetFromVector(u);
    u_p.GetMutableVectorData<double>()->SetFromVector(u);

    rbp.CalcTimeDerivatives(*context_rbp, xdot_rbp.get());
    p.CalcTimeDerivatives(*context_p, xdot_p.get());

    EXPECT_TRUE(CompareMatrices(xdot_rbp->CopyToVector(),
                                xdot_p->CopyToVector(), 1e-8,
                                MatrixCompareType::absolute));

    rbp.CalcOutput(*context_rbp, y_rbp.get());
    p.CalcOutput(*context_p, y_p.get());

    EXPECT_TRUE(CompareMatrices(y_rbp->get_vector_data(0)->CopyToVector(),
                                y_p->get_vector_data(0)->CopyToVector(), 1e-8,
                                MatrixCompareType::absolute));
  }
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake
