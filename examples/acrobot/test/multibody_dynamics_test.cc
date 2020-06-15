#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

// Tests that the hand-derived dynamics (from the textbook) match the dynamics
// generated from the urdf and sdf files via the MultibodyPlant class.
GTEST_TEST(MultibodyDynamicsTest, AllTests) {
  for (const std::string& ext : {"urdf", "sdf"}) {
    const double kTimeStep = 0.0;
    multibody::MultibodyPlant<double> mbp(kTimeStep);
    multibody::Parser(&mbp).AddModelFromFile(
        FindResourceOrThrow("drake/examples/acrobot/Acrobot." + ext));
    mbp.Finalize();

    AcrobotPlant<double> p;

    auto context_mbp = mbp.CreateDefaultContext();
    auto context_p = p.CreateDefaultContext();

    auto& u_mbp = mbp.get_actuation_input_port().FixValue(context_mbp.get(),
                                                          0.0);
    auto& u_p = p.get_input_port(0).FixValue(context_p.get(), 0.0);

    Eigen::Vector4d x;
    Vector1d u;
    auto xdot_mbp = mbp.AllocateTimeDerivatives();
    auto xdot_p = p.AllocateTimeDerivatives();
    auto y_mbp = mbp.AllocateOutput();
    auto y_p = p.AllocateOutput();

    srand(42);
    for (int i = 0; i < 100; ++i) {
      x = Eigen::Vector4d::Random();
      u = Vector1d::Random();

      context_mbp->get_mutable_continuous_state_vector().SetFromVector(x);
      context_p->get_mutable_continuous_state_vector().SetFromVector(x);

      u_mbp.GetMutableVectorData<double>()->SetFromVector(u);
      u_p.GetMutableVectorData<double>()->SetFromVector(u);

      mbp.CalcTimeDerivatives(*context_mbp, xdot_mbp.get());
      p.CalcTimeDerivatives(*context_p, xdot_p.get());

      EXPECT_TRUE(CompareMatrices(xdot_mbp->CopyToVector(),
                                  xdot_p->CopyToVector(), 1e-8,
                                  MatrixCompareType::absolute));

      mbp.CalcOutput(*context_mbp, y_mbp.get());
      p.CalcOutput(*context_p, y_p.get());

      EXPECT_TRUE(CompareMatrices(
          y_mbp->get_vector_data(mbp.get_state_output_port().get_index())
              ->CopyToVector(),
          y_p->get_vector_data(0)->CopyToVector(), 1e-8,
          MatrixCompareType::absolute));
    }
  }
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake
