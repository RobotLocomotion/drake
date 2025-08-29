#include "drake/multibody/fem/force_density_field_base.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/force_density_field.h"

namespace drake {
namespace multibody {
namespace fem {
namespace {

using drake::systems::BasicVector;
using Eigen::Vector3d;

/* A concrete force density field for testing. */
class ConstantForceDensityField final : public ForceDensityField<double> {
 public:
  /* Constructs an force density field that has the functional form given by
   input `field` which is then scaled by a scalar value via input port. */
  explicit ConstantForceDensityField(
      std::function<Vector3<double>(const Vector3<double>&)> field)
      : field_(std::move(field)) {}

  /* Gets the double-valued input port -- the input port value scales the
   force density multiplicatively. */
  const systems::InputPort<double>& get_input_port() const {
    return parent_system_or_throw().get_input_port(scale_port_index_);
  }

 private:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConstantForceDensityField);

  Vector3<double> DoEvaluateAt(const systems::Context<double>& context,
                               const Vector3<double>& p_WQ) const final {
    return get_input_port().Eval(context)(0) * field_(p_WQ);
  };

  std::unique_ptr<ForceDensityFieldBase<double>> DoClone() const final {
    return std::unique_ptr<ConstantForceDensityField>(
        new ConstantForceDensityField(*this));
  }

  void DoDeclareInputPorts(multibody::MultibodyPlant<double>* plant) final {
    scale_port_index_ =
        this->DeclareVectorInputPort(plant, "on/off signal for the force field",
                                     BasicVector<double>(1.0))
            .get_index();
  }

  std::function<Vector3<double>(const Vector3<double>&)> field_;
  systems::InputPortIndex scale_port_index_;
};

class ForceDensityFieldTest : public ::testing::Test {
 protected:
  void SetUp() override {
    /* Define an arbtrary force density field. */
    auto force_field = [](const Vector3d& x) {
      return Vector3d(std::sin(x[0]), std::cos(x[1]), x[2]);
    };
    dut_ = std::make_unique<ConstantForceDensityField>(force_field);
    dut_->DeclareSystemResources(&plant_);
    plant_.Finalize();
  }

  std::unique_ptr<ConstantForceDensityField> dut_;
  MultibodyPlant<double> plant_{0.01};
};

TEST_F(ForceDensityFieldTest, EvaluateAt) {
  auto plant_context = plant_.CreateDefaultContext();
  EXPECT_EQ(dut_->density_type(), ForceDensityType::kPerCurrentVolume);
  const double scale = 2.71;
  dut_->get_input_port().FixValue(plant_context.get(), Vector1d(scale));
  const Vector3d p_WQ(1, 2, 3);
  const Vector3d expected_force_density =
      scale * Vector3d(std::sin(p_WQ[0]), std::cos(p_WQ[1]), p_WQ[2]);
  EXPECT_EQ(dut_->EvaluateAt(*plant_context, p_WQ), expected_force_density);
}

TEST_F(ForceDensityFieldTest, Clone) {
  auto clone = dut_->Clone();
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->density_type(), dut_->density_type());

  auto concrete_clone = dynamic_cast<ConstantForceDensityField*>(clone.get());
  ASSERT_NE(concrete_clone, nullptr);
  MultibodyPlant<double> plant_clone(0.01);
  concrete_clone->DeclareSystemResources(&plant_clone);
  plant_clone.Finalize();
  auto plant_context = plant_.CreateDefaultContext();
  auto clone_context = plant_clone.CreateDefaultContext();

  const double scale = 2.71;
  dut_->get_input_port().FixValue(plant_context.get(), Vector1d(scale));
  concrete_clone->get_input_port().FixValue(clone_context.get(),
                                            Vector1d(scale));
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        const Vector3d p_WQ(i, j, k);
        EXPECT_EQ(dut_->EvaluateAt(*plant_context, p_WQ),
                  clone->EvaluateAt(*clone_context, p_WQ));
      }
    }
  }
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
