#pragma once

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
// Declare a constraint p * x(0) + x(1) >= 2
//                      x(1) * x(1) + p * x(0) * x(0) <= x(2)
template <typename T>
void DummySystemConstraintCalc(const Context<T>& context, VectorX<T>* y) {
  const T p = context.get_numeric_parameter(0).GetAtIndex(0);
  const Vector3<T> x = context.get_continuous_state_vector().CopyToVector();
  y->resize(2);
  (*y)(0) = p * x(0) + x(1);
  (*y)(1) = x(2) - x(1) * x(1) - p * x(0) * x(0);
}

/**
 * A dummy continuous system with parameter(s).
 */
template <typename T>
class DummySystem : public LeafSystem<T> {
 public:
  DummySystem(bool with_abstract_state = false,
              bool with_abstract_parameter = false)
      : LeafSystem<T>(SystemTypeTag<DummySystem>{}),
        with_abstract_state_(with_abstract_state),
        with_abstract_parameter_(with_abstract_parameter) {
    this->DeclareContinuousState(3);
    this->DeclareDiscreteState(2);
    this->DeclareNumericParameter(BasicVector<T>(Vector1<T>::Constant(1.0)));
    this->DeclareInputPort("u", PortDataType::kVectorValued, 2);
    this->constraint_index_ = this->DeclareInequalityConstraint(
        DummySystemConstraintCalc<T>, {Eigen::Vector2d(2, 0), std::nullopt},
        "dummy_system_constraint");
    if (with_abstract_state_) {
      this->DeclareAbstractState(Value<int>(5));
    }
    if (with_abstract_parameter_) {
      this->DeclareAbstractParameter(Value<std::string>("parameter"));
    }
  }

  template <typename U>
  explicit DummySystem(const DummySystem<U>& other)
      : DummySystem(other.with_abstract_state(),
                    other.with_abstract_parameter()) {}

  SystemConstraintIndex constraint_index() const { return constraint_index_; }

  bool with_abstract_state() const { return with_abstract_state_; }

  bool with_abstract_parameter() const { return with_abstract_parameter_; }

 private:
  // xdot = [p0 * x(0); p0 * x(1) + x(0), x(2) - x(1)]
  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override {
    const Vector3<T> x = context.get_continuous_state_vector().CopyToVector();
    const T& p0 = context.get_numeric_parameter(0).GetAtIndex(0);
    const Vector3<T> xdot(p0 * x(0), p0 * x(1) + x(0), x(2) - x(1));
    derivatives->SetFromVector(xdot);
  }

  SystemConstraintIndex constraint_index_;
  bool with_abstract_state_{};
  bool with_abstract_parameter_{};
};

template <typename T>
void CalcUnitQuaternion(const System<T>& system, const Context<T>& context,
                        VectorX<T>* value) {
  (*value)(0) = dynamic_cast<const multibody::MultibodyPlant<T>&>(system)
                    .GetPositions(context)
                    .template head<4>()
                    .squaredNorm() -
                T(1.0);
}

class FreeBodyPlantTest : public ::testing::Test {
 public:
  FreeBodyPlantTest()
      : plant_double_{std::make_unique<multibody::MultibodyPlant<double>>(0)} {
    const double mass{1};
    const Eigen::Vector3d p_BoBcm_B(0, 0, 0);
    const multibody::RotationalInertia<double> I_BBcm_B{0.01, 0.01, 0.01};
    const multibody::SpatialInertia<double> M_BBo_B =
        multibody::SpatialInertia<double>::MakeFromCentralInertia(
            mass, p_BoBcm_B, I_BBcm_B);
    plant_double_->AddRigidBody("body", M_BBo_B);

    auto unit_quaternion_constraint =
        ExternalSystemConstraint::MakeForNonsymbolicScalars(
            "quaternion_norm_square = 1", SystemConstraintBounds::Equality(1),
            [](const auto& system, const auto& context, auto* value) {
              CalcUnitQuaternion(system, context, value);
            });

    this->unit_quaternion_constraint_index_ =
        plant_double_->AddExternalConstraint(unit_quaternion_constraint);

    plant_double_->Finalize();
    plant_autodiff_ =
        dynamic_pointer_cast<multibody::MultibodyPlant<AutoDiffXd>>(
            plant_double_->ToAutoDiffXd());
  }

  SystemConstraintIndex unit_quaternion_constraint_index() const {
    return unit_quaternion_constraint_index_;
  }

 protected:
  std::unique_ptr<multibody::MultibodyPlant<double>> plant_double_;
  std::unique_ptr<multibody::MultibodyPlant<AutoDiffXd>> plant_autodiff_;
  SystemConstraintIndex unit_quaternion_constraint_index_;
};

}  // namespace systems
}  // namespace drake
