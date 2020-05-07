#include "drake/multibody/tree/force_element.h"

#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

namespace {

constexpr double kMass = 1.0;              // [Kg]
constexpr double kPrincipalInertia = 1.0;  // [Kg m^2]
constexpr double kStiffness = 5.0;         // [N/m]
constexpr double kDamping = 2.0;           // [Ns/m]

using Eigen::Vector3d;
using math::RigidTransform;
using math::RigidTransformd;

/// Implements a simple one dimensional spring-damper element.
/// This force element does not support energy budgetting nor scalar conversion.
/// This essentially amounts to the minimal code you need to write to implement
/// a force element and get forces properly applied. See
/// OneDimensionalSpringDamperWithEnergyBudgeting below for an implementation
/// that supports energy budgets and scalar conversion.
template <typename T>
class OneDimensionalSpringDamper : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OneDimensionalSpringDamper);

  //// Simple model of a spring-damper applying forces on `body`.
  OneDimensionalSpringDamper(const Body<T>& body, double stiffness,
                             double damping)
      : OneDimensionalSpringDamper(body.index(), body.model_instance(),
                                   stiffness, damping) {}

  const Body<T>& body() const {
    const auto& plant = this->GetParentPlant();
    return plant.get_body(body_index_);
  }

  double stiffness() const { return stiffness_; }

  double damping() const { return damping_; }

  /// Total force on body() as: f = -k * x - c * v, with x and v the x
  /// components of the position and velocity of body() in the world,
  /// respectively.
  Vector3<T> CalcForce(const systems::Context<T>& context) const {
    return CalcSpringForce(context) + CalcDamperForce(context);
  }

  /// The spring component only, f_spring = -k * x.
  Vector3<T> CalcSpringForce(const systems::Context<T>& context) const {
    const Vector3<T>& p_WB = body().EvalPoseInWorld(context).translation();
    // This one-dimensional model only uses the x components.
    const T& x = p_WB[0];
    const T f = -stiffness() * x;
    return Vector3<T>(0.0, 0.0, f);
  }

  /// The damper component only, f_damper = -c * v.
  Vector3<T> CalcDamperForce(const systems::Context<T>& context) const {
    const Vector3<T>& v_WB =
        body().EvalSpatialVelocityInWorld(context).translational();
    // This one-dimensional model only uses the x components.
    const T& v = v_WB[0];
    const T f = -damping() * v;
    return Vector3<T>(0.0, 0.0, f);
  }

 protected:
  /// T agnostic constructor to be used in the subclass's scalar conversion
  /// overloads
  OneDimensionalSpringDamper(BodyIndex body_index,
                             ModelInstanceIndex model_instance,
                             double stiffness, double damping)
      : ForceElement<T>(model_instance),
        body_index_(body_index),
        stiffness_(stiffness),
        damping_(damping) {}

 private:
  BodyIndex body_index_;
  double stiffness_{0.0};
  double damping_{0.0};

  /// Minimum ForceElement overload requirement in order to have a functional
  /// force element model.
  void DoAddForceContribution(const systems::Context<T>& context,
                              MultibodyForces<T>* forces) const final {
    const SpatialForce<T> F_Bo_W(Vector3<T>::Zero(), CalcForce(context));
    body().AddInForceInWorld(context, F_Bo_W, &*forces);
  }
};

/// This class extends OneDimensionalSpringDamper above by implementing energy
/// budget and scalar conversion methods.
template <typename T>
class OneDimensionalSpringDamperWithEnergyBudgeting
    : public OneDimensionalSpringDamper<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(
      OneDimensionalSpringDamperWithEnergyBudgeting);

  OneDimensionalSpringDamperWithEnergyBudgeting(const Body<T>& body,
                                                double stiffness,
                                                double damping)
      : OneDimensionalSpringDamperWithEnergyBudgeting(
            body.index(), body.model_instance(), stiffness, damping) {}

 private:
  // Make OneDimensionalSpringDamperWithEnergyBudgeting templated on every other
  // scalar type a friend of OneDimensionalSpringDamperWithEnergyBudgeting<T> so
  // that scalar conversion methods can access private members of
  // OneDimensionalSpringDamperWithEnergyBudgeting<T>.
  template <typename>
  friend class OneDimensionalSpringDamperWithEnergyBudgeting;

  // T agnostic constructor to be used in scalar conversion overloads.
  OneDimensionalSpringDamperWithEnergyBudgeting(
      BodyIndex body_index, ModelInstanceIndex model_instance, double stiffness,
      double damping)
      : OneDimensionalSpringDamper<T>(body_index, model_instance, stiffness,
                                      damping) {}

  // ForceElement overload allowing MultibodyPlant to track potential energy.
  T DoCalcPotentialEnergy(const systems::Context<T>& context) const final {
    const RigidTransform<T>& X_WB = this->body().EvalPoseInWorld(context);
    const T x = X_WB.translation()[0];
    return 0.5 * this->stiffness() * x * x;
  }

  // ForceElement overload allowing MultibodyPlant to track the work performed
  // by conservative forces.
  // This quantity is defined to be positive when the potential energy is
  // decreasing. In other words, with `PE` the potential energy, the
  // conservative power is defined as `Pc = -d(PE)/dt`.
  T DoCalcConservativePower(const systems::Context<T>& context) const final {
    const Vector3<T> fspring_Bo_W = this->CalcSpringForce(context);
    const Vector3<T>& v_WB =
        this->body().EvalSpatialVelocityInWorld(context).translational();
    return -fspring_Bo_W.dot(v_WB);
  }

  // ForceElement overload allowing MBP to track the work performed by
  // non-conservative forces.
  // Rate at which mechanical energy is being dissipated (negative).
  T DoCalcNonConservativePower(const systems::Context<T>& context) const final {
    const Vector3<T> fdamper_Bo_W = this->CalcDamperForce(context);
    const Vector3<T>& v_WB =
        this->body().EvalSpatialVelocityInWorld(context).translational();
    return -fdamper_Bo_W.dot(v_WB);
  }

  // These overload below enable scalar-conversion of MultibodyPlant models with
  // force elements.
  std::unique_ptr<ForceElement<double>> DoToDouble() const final {
    return std::unique_ptr<
        OneDimensionalSpringDamperWithEnergyBudgeting<double>>(
        new OneDimensionalSpringDamperWithEnergyBudgeting<double>(
            this->body().index(), this->model_instance(), this->stiffness(),
            this->damping()));
  }

  std::unique_ptr<ForceElement<AutoDiffXd>> DoToAutoDiffXd() const final {
    return std::unique_ptr<
        OneDimensionalSpringDamperWithEnergyBudgeting<AutoDiffXd>>(
        new OneDimensionalSpringDamperWithEnergyBudgeting<AutoDiffXd>(
            this->body().index(), this->model_instance(), this->stiffness(),
            this->damping()));
  }

  std::unique_ptr<ForceElement<symbolic::Expression>> DoToSymbolic()
      const final {
    return std::unique_ptr<
        OneDimensionalSpringDamperWithEnergyBudgeting<symbolic::Expression>>(
        new OneDimensionalSpringDamperWithEnergyBudgeting<symbolic::Expression>(
            this->body().index(), this->model_instance(), this->stiffness(),
            this->damping()));
  }
};

class OneDimensionalSpringDamperTest : public ::testing::Test {
 protected:
  // Helper method to build a model with a single body and a
  // OneDimensionalSpringDamper on it.
  void BuildModel(bool model_with_budgeting) {
    // Irrelevant for this test whether the model is continuous or discrete.
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);

    const auto M_B = SpatialInertia<double>::MakeFromCentralInertia(
        kMass, Vector3d::Zero(),
        kMass * UnitInertia<double>::TriaxiallySymmetric(kPrincipalInertia));
    body_ = &plant_->AddRigidBody("body", M_B);

    if (model_with_budgeting) {
      spring_damper_ =
          &plant_
               ->AddForceElement<OneDimensionalSpringDamperWithEnergyBudgeting>(
                   *body_, kStiffness, kDamping);
    } else {
      spring_damper_ = &plant_->AddForceElement<OneDimensionalSpringDamper>(
          *body_, kStiffness, kDamping);
    }

    // Set gravity to zero so that we have one less energy term to consider when
    // verifying the energy correctness of the force element under test.
    plant_->mutable_gravity_field().set_gravity_vector(Vector3d::Zero());

    // Finish building the model and create the default context.
    plant_->Finalize();
    context_ = plant_->CreateDefaultContext();
  }

  void SetOneDimensionalState(double x, double v) {
    const Vector3d p_WB(x, 0.0, 0.0);
    const Vector3d v_WB(v, 0.0, 0.0);
    plant_->SetFreeBodyPose(context_.get(), *body_, RigidTransformd(p_WB));
    plant_->SetFreeBodySpatialVelocity(
        context_.get(), *body_,
        SpatialVelocity<double>(Vector3d::Zero(), v_WB));
  }

  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<systems::Context<double>> context_;

  const Body<double>* body_{nullptr};
  const OneDimensionalSpringDamper<double>* spring_damper_{nullptr};
};

// This test verifies forces are applied correctly at the MultibodyPlant level.
TEST_F(OneDimensionalSpringDamperTest, ForceContribution) {
  BuildModel(false /* Build basic model with no support for energy budgets */);

  // Set an arbitrary non-zero state.
  SetOneDimensionalState(1.0, 2.0);

  // Expected drag force contribution.
  const Vector3d f_WB = spring_damper_->CalcForce(*context_);

  // Verify the force contribution gets added properly.
  MultibodyForces<double> forces(*plant_);
  plant_->CalcForceElementsContribution(*context_, &forces);
  const SpatialForce<double>& F_B_W = forces.body_forces()[body_->node_index()];
  EXPECT_EQ(F_B_W.translational(), f_WB);
  EXPECT_EQ(F_B_W.rotational(), Vector3d::Zero());
}

// This test verifies that the use of non-implemented APIs results in an
// exception.
TEST_F(OneDimensionalSpringDamperTest, BasicModelWithNoBudgetingThrows) {
  BuildModel(false /* Build basic model with no energy budgets */);
  EXPECT_THROW(spring_damper_->CalcPotentialEnergy(*context_),
               std::logic_error);
  EXPECT_THROW(spring_damper_->CalcConservativePower(*context_),
               std::logic_error);
  EXPECT_THROW(spring_damper_->CalcNonConservativePower(*context_),
               std::logic_error);
}

// This test verifies the proper tracking of energy at the MultibodyPlant level.
TEST_F(OneDimensionalSpringDamperTest, ModelWithEnergyBudgeting) {
  BuildModel(true /* Build full model that supports energy budgets */);

  // Set an arbitrary non-zero state.
  SetOneDimensionalState(1.0, 2.0);

  EXPECT_NO_THROW(spring_damper_->CalcPotentialEnergy(*context_));
  EXPECT_EQ(spring_damper_->CalcPotentialEnergy(*context_),
            plant_->CalcPotentialEnergy(*context_));

  EXPECT_NO_THROW(spring_damper_->CalcConservativePower(*context_));
  EXPECT_EQ(spring_damper_->CalcConservativePower(*context_),
            plant_->CalcConservativePower(*context_));

  EXPECT_NO_THROW(spring_damper_->CalcNonConservativePower(*context_));
  EXPECT_EQ(spring_damper_->CalcNonConservativePower(*context_),
            plant_->CalcNonConservativePower(*context_));
}

// This test verifies the proper scalar-conversion of a model with force
// elements.
TEST_F(OneDimensionalSpringDamperTest, ScalarConversion) {
  BuildModel(
      true /* Full model supporting energy budgets and scalar  conversion */);

  // To AutoDiffXd.
  const MultibodyPlant<AutoDiffXd> plant_ad(*plant_);
  ASSERT_NO_THROW(plant_ad.get_force_element(spring_damper_->index()));
  EXPECT_EQ(plant_ad.get_force_element(spring_damper_->index()).index(),
            spring_damper_->index());

  // To symbolic::Expression.
  const MultibodyPlant<symbolic::Expression> plant_sym(*plant_);
  ASSERT_NO_THROW(plant_sym.get_force_element(spring_damper_->index()));
  EXPECT_EQ(plant_sym.get_force_element(spring_damper_->index()).index(),
            spring_damper_->index());
}

}  // namespace
}  // namespace multibody
}  // namespace drake
