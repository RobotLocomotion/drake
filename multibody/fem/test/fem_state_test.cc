#include "drake/multibody/fem/fem_state.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/fem/fem_state_system.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace fem {
namespace {

using Eigen::VectorXd;
using multibody::internal::ForceDensityEvaluator;

constexpr int kNumElements = 2;
constexpr int kNumNodes = 4;
constexpr int kNumDofs = 3 * kNumNodes;

/* Dummy values for the states. */
template <typename T>
VectorX<T> q() {
  Vector<T, kNumDofs> q;
  q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2;
  return q;
}
template <typename T>
VectorX<T> v() {
  Vector<T, kNumDofs> v;
  v << 1.1, 1.2, 2.3, 2.4, 2.5, 2.6, 2.7, 1.8, 1.9, 2.0, 2.1, 2.2;
  return v;
}
template <typename T>
VectorX<T> a() {
  Vector<T, kNumDofs> a;
  a << 2.1, 2.2, 3.3, 3.4, 3.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2;
  return a;
}

/* Dummy per-element data. */
template <typename T>
struct Data {
  T val{0.0};
};

template <typename T>
class FemStateTest : public ::testing::Test {
 protected:
  void SetUp() {
    fem_state_system_ =
        std::make_unique<internal::FemStateSystem<T>>(q<T>(), v<T>(), a<T>());

    std::function<void(const systems::Context<T>&, std::vector<Data<T>>*)>
        calc_element_data = [this](const systems::Context<T>& context,
                                   std::vector<Data<T>>* element_data) {
          const FemState<T> fem_state(fem_state_system_.get(), &context);
          const VectorX<T>& q = fem_state.GetPositions();
          element_data->resize(kNumElements);
          for (int i = 0; i < static_cast<int>(element_data->size()); ++i) {
            (*element_data)[i].val = q(i);
          }
        };

    cache_index_ =
        fem_state_system_
            ->DeclareCacheEntry("dummy_data",
                                systems::ValueProducer(calc_element_data),
                                {fem_state_system_->discrete_state_ticket(
                                    fem_state_system_->fem_position_index())})
            .cache_index();

    /* Set up a few external forces and their evaluators for testing. */
    const T mass_density = 2.7;
    const Vector3<T> g1(0, 0, -9.81);
    const Vector3<T> g2(0, 0, -10.0);
    external_forces_.emplace_back(
        std::make_unique<GravityForceField<T>>(g1, mass_density));
    external_forces_.emplace_back(
        std::make_unique<GravityForceField<T>>(g2, mass_density));
    MultibodyPlant<T> plant(0.01);
    plant.Finalize();
    plant_context_ = plant.CreateDefaultContext();
    force_density_evaluators_.emplace_back(external_forces_[0].get(),
                                           plant_context_.get());
    force_density_evaluators_.emplace_back(external_forces_[1].get(),
                                           plant_context_.get());
  }

  /* Checks that `clone` is a deep copy of `state`. */
  void VerifyIsClone(const FemState<T>& state, const FemState<T>& clone) {
    EXPECT_EQ(state.num_dofs(), clone.num_dofs());
    EXPECT_EQ(state.GetPositions(), clone.GetPositions());
    EXPECT_EQ(state.GetVelocities(), clone.GetVelocities());
    EXPECT_EQ(state.GetAccelerations(), clone.GetAccelerations());
    EXPECT_EQ(state.GetExternalForces(), clone.GetExternalForces());

    const std::vector<Data<T>>& element_data =
        state.template EvalElementData<Data<T>>(cache_index_);
    const std::vector<Data<T>>& cloned_data =
        clone.template EvalElementData<Data<T>>(cache_index_);
    ASSERT_EQ(element_data.size(), cloned_data.size());
    for (int i = 0; i < static_cast<int>(element_data.size()); ++i) {
      EXPECT_EQ(element_data[i].val, cloned_data[i].val);
    }
  }

  std::unique_ptr<internal::FemStateSystem<T>> fem_state_system_;
  systems::CacheIndex cache_index_;
  std::vector<ForceDensityEvaluator<T>> force_density_evaluators_;
  std::vector<std::unique_ptr<ForceDensityField<T>>> external_forces_;
  std::unique_ptr<systems::Context<T>> plant_context_;
};

using NonSymbolicScalars = ::testing::Types<double, AutoDiffXd>;
TYPED_TEST_SUITE(FemStateTest, NonSymbolicScalars);

TYPED_TEST(FemStateTest, SharedFemState) {
  using T = TypeParam;
  auto context = this->fem_state_system_->CreateDefaultContext();
  FemState<T> state(this->fem_state_system_.get(), context.get());
  EXPECT_EQ(state.num_dofs(), kNumDofs);
  EXPECT_EQ(state.num_nodes(), kNumNodes);
  EXPECT_EQ(state.GetPositions(), q<T>());
  EXPECT_EQ(state.GetVelocities(), v<T>());
  EXPECT_EQ(state.GetAccelerations(), a<T>());
  DRAKE_EXPECT_THROWS_MESSAGE(state.SetPositions(-1.23 * q<T>()),
                              "Trying to mutate a shared FemState.");
  DRAKE_EXPECT_THROWS_MESSAGE(state.SetVelocities(3.14 * v<T>()),
                              "Trying to mutate a shared FemState.");
  DRAKE_EXPECT_THROWS_MESSAGE(state.SetAccelerations(-1.29 * a<T>()),
                              "Trying to mutate a shared FemState.");
}

TYPED_TEST(FemStateTest, GetStates) {
  using T = TypeParam;
  const FemState<T> state(this->fem_state_system_.get());
  EXPECT_EQ(state.num_dofs(), kNumDofs);
  EXPECT_EQ(state.num_nodes(), kNumNodes);
  EXPECT_EQ(state.GetPositions(), q<T>());
  EXPECT_EQ(state.GetVelocities(), v<T>());
  EXPECT_EQ(state.GetAccelerations(), a<T>());
}

TYPED_TEST(FemStateTest, SetStates) {
  using T = TypeParam;
  FemState<T> state(this->fem_state_system_.get());
  state.SetPositions(-1.23 * q<T>());
  state.SetVelocities(3.14 * v<T>());
  state.SetAccelerations(-1.29 * a<T>());
  EXPECT_EQ(state.GetPositions(), -1.23 * q<T>());
  EXPECT_EQ(state.GetVelocities(), 3.14 * v<T>());
  EXPECT_EQ(state.GetAccelerations(), -1.29 * a<T>());
  /* Setting values with incompatible sizes should throw. */
  EXPECT_THROW(state.SetPositions(VectorX<T>::Constant(1, 1.0)),
               std::exception);
  EXPECT_THROW(state.SetVelocities(VectorX<T>::Constant(1, 1.0)),
               std::exception);
  EXPECT_THROW(state.SetAccelerations(VectorX<T>::Constant(1, 1.0)),
               std::exception);
}

TYPED_TEST(FemStateTest, SetExternalForces) {
  using T = TypeParam;
  FemState<T> state(this->fem_state_system_.get());
  state.SetExternalForces(this->force_density_evaluators_);
  EXPECT_EQ(state.GetExternalForces(), this->force_density_evaluators_);
}

TYPED_TEST(FemStateTest, ElementData) {
  using T = TypeParam;
  const FemState<T> state(this->fem_state_system_.get());
  const std::vector<Data<T>>& element_data =
      state.template EvalElementData<Data<T>>(this->cache_index_);
  ASSERT_EQ(element_data.size(), kNumElements);
  const VectorX<T> positions = q<T>();
  for (int i = 0; i < kNumElements; ++i) {
    EXPECT_EQ(element_data[i].val, positions(i));
  }
  /* Verify that an exception is thrown if the wrong data type is requested.
   */
  EXPECT_THROW(state.template EvalElementData<double>(this->cache_index_),
               std::exception);
}

TYPED_TEST(FemStateTest, Clone) {
  using T = TypeParam;
  /* Owned context version. */
  {
    FemState<T> state(this->fem_state_system_.get());
    state.SetExternalForces(this->force_density_evaluators_);
    std::unique_ptr<FemState<T>> clone = state.Clone();
    this->VerifyIsClone(state, *clone);
  }
  /* Shared context version. */
  {
    auto context = this->fem_state_system_->CreateDefaultContext();
    FemState<T> state(this->fem_state_system_.get(), context.get());
    state.SetExternalForces(this->force_density_evaluators_);
    std::unique_ptr<FemState<T>> clone = state.Clone();
    this->VerifyIsClone(state, *clone);
  }
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
