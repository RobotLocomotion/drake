#include "drake/multibody/fem/fem_state.h"

#include <gtest/gtest.h>

#include "drake/multibody/fem/fem_state_manager.h"

namespace drake {
namespace multibody {
namespace fem {
namespace {

using Eigen::VectorXd;
constexpr int kNumElements = 2;
constexpr int kNumDofs = 12;

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
struct Data {
  double val{0.0};
};

template <typename T>
class FemStateTest : public ::testing::Test {
 protected:
  void SetUp() {
    fem_state_manager_ =
        std::make_unique<internal::FemStateManager<T>>(q<T>(), v<T>(), a<T>());

    auto calc_element_data =
        std::function<void(const VectorX<T>&, const VectorX<T>&,
                           const VectorX<T>&, std::vector<Data>*)>{
            [](const VectorX<T>& q, const VectorX<T>&, const VectorX<T>&,
               std::vector<Data>* element_data) {
              for (int i = 0; i < static_cast<int>(element_data->size()); ++i) {
                (*element_data)[i].val = ExtractDoubleOrThrow(q(i));
              }
            }};
    fem_state_manager_->DeclareElementData(kNumElements,
                                           std::move(calc_element_data));
  }

  std::unique_ptr<internal::FemStateManager<T>> fem_state_manager_;
};

using NonSymbolicScalars = ::testing::Types<double, AutoDiffXd>;
TYPED_TEST_SUITE(FemStateTest, NonSymbolicScalars);

TYPED_TEST(FemStateTest, GetStates) {
  using T = TypeParam;
  const FemState<T> state(this->fem_state_manager_.get());
  EXPECT_EQ(state.num_dofs(), kNumDofs);
  EXPECT_EQ(state.GetPositions(), q<T>());
  EXPECT_EQ(state.GetVelocities(), v<T>());
  EXPECT_EQ(state.GetAccelerations(), a<T>());
}

TYPED_TEST(FemStateTest, SetStates) {
  using T = TypeParam;
  FemState<T> state(this->fem_state_manager_.get());
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

TYPED_TEST(FemStateTest, ElementData) {
  using T = TypeParam;
  const FemState<T> state(this->fem_state_manager_.get());
  const VectorX<T> positions = q<T>();
  for (FemElementIndex i(0); i < kNumElements; ++i) {
    EXPECT_EQ(state.template EvalElementData<Data>(i).val,
              ExtractDoubleOrThrow(positions(i)));
    /* Verify that an exception is thrown if the wrong data type is requested.
     */
    EXPECT_THROW(state.template EvalElementData<double>(i), std::exception);
  }
  /* Verify that an exception is thrown if the element index is invalid. */
  const FemElementIndex invalid_index(kNumElements);
  EXPECT_THROW(state.template EvalElementData<Data>(invalid_index),
               std::exception);
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
