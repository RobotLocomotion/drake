#include "drake/multibody/fem/fem_element.h"

#include <gtest/gtest.h>

#include "drake/multibody/fem/test/dummy_element.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;

using DummyElementTraits = FemElementTraits<DummyElement>;
using T = DummyElementTraits::T;
using Data = DummyElementTraits::Data;
constexpr int kNumNodes = DummyElementTraits::num_nodes;
const std::array<FemNodeIndex, kNumNodes> kNodeIndices = {
    {FemNodeIndex(0), FemNodeIndex(1), FemNodeIndex(3), FemNodeIndex(2)}};
const DummyElementTraits::ConstitutiveModel kConstitutiveModel(5e4, 0.4);
const DampingModel<T> kDampingModel(0.01, 0.02);
constexpr int kNumDofs = DummyElementTraits::num_dofs;

class FemElementTest : public ::testing::Test {
 protected:
  /* Default values for the state. */
  static VectorX<T> q() {
    Vector<T, kNumDofs> q;
    q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2;
    return q;
  }
  static VectorX<T> v() {
    Vector<T, kNumDofs> v;
    v << 1.1, 1.2, 2.3, 2.4, 2.5, 2.6, 2.7, 1.8, 1.9, 2.0, 2.1, 2.2;
    return v;
  }
  static VectorX<T> a() {
    Vector<T, kNumDofs> a;
    a << 2.1, 2.2, 3.3, 3.4, 3.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2;
    return a;
  }

  void SetUp() override {
    fem_state_system_ =
        std::make_unique<internal::FemStateSystem<T>>(q(), v(), a());
    /* Set up element data. */
    std::function<void(const systems::Context<T>&, std::vector<Data>*)>
        calc_element_data = [this](const systems::Context<T>& context,
                                   std::vector<Data>* element_data) {
          /* There's only one element in the system. */
          element_data->resize(1);
          const FemState<T> fem_state(fem_state_system_.get(), &context);
          (*element_data)[0] = this->element_.ComputeData(fem_state);
        };

    cache_index_ =
        fem_state_system_
            ->DeclareCacheEntry("dummy element data",
                                systems::ValueProducer(calc_element_data))
            .cache_index();

    fem_state_ = std::make_unique<FemState<T>>(fem_state_system_.get());
  }

  /* Evaluates the element data of the element under test. */
  const Data& EvalElementData() const {
    const std::vector<Data>& element_data =
        fem_state_->EvalElementData<Data>(cache_index_);
    DRAKE_DEMAND(element_data.size() == 1);
    return element_data[0];
  }

  std::unique_ptr<internal::FemStateSystem<T>> fem_state_system_;
  std::unique_ptr<FemState<T>> fem_state_;
  /* FemElement under test. */
  DummyElement element_{kNodeIndices, kConstitutiveModel, kDampingModel};
  systems::CacheIndex cache_index_;
};

TEST_F(FemElementTest, Constructor) {
  EXPECT_EQ(element_.node_indices(), kNodeIndices);
}

/* Tests that the element data logic is correctly executed through
 `ComputeData`. */
TEST_F(FemElementTest, ElementData) {
  /* We know that dummy element's data is computed as the sum of the last
   entries in the states. */
  const Data& data = EvalElementData();
  EXPECT_EQ(data.value,
            q()(kNumDofs - 1) + v()(kNumDofs - 1) + a()(kNumDofs - 1));
}

TEST_F(FemElementTest, ExtractElementDofs) {
  VectorXd expected_element_q(kNumDofs);
  for (int i = 0; i < kNumNodes; ++i) {
    expected_element_q.segment<3>(3 * i) = q().segment<3>(3 * kNodeIndices[i]);
  }
  EXPECT_EQ(DummyElement::ExtractElementDofs(kNodeIndices, q()),
            expected_element_q);
}

/* The following tests confirm that CalcInverseDynamics(),
 AddScaledStiffnessMatrix, AddScaledDampingMatrix(), AddScaledMassMatrix(),
 correctly invoke their DoCalc and DoAdd counterparts. We confirm this with a
 custom subclass of FemElement whose implementation returns/adds a specific
 value. */
TEST_F(FemElementTest, InverseDynamics) {
  Vector<T, DummyElementTraits::num_dofs> external_force;
  element_.CalcInverseDynamics(EvalElementData(), &external_force);
  const Vector<T, kNumDofs> zero_vector = Vector<T, kNumDofs>::Zero();
  EXPECT_EQ(external_force, zero_vector);

  fem_state_->SetPositions(zero_vector);
  fem_state_->SetVelocities(zero_vector);
  fem_state_->SetAccelerations(zero_vector);
  element_.CalcInverseDynamics(EvalElementData(), &external_force);
  EXPECT_EQ(external_force, element_.inverse_dynamics_force());
}

TEST_F(FemElementTest, StiffnessMatrix) {
  Eigen::Matrix<T, DummyElementTraits::num_dofs, DummyElementTraits::num_dofs>
      K;
  K.setZero();
  const T scale = 3.14;
  element_.AddScaledStiffnessMatrix(EvalElementData(), scale, &K);
  EXPECT_EQ(K, scale * element_.stiffness_matrix());
}

TEST_F(FemElementTest, DampingMatrix) {
  Eigen::Matrix<T, DummyElementTraits::num_dofs, DummyElementTraits::num_dofs>
      D;
  D.setZero();
  const T scale = 3.14;
  element_.AddScaledDampingMatrix(EvalElementData(), scale, &D);

  Eigen::Matrix<T, DummyElementTraits::num_dofs, DummyElementTraits::num_dofs>
      expected_D;
  expected_D.setZero();
  element_.AddScaledMassMatrix(
      EvalElementData(), scale * kDampingModel.mass_coeff_alpha(), &expected_D);
  element_.AddScaledStiffnessMatrix(
      EvalElementData(), scale * kDampingModel.stiffness_coeff_beta(),
      &expected_D);
  EXPECT_EQ(D, expected_D);
}

TEST_F(FemElementTest, MassMatrix) {
  Eigen::Matrix<T, DummyElementTraits::num_dofs, DummyElementTraits::num_dofs>
      M;
  M.setZero();
  const T scale = 3.14;
  element_.AddScaledMassMatrix(EvalElementData(), scale, &M);
  EXPECT_EQ(M, scale * element_.mass_matrix());
}

TEST_F(FemElementTest, AddScaledGravityForce) {
  const Vector3d gravity_vector(1, 2, 3);
  VectorXd gravity_vector_all_nodes(kNumDofs);
  for (int i = 0; i < kNumNodes; ++i) {
    gravity_vector_all_nodes.segment<3>(3 * i) = gravity_vector;
  }
  VectorXd scaled_gravity_force = VectorXd::Zero(kNumDofs);
  const double scale = 2.0;
  /* The only external force in FemElement is gravity. */
  const Data& data = EvalElementData();
  /* We explicitly test the gravity force calculation implementation in
   FemElement (instead of the specific one in DummyElement). */
  const FemElement<DummyElement>& base_fem_element = element_;
  base_fem_element.AddScaledGravityForce(data, scale, gravity_vector,
                                         &scaled_gravity_force);
  const VectorXd expected_scaled_gravity_force =
      scale * element_.mass_matrix() * gravity_vector_all_nodes;
  EXPECT_EQ(expected_scaled_gravity_force, scaled_gravity_force);
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
