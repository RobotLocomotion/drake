#pragma once

#include "drake/multibody/fem/fem_model_impl.h"
#include "drake/multibody/fem/test/dummy_element.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace test {

/* A dummy FemModelImpl with a single DummyElement for testing purpose. */
class DummyModel final : public FemModelImpl<DummyElement> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyModel);

  using Traits = DummyElement::Traits;
  using T = Traits::T;
  using ConstitutiveModel = typename Traits::ConstitutiveModel;
  static constexpr int kNumDofs = Traits::num_dofs;
  const T kYoungsModulus = 1e7;
  const T kPoissonRatio = 0.49;
  const T kMassDamping = 0.001;
  const T kStiffnessDamping = 0.02;

  /* Creates a dummy FEM model with a single element. */
  DummyModel() {
    const ElementIndex element_index(0);
    const std::array node_indices = {NodeIndex(0), NodeIndex(1), NodeIndex(2),
                                     NodeIndex(3)};
    const ConstitutiveModel constitutive_model(1e7, 0.49);
    const DampingModel<T> damping_model(kMassDamping, kStiffnessDamping);
    this->AddElement(element_index, node_indices, constitutive_model,
                     damping_model);
    increment_num_nodes(4);
  }

 private:
  /* Creates an all-zero FEM state for the dummy model. */
  FemStateImpl<DummyElement> DoMakeFemStateImpl() const final {
    return FemStateImpl<DummyElement>(VectorX<T>::Zero(kNumDofs),
                                      VectorX<T>::Zero(kNumDofs),
                                      VectorX<T>::Zero(kNumDofs));
  }
};

}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
