#include "drake/multibody/fem/test/dummy_model.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

DummyModel::DummyModel() {
  const FemElementIndex kElementIndex0 = FemElementIndex(0);
  const FemElementIndex kElementIndex1 = FemElementIndex(1);
  const std::array<FemNodeIndex, Traits::num_nodes> kNodeIndices0 = {
      FemNodeIndex(0), FemNodeIndex(1), FemNodeIndex(2), FemNodeIndex(3)};
  const std::array<FemNodeIndex, Traits::num_nodes> kNodeIndices1 = {
      FemNodeIndex(2), FemNodeIndex(3), FemNodeIndex(4), FemNodeIndex(5)};
  const Traits::ConstitutiveModel kConstitutiveModel(5e4, 0.4);
  const DampingModel<T> kDampingModel(kMassDamping, kStiffnessDamping);

  DummyElement element0(kElementIndex0, kNodeIndices0, kConstitutiveModel,
                        kDampingModel);
  DummyElement element1(kElementIndex1, kNodeIndices1, kConstitutiveModel,
                        kDampingModel);
  AddElement(std::move(element0));
  AddElement(std::move(element1));
  num_nodes_ = 6;
  this->UpdateFemStateSystem();
}

void DummyModel::AddElementWithDistinctNodes(DummyElement&& element) {
  AddElement(std::move(element));
  num_nodes_ += DummyElement::Traits::num_nodes;
  this->UpdateFemStateSystem();
}

VectorX<DummyModel::T> DummyModel::MakeReferencePositions() const {
  /* The number of nodes is equal to the number of elements times the number of
   nodes per element since no element shares nodes. */
  VectorX<T> reference_positions(3 * num_nodes_);
  for (int i = 0; i < num_nodes_; ++i) {
    reference_positions.segment<3>(3 * i) = Vector3<T>(1, 2, 3);
  }
  return reference_positions;
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
