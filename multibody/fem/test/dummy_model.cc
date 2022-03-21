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
      FemNodeIndex(4), FemNodeIndex(5), FemNodeIndex(6), FemNodeIndex(7)};
  const Traits::ConstitutiveModel kConstitutiveModel(5e4, 0.4);
  const DampingModel<T> kDampingModel(kMassDamping, kStiffnessDamping);

  DummyElement element0(kElementIndex0, kNodeIndices0, kConstitutiveModel,
                        kDampingModel);
  DummyElement element1(kElementIndex1, kNodeIndices1, kConstitutiveModel,
                        kDampingModel);
  AddElementWithDistinctNodes(std::move(element0));
  AddElementWithDistinctNodes(std::move(element1));
}

void DummyModel::AddElementWithDistinctNodes(DummyElement&& element) {
  AddElement(std::move(element));
  this->UpdateFemStateSystem();
}

VectorX<DummyModel::T> DummyModel::MakeReferencePositions() const {
  /* The number of nodes is equal to the number of elements times the number of
   nodes per element since no element shares nodes. */
  const int num_nodes = num_elements() * Traits::num_nodes;
  VectorX<T> reference_positions(3 * num_nodes);
  for (int i = 0; i < num_nodes; ++i) {
    reference_positions.segment<3>(3 * i) = Vector3<T>(1, 2, 3);
  }
  return reference_positions;
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
