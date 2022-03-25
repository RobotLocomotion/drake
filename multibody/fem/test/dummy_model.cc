#include "drake/multibody/fem/test/dummy_model.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

DummyModel::DummyBuilder::DummyBuilder(DummyModel* model)
    : FemModel<T>::Builder(model), model_(model) {
  DRAKE_DEMAND(model_ != nullptr);
}

void DummyModel::DummyBuilder::DoBuild() {
  const FemNodeIndex num_existing_nodes(model_->num_nodes());
  for (auto& e : elements_) {
    e.OffsetNodeIndex(num_existing_nodes);
  }
  model_->AddElements(&elements_);
  elements_.clear();
  const int num_old_dofs = model_->reference_positions_.size();
  const int num_new_dofs = reference_positions_.size();
  model_->reference_positions_.conservativeResize(num_old_dofs + num_new_dofs);
  model_->reference_positions_.tail(num_new_dofs) =
      std::move(reference_positions_);
}

void DummyModel::DummyBuilder::AddTwoElementsWithSharedNodes() {
  ThrowIfBuilt();
  const std::array<FemNodeIndex, Traits::num_nodes> kNodeIndices0 = {
      FemNodeIndex(0), FemNodeIndex(1), FemNodeIndex(2), FemNodeIndex(3)};
  const std::array<FemNodeIndex, Traits::num_nodes> kNodeIndices1 = {
      FemNodeIndex(2), FemNodeIndex(3), FemNodeIndex(4), FemNodeIndex(5)};
  const Traits::ConstitutiveModel kConstitutiveModel(kYoungsModulus,
                                                     kPoissonsRatio);
  const DampingModel<T> kDampingModel(kMassDamping, kStiffnessDamping);
  DummyElement element0(kNodeIndices0, kConstitutiveModel, kDampingModel);
  DummyElement element1(kNodeIndices1, kConstitutiveModel, kDampingModel);

  const int num_existing_nodes = reference_positions_.size() / 3;
  element0.OffsetNodeIndex(FemNodeIndex(num_existing_nodes));
  element1.OffsetNodeIndex(FemNodeIndex(num_existing_nodes));

  elements_.emplace_back(element0);
  elements_.emplace_back(element1);
  constexpr int num_new_nodes = 6;
  VectorX<T> new_reference_positions(3 * num_new_nodes);
  for (int a = 0; a < num_new_nodes; ++a) {
    new_reference_positions.segment<3>(3 * a) = DummyModel::dummy_position();
  }
  reference_positions_.conservativeResize(reference_positions_.size() +
                                          3 * num_new_nodes);
  reference_positions_.tail<3 * num_new_nodes>() =
      std::move(new_reference_positions);
}

void DummyModel::DummyBuilder::AddElementWithDistinctNodes() {
  ThrowIfBuilt();
  const std::array<FemNodeIndex, Traits::num_nodes> kNodeIndices0 = {
      FemNodeIndex(0), FemNodeIndex(1), FemNodeIndex(2), FemNodeIndex(3)};
  const Traits::ConstitutiveModel kConstitutiveModel(kYoungsModulus,
                                                     kPoissonsRatio);
  const DampingModel<T> kDampingModel(kMassDamping, kStiffnessDamping);
  DummyElement element0(kNodeIndices0, kConstitutiveModel, kDampingModel);

  const int num_existing_nodes = reference_positions_.size() / 3;
  element0.OffsetNodeIndex(FemNodeIndex(num_existing_nodes));

  elements_.emplace_back(element0);
  constexpr int num_new_nodes = 4;
  VectorX<T> new_reference_positions(3 * num_new_nodes);
  for (int a = 0; a < num_new_nodes; ++a) {
    new_reference_positions.segment<3>(3 * a) = DummyModel::dummy_position();
  }
  reference_positions_.conservativeResize(reference_positions_.size() +
                                          3 * num_new_nodes);
  reference_positions_.tail<3 * num_new_nodes>() = new_reference_positions;
}

VectorX<DummyModel::T> DummyModel::MakeReferencePositions() const {
  return reference_positions_;
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
