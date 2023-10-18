#include "drake/multibody/fem/test/dummy_model.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <bool is_linear>
DummyModel<is_linear>::DummyBuilder::DummyBuilder(DummyModel* model)
    : FemModel<T>::Builder(model), model_(model) {
  DRAKE_DEMAND(model_ != nullptr);
}

template <bool is_linear>
void DummyModel<is_linear>::DummyBuilder::DoBuild() {
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

template <bool is_linear>
void DummyModel<is_linear>::DummyBuilder::AddTwoElementsWithSharedNodes() {
  this->ThrowIfBuilt();
  const std::array<FemNodeIndex, Traits::num_nodes> kNodeIndices0 = {
      FemNodeIndex(0), FemNodeIndex(1), FemNodeIndex(2), FemNodeIndex(3)};
  const std::array<FemNodeIndex, Traits::num_nodes> kNodeIndices1 = {
      FemNodeIndex(2), FemNodeIndex(3), FemNodeIndex(4), FemNodeIndex(5)};
  const typename Traits::ConstitutiveModel kConstitutiveModel(kYoungsModulus,
                                                              kPoissonsRatio);
  const DampingModel<T> kDampingModel(kMassDamping, kStiffnessDamping);
  DummyElement<is_linear> element0(kNodeIndices0, kConstitutiveModel,
                                   kDampingModel);
  DummyElement<is_linear> element1(kNodeIndices1, kConstitutiveModel,
                                   kDampingModel);

  const int num_existing_nodes = reference_positions_.size() / 3;
  element0.OffsetNodeIndex(FemNodeIndex(num_existing_nodes));
  element1.OffsetNodeIndex(FemNodeIndex(num_existing_nodes));

  elements_.emplace_back(element0);
  elements_.emplace_back(element1);
  constexpr int num_new_nodes = 6;
  VectorX<T> new_reference_positions(3 * num_new_nodes);
  for (int a = 0; a < num_new_nodes; ++a) {
    new_reference_positions.template segment<3>(3 * a) =
        DummyModel::dummy_position();
  }
  reference_positions_.conservativeResize(reference_positions_.size() +
                                          3 * num_new_nodes);
  reference_positions_.template tail<3 * num_new_nodes>() =
      std::move(new_reference_positions);
}

template <bool is_linear>
void DummyModel<is_linear>::DummyBuilder::AddElementWithDistinctNodes() {
  this->ThrowIfBuilt();
  const std::array<FemNodeIndex, Traits::num_nodes> kNodeIndices0 = {
      FemNodeIndex(0), FemNodeIndex(1), FemNodeIndex(2), FemNodeIndex(3)};
  const typename Traits::ConstitutiveModel kConstitutiveModel(kYoungsModulus,
                                                              kPoissonsRatio);
  const DampingModel<T> kDampingModel(kMassDamping, kStiffnessDamping);
  DummyElement<is_linear> element0(kNodeIndices0, kConstitutiveModel,
                                   kDampingModel);

  const int num_existing_nodes = reference_positions_.size() / 3;
  element0.OffsetNodeIndex(FemNodeIndex(num_existing_nodes));

  elements_.emplace_back(element0);
  constexpr int num_new_nodes = 4;
  VectorX<T> new_reference_positions(3 * num_new_nodes);
  for (int a = 0; a < num_new_nodes; ++a) {
    new_reference_positions.template segment<3>(3 * a) =
        DummyModel<is_linear>::dummy_position();
  }
  reference_positions_.conservativeResize(reference_positions_.size() +
                                          3 * num_new_nodes);
  reference_positions_.template tail<3 * num_new_nodes>() =
      new_reference_positions;
}

template <bool is_linear>
VectorX<typename DummyModel<is_linear>::T>
DummyModel<is_linear>::MakeReferencePositions() const {
  return reference_positions_;
}

template class DummyModel<true>;
template class DummyModel<false>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
