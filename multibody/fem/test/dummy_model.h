#pragma once

#include <array>
#include <utility>
#include <vector>

#include "drake/multibody/fem/fem_model_impl.h"
#include "drake/multibody/fem/test/dummy_element.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

class DummyModel final : public FemModelImpl<DummyElement> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyModel);
  using Traits = FemElementTraits<DummyElement>;
  using T = Traits::T;
  static constexpr double kMassDamping = 0.01;
  static constexpr double kStiffnessDamping = 0.02;

  /* Constructs a dummy FEM model with two dummy FEM elements. */
  DummyModel();

  ~DummyModel() = default;

  /* Adds a new element to the model and updates the model. It is required that
   all the nodes in the new element does not already exist in this FEM model. */
  void AddElementWithDistinctNodes(DummyElement&& element);

  /* Returns a dummy reference position where all nodes are positioned at
   (1, 2, 3). */
  VectorX<T> MakeReferencePositions() const final;

 private:
  /* Number of nodes in the FEM model. */
  int num_nodes_{0};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
