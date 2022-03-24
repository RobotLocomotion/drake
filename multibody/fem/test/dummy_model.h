#pragma once

#include <array>
#include <memory>
#include <utility>
#include <vector>

#include "drake/multibody/fem/fem_model_impl.h"
#include "drake/multibody/fem/test/dummy_element.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* A dummy FEM model for testing purpose only. */
class DummyModel final : public FemModelImpl<DummyElement> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyModel);
  using Traits = FemElementTraits<DummyElement>;
  using T = Traits::T;
  static constexpr double kMassDamping = 0.01;
  static constexpr double kStiffnessDamping = 0.02;
  static constexpr double kYoungsModulus = 5e4;
  static constexpr double kPoissonsRatio = 0.4;

  class DummyBuilder : public FemModel<T>::Builder {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyBuilder);

    /* Constructs a new dummy builder that is associated with the given `model`.
     */
    explicit DummyBuilder(DummyModel* model);

    virtual ~DummyBuilder() = default;

    /* Specifies a new element in the model where all nodes in the new element
     are not shared by any existing element in the model. */
    void AddElementWithDistinctNodes();

    /* Specifies two new elements in the model. The two added elements share two
     common nodes. */
    void AddTwoElementsWithSharedNodes();

   private:
    void DoBuild() final;

    DummyModel* model_{nullptr};
    std::vector<DummyElement> elements_;
    VectorX<T> reference_positions_;
  };

  /* Constructs an empty DummyModel. */
  DummyModel() = default;

  ~DummyModel() = default;

  /* Returns a dummy position that's used for the reference positions for all
   nodes in the dummy model. */
  static Vector3<T> dummy_position() { return Vector3<T>(1, 2, 3); }

  VectorX<T> MakeReferencePositions() const final;

 private:
  /* The reference positions of all nodes in the model. */
  VectorX<T> reference_positions_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
