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
    explicit DummyBuilder(DummyModel* model);

    virtual ~DummyBuilder() = default;

    /* Adds a new element to the model where all nodes in the new element are
     not shared by any existing element in the model. */
    void AddElementWithDistinctNodes();

    /* Adds two new elements to the model. The two added elements share two
     commom nodes. */
    void AddTwoElementWithSharedNodes();

   private:
    void DoBuild() final;

    DummyModel* model_{};
    std::vector<DummyElement> elements_{};
    VectorX<T> reference_positions_;
  };

  /* Constructs a dummy FEM model with two dummy FEM elements. */
  DummyModel();

  ~DummyModel() = default;

  /* Returns a dummy position that's used for the reference positions for all
   nodes in the dummy model. */
  static Vector3<T> dummy_position() { return Vector3<T>(1, 2, 3); }

  std::unique_ptr<DummyBuilder> MakeBuilder() {
    return std::make_unique<DummyBuilder>(this);
  }

  VectorX<T> MakeReferencePositions() const final;

 private:
  VectorX<T> reference_positions_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
