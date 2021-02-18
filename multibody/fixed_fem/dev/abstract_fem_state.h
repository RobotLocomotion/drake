#pragma once

namespace drake {
namespace multibody {
namespace fixed_fem {

/** An abstract base class for FemState that hides the templated Element type.
 See FemState for more information. */
template <typename T>
class AbstractFemState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AbstractFemState);
  AbstractFemState() = default;
  virtual ~AbstractFemState() = default;

  /** Calculates the norm of the state with the highest order. */
  virtual T HighestOrderStateNorm() const = 0;

  virtual int num_generalized_positions() const = 0;

  // TODO(xuchenhan-tri): Expose more public methods in FemState in
  //  AbstractFemState as needed.
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
