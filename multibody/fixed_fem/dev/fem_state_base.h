#pragma once

namespace drake {
namespace multibody {
namespace fixed_fem {

template <typename T>
class DirichletBoundaryCondition;

/** An abstract base class for FemState that hides the templated Element type.
 See FemState for more information. */
template <typename T>
class FemStateBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FemStateBase);
  FemStateBase() = default;
  virtual ~FemStateBase() = default;

  /** Calculates the norm of the state with the highest order. */
  virtual T HighestOrderStateNorm() const = 0;

  virtual int num_generalized_positions() const = 0;

  // TODO(xuchenhan-tri): Consider whether this method should belong in
  //  DirichletBoundaryCondition along with the methods that apply the BC to
  //  residuals and tangent matrices.
  /** Modifies `this` FEM state so that it complies with the given boundary
   conditions.
   @throw std::exception if the any of the indexes of the dofs under the
   boundary condition specified by the given DirichletBoundaryCondition does
   not exist in `this` FEM state`. */
  virtual void ApplyBoundaryConditions(
      const DirichletBoundaryCondition<T>& bc) = 0;

  // TODO(xuchenhan-tri): Expose more public methods in FemState in
  //  FemStateBase as needed.
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
