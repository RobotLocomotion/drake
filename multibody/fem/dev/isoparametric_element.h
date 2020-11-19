#pragma once

#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fem {

/** IsoparametricElement is a class that helps evaluate shape functions
 and their derivatives at prescribed locations. The shape function
 `S` located at a vertex `a` maps parent domain to a scalar. The reference
 position `X` as well as `u`, the function we wish to interpolate, are
 interpolated from the shape function, i.e.:

 <pre>
     X(ξ) = Sₐ(ξ)Xₐ, and
     u(ξ) = Sₐ(ξ)uₐ,
 </pre>

 where ξ ∈ ℝᵈ and d is the natural dimension (dimension of the parent
 domain), which may be different from the dimensions of X and u (e.g. 2D
 membrane or shell element in 3D dynamics simulation). The constructor
 for this class takes in a vector of locations at which we may
 evaluate and/or interpolate various quantities. If you need to evaluate
 and/or interpolate at other locations, construct another instance of
 IsoparametricElement and pass the new locations into the constructor.
 @tparam_nonsymbolic_scalar T.
 @tparam NaturalDim The dimension of the parent domain. */
/* TODO(xuchenhan-tri): Consider templatize on spatial dimension as well. */
template <typename T, int NaturalDim>
class IsoparametricElement {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IsoparametricElement);

  using VectorD = Eigen::Matrix<T, NaturalDim, 1>;

  /** The dimension of the parent domain. */
  static constexpr int kNaturalDim = NaturalDim;

  /** Constructs an isoparametric element that performs calculations at the
   locations specified by the input `locations`. */
  explicit IsoparametricElement(std::vector<VectorD> locations)
      : locations_(std::move(locations)) {}

  virtual ~IsoparametricElement() = default;

  /** The number of nodes in the element. E.g. 3 for linear triangles, 9 for
   quadratic quadrilaterals and 4 for linear tetrahedrons. */
  virtual int num_nodes() const = 0;

  /** The number of locations to evaluate various quantities in the
   element. */
  int num_sample_locations() const { return locations_.size(); }

  /** Returns a vector of locations ξ in the parent domain at which this class
  evaluates elemental quantities, as provided at construction. */
  const std::vector<VectorD>& locations() const { return locations_; }

  /** Computes the shape function vector
      S(ξ) = [S₀(ξ); S₁(ξ); ... Sₐ(ξ); ... Sₙ₋₁(ξ)]
   at each ξ in the parent domain provided at construction. The a-th component
   of S(ξ) corresponds to the shape function Sₐ(ξ) for node a.
   @returns S Vector of size equal to 'num_sample_locations()`. The q-th entry
   of the output contains vector S(ξ), of size num_nodes(), evaluated at the
   q-th ξ in the parent domain provided at construction. */
  virtual const std::vector<VectorX<T>>& CalcShapeFunctions() const = 0;

  /** Computes dS/dξ, a matrix of size `num_nodes()` by `NaturalDim` evaluated
   at each ξ in the parent domain provided at construction.
   @returns dSdxi The gradient of the shape function with respect to the
   parent coordinates evaluated at each ξ in the parent domain provided at
   construction. dSdxi is a vector of size `num_sample_locations()`. The q-th
   entry contains dS/dξ evaluated at the q-th ξ in the parent domain provided at
   construction. */
  virtual const std::vector<MatrixX<T>>& CalcGradientInParentCoordinates()
      const = 0;

  // TODO(xuchenhan-tri): Implement CalcGradientInSpatialCoordinates().

  /** Computes dx/dξ, a matrix of size `nsd` by `NaturalDim` at each ξ in the
   parent domain provided at construction. `nsd` is the number of spatial
   dimensions, defined by the number of rows in `xa`.
   @param xa Spatial coordinates for each element node.
   @returns jacobian The Jacobian of the spatial coordinates with respect to
   parent coordinates evaluated at each ξ in the parent domain provided at
   construction. 'jacobian' is represented by a vector of size
   `num_sample_locations()`. The q-th entry contains the dx/dξ evaluated at the
   q-th ξ in the parent domain provided at construction.
   @pre `xa` must have `num_nodes()` columns. */
  std::vector<MatrixX<T>> CalcJacobian(
      const Eigen::Ref<const MatrixX<T>>& xa) const;

  /** Computes dξ/dx, a matrix of size `NaturalDim` by `nsd`, at each ξ in the
   parent domain provided at construction. `nsd` is the "number of spatial
   dimensions, defined by the number of rows in `xa`.
   @param xa Spatial coordinates for each element node.
   @returns The gradient of the shape function with respect to the spatial
   coordinates evaluated at each ξ in the parent domain provided at
   construction represented by a vector
   of size `num_quad()`. The q-th entry contains dξ/dx evaluated at the q-th ξ
   in the parent domain provided at construction.
   @pre `xa` must have `num_nodes()` columns.
   @see CalcJacobian().  */
  std::vector<MatrixX<T>> CalcJacobianInverse(
      const Eigen::Ref<const MatrixX<T>>& xa) const;

  /** Preferred signature for computing dξ/dx when the Jacobians are
   available so as to avoid recomputing the Jacobians.
   @param jacobian A vector of size `num_sample_locations()` with the q-th entry
   containing the element Jacobian evaluated at the q-th ξ
   in the parent domain provided at construction.
   @pre `jacobian.size()` must be equal to `num_sample_locations()`. Each entry
   in `jacobian` must have `NaturalDim` columns.
   @see CalcJacobian(). */
  std::vector<MatrixX<T>> CalcJacobianInverse(
      const std::vector<MatrixX<T>>& jacobian) const;

  /** Interpolates scalar nodal values `ua` into u(ξ) at each ξ
   in the parent domain provided at construction.
   @param ua The value of scalar function u at each element node.
   @pre `ua` must be a vector of size num_nodes(). */
  std::vector<T> InterpolateScalar(
      const Eigen::Ref<const VectorX<T>>& ua) const;

  /** Interpolates vector nodal values `ua` into u(ξ) at each ξ
   in the parent domain provided at construction.
   @param ua The value of vector function u at each element node.
   @pre `ua` must have size nsd by num_nodes(). */
  std::vector<VectorX<T>> InterpolateVector(
      const Eigen::Ref<const MatrixX<T>>& ua) const;

 private:
  // The locations at which to evaluate various quantities of this
  // element.
  std::vector<VectorD> locations_;
};

template <class QuadratureType, typename T = void, int NaturalDimension = 0>
struct is_isoparametric_element {
  static constexpr bool value = false;
};

}  // namespace fem
}  // namespace multibody
}  // namespace drake
