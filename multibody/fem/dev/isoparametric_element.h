#pragma once

#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace fem {

/** IsoparametricElement is a class that helps evaluate shape functions
    and their derivatives at quadrature locations. The shape function
    `S` located at a vertex `a` maps parent domain to a scalar. The reference
    position `X` as well as `u`, the function we wish to interpolate, are
    interpolated from the shape function, i,e:

    <pre>
        X(ξ) = Sₐ(ξ)Xₐ, and
        u(ξ) = Sₐ(ξ)uₐ,
    </pre>

    where ξ ∈ ℝᵈ and d is the natural dimension (dimension of the parent
    domain), which may be different from the dimensions of X and u (e.g. 2D
    membrane or shell element in 3D dynamics simulation). The constructor
    for this class takes in a vector of quadrature locations at which we may
    evaluate and/or interpolate various quantities. If you need to evaluate
    and/or interpolate at other locations, construct another instance of
    IsoparametricElement and pass the new locations into the constructor.
    @tparam T The scalar type for ξ, X and u.
    @tparam NaturalDim The dimension of the parent domain.
*/
template <typename T, int NaturalDim>
class IsoparametricElement {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IsoparametricElement);

  using VectorD = Eigen::Matrix<T, NaturalDim, 1>;

  /** Constructs an isoparametric element that performs calculations at the
      locations specified by the input `quadrature_locations`. */
  explicit IsoparametricElement(std::vector<VectorD> quadrature_locations)
      : quadrature_locations_(std::move(quadrature_locations)) {}

  virtual ~IsoparametricElement() = default;

  /** The number of nodes in the element. E.g. 3 for linear triangles, 9 for
      quadratic quadrilaterals and 4 for linear tetrahedrons. */
  virtual int num_nodes() const = 0;

  /** The number of quadrature locations to evaluate various quantities in the
      element. */
  int num_quads() const { return quadrature_locations_.size(); }

  const std::vector<VectorD>& quadrature_locations() const {
    return quadrature_locations_;
  }

  /** Computes the shape function vector
         S(ξ) = [S₀(ξ); S₁(ξ); ... Sₐ(ξ); ... Sₙ₋₁(ξ)]
      at each point specified by `quadrature_locations()`. The a-th
      component of S(ξ) corresponds to the shape function Sₐ(ξ) for node a.
      @returns S Vector of size equal to 'num_quads()`. The q-th entry of
      the output contains vector S(ξ), of size num_nodes(), evaluated at the
      q-th quadrature location.
  */
  virtual const std::vector<VectorX<T>>& CalcShapeFunctions() const = 0;

  /** Computes dS/dξ, a matrix of size `num_nodes()` by `NaturalDim` evaluated
      at each quadrature location specified by `quadrature_locations()`.
      @returns dSdxi The gradient of the shape function with respect to the
      parent coordinates evaluated at each quadrature location. dSdxi is a
      vector of size `num_quads()`. The q-th entry contains dS/dξ evaluated at
      the q-th quadrature location.
  */
  virtual const std::vector<MatrixX<T>>& CalcGradientInParentCoordinates()
      const = 0;

  /** Computes dx/dξ, a matrix of size `nsd` by `NaturalDim` at each quadrature
      location specified by `quadrature_locations()`. `nsd` is the number of
      spatial dimensions, defined by the number of rows in `xa`.
      @param xa Spatial coordinates for each element node.
      @returns jacobian The Jacobian of the spatial coordinates with respect to
      parent coordinates evaluated at each quadrature location. 'jacobian' is
      represented by a vector of size `num_quads()`. The q-th entry
      contains the dx/dξ evaluated at the q-th quadrature location.
      @pre `xa` must have `num_nodes()` columns.
   */
  std::vector<MatrixX<T>> CalcElementJacobian(
      const Eigen::Ref<const MatrixX<T>>& xa) const;

  /** Computes dξ/dx, a matrix of size `NaturalDim` by `nsd`, at each quadrature
      point specified by `quadrature_locations()`. `nsd` is the "number of
      spatial dimensions, defined by the number of rows in `xa`.
      @param xa Spatial coordinates for each element node.
      @returns The gradient of the shape function with respect to the spatial
      coordinates evaluated at each quadrature location represented by a vector
      of size `num_quad()`. The q-th entry contains dξ/dx evaluated at the q-th
      quadrature location.
      @pre `xa` must have `num_nodes()` columns.
  */
  std::vector<MatrixX<T>> CalcElementJacobianInverse(
      const Eigen::Ref<const MatrixX<T>>& xa) const;

  /** Alternative signature for computing dξ/dx when the element Jacobians are
      available.
      @param jacobian A vector of size `num_quads()` with the q-th entry
      containing the element Jacobian evaluated at the q-th quadrature point.
      @pre `jacobian.size()` must be equal to `num_quads()`. Each entry in
     `jacobian` must have `NaturalDim` columns.
   */
  std::vector<MatrixX<T>> CalcElementJacobianInverse(
      const std::vector<MatrixX<T>>& jacobian) const;

  /** Interpolates scalar nodal values `ua` into u(ξ) at each quadrature point.
      @param ua The value of scalar function u at each element node.
      @pre `ua` must be a vector of size num_nodes().
  */
  std::vector<T> InterpolateScalar(
      const Eigen::Ref<const VectorX<T>>& ua) const;

  /** Interpolates vector nodal values `ua` into u(ξ) at each quadrature point.
      @param ua The value of vector function u at each element node.
      @pre `ua` must be a vector of size num_nodes().
  */
  std::vector<VectorX<T>> InterpolateVector(
      const Eigen::Ref<const MatrixX<T>>& ua) const;

 private:
  // The quadrature locations at which to evaluate various quantities of this
  // element.
  std::vector<VectorD> quadrature_locations_;
};
}  // namespace fem
}  // namespace drake
