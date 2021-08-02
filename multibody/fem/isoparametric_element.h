#pragma once

#include <array>
#include <utility>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* IsoparametricElement is a class that evaluates shape functions
 and their derivatives at prescribed locations. The shape function
 `S` located at a vertex `a` maps the parent domain to a scalar. The reference
 position `X` as well as `u(X)`, an arbitrary function on the reference domain,
 can be interpolated from nodal values `Xₐ` and `uₐ` to any location in the
 parent domain using the shape function, i.e.:

     X(ξ) = Sₐ(ξ)Xₐ, and
     u(X(ξ)) = Sₐ(ξ)uₐ,

 where ξ ∈ ℝᵈ is in the parent domain and d is its dimension (the natural
 dimension), which may be different from the dimension of X (the spatial
 dimension). For example 2D membrane or shell element in 3D dynamics simulation
 has natural dimension 2 and spatial dimension 3. The constructor for this
 class takes in an std::array of locations at which we may evaluate and/or
 interpolate various quantities. If you need to evaluate and/or interpolate at
 other locations, construct another instance of IsoparametricElement and pass
 the new locations into the constructor.

 IsoparametricElement serves as the interface base class. Since shape
 functions are usually evaluated in computationally intensive inner loops of the
 simulation, the overhead caused by virtual methods may be significant.
 Therefore, this class uses CRTP to achieve compile-time polymorphism
 and avoids the overhead of virtual methods, and permits inlining instead.
 Concrete isoparametric elements must inherit from this base class and implement
 the interface this class provides. The derived isoparametric element must also
 be accompanied by a corresponding traits class that declares the compile time
 quantities and type declarations that this base class requires. One cannot
 combine the derived class and the traits class because the derived class is
 incomplete at the time when the traits is needed.
 @tparam DerivedElement  The concrete isoparametric element that inherits
                         from IsoparametricElement through CRTP.
 @tparam DerivedTraits   The traits class associated with the DerivedElement.
                         See LinearSimplexElementTraits for an example. */
template <class DerivedElement, class DerivedTraits>
class IsoparametricElement {
 public:
  /* The number of locations to evaluate various quantities in the
   element. */
  static constexpr int num_sample_locations =
      DerivedTraits::num_sample_locations;

  /* The dimension of the parent domain. */
  static constexpr int natural_dimension = DerivedTraits::natural_dimension;

  /* The dimension of the spatial domain. */
  static constexpr int spatial_dimension = DerivedTraits::spatial_dimension;

  /* The number of nodes in the element. E.g. 3 for linear triangles, 9 for
   quadratic quadrilaterals and 4 for linear tetrahedrons. */
  static constexpr int num_nodes = DerivedTraits::num_nodes;

  template <typename U>
  using ArrayNumSamples = std::array<U, num_sample_locations>;

  using T = typename DerivedTraits::Scalar;

  /* Fixed size vector type to store shape functions evaluated at a sample
   location. */
  using VectorNumNodes = Vector<T, num_nodes>;

  /* Fixed size matrix type to store gradient in parent coordinates at a sample
   locations. */
  using GradientInParentCoordinates =
      Eigen::Matrix<T, num_nodes, natural_dimension>;

  /* Fixed size matrix type to store gradient in spatial coordinates at a sample
   location. */
  using GradientInSpatialCoordinates =
      Eigen::Matrix<T, num_nodes, spatial_dimension>;

  /* Fixed size matrix type to store the Jacobian matrix at a sample location.
   */
  using JacobianMatrix = Eigen::Matrix<T, spatial_dimension, natural_dimension>;

  /* Fixed size matrix type to store the pseudoinverse Jacobian matrix at a
   sample location. */
  using PseudoinverseJacobianMatrix =
      Eigen::Matrix<T, natural_dimension, spatial_dimension>;

  /* Returns the std::array of sample locations in the parent domain at which
   the isoparametric element evaluates elemental quantities, as provided at
   construction. */
  const ArrayNumSamples<Vector<double, natural_dimension>>& locations() const {
    return locations_;
  }

  /* Obtains the shape function array
       S(ξ) = [S₀(ξ); S₁(ξ); ... Sₐ(ξ); ...; Sₙ₋₁(ξ)]
   at each sample location in the parent domain provided at construction.
   @returns an std::array of size equal to `num_sample_locations`. The q-th
   entry contains the vector S(ξ), of size `num_nodes`, evaluated at the q-th
   sample location in the parent domain provided at construction. The a-th
   component of S(ξ) corresponds to the shape function Sₐ(ξ) for node `a`. */
  const ArrayNumSamples<VectorNumNodes>& GetShapeFunctions() const {
    return S_;
  }

  /* Obtains the gradient of the shape functions in parent coordinates, dS/dξ,
   evaluated at each sample location in the parent domain provided at
   construction.
   @returns an std::array of size `num_sample_locations`. The q-th entry
   contains the matrix dS/dξ, of size `num_nodes`-by-`natural_dimension`,
   evaluated at the q-th sample location in the parent domain provided at
   construction. The a-th row of the matrix gives the derivatives dSₐ/dξ for
   node `a`. */
  const ArrayNumSamples<GradientInParentCoordinates>&
  GetGradientInParentCoordinates() const {
    return dSdxi_;
  }

  /* Calculates the gradient of the shape functions in spatial coordinates,
   dS/dX, evaluated at each sample location in the parent domain provided at
   construction.
   @param xa  Spatial coordinates for each element node stored column-wise.
   @returns an std::array of size `num_sample_locations`. The q-th entry
   contains the matrix dS/dX, of size `num_nodes`-by-`spatial_dimension`,
   evaluated at the q-th sample location in the parent domain provided at
   construction. The a-th row of the matrix gives the derivatives dSₐ/dX for
   node `a`.
   @pre the element with node positions `xa` is not degenerate. */
  ArrayNumSamples<GradientInSpatialCoordinates>
  CalcGradientInSpatialCoordinates(
      const Eigen::Ref<const Eigen::Matrix<T, spatial_dimension, num_nodes>>&
          xa) const {
    return derived().CalcGradientInSpatialCoordinates(xa);
  }

  /* Computes the Jacobian matrix, dx/dξ, at each sample location in the parent
   domain provided at construction.
   @param xa  Spatial coordinates for each element node stored column-wise.
   @returns an std::array of size 'num_sample_locations`. The q-th entry
   contains the Jacobian matrix dx/dξ, of size
   `spatial_dimension`-by-`natural_dimension`, evaluated at the q-th sample
   location in the parent domain provided at construction. */
  ArrayNumSamples<JacobianMatrix> CalcJacobian(
      const Eigen::Ref<const Eigen::Matrix<T, spatial_dimension, num_nodes>>&
          xa) const {
    return derived().CalcJacobian(xa);
  }

  /* Computes dξ/dx, the pseudoinverse Jacobian matrix, at each sample location
   in the parent domain provided at construction.
   @param jacobian An array of size `num_sample_locations`. The q-th entry
   contains the Jacobian matrix dx/dξ, of size
   `spatial_dimension`-by-`natural_dimension`, evaluated at the q-th sample
   location in the parent domain provided at construction.
   @pre Each entry in `jacobian` is full rank.
   @see CalcJacobian(). */
  ArrayNumSamples<PseudoinverseJacobianMatrix> CalcJacobianPseudoinverse(
      const ArrayNumSamples<JacobianMatrix>& jacobian) const {
    return derived().CalcJacobianPseudoinverse(jacobian);
  }

  /* Interpolates nodal values `ua` into u(ξ) at each sample location
   in the parent domain provided at construction.
   @param ua The value of function u at each element node stored column-wise.
   @returns an std::array of size `num_sample_locations` of the interpolated
   values of u.
   @tparam dim The dimension of the value of u. */
  template <int dim>
  ArrayNumSamples<Vector<T, dim>> InterpolateNodalValues(
      const Eigen::Ref<const Eigen::Matrix<T, dim, num_nodes>>& ua) const {
    const ArrayNumSamples<Vector<T, num_nodes>>& S = GetShapeFunctions();
    ArrayNumSamples<Vector<T, dim>> interpolated_value;
    for (int q = 0; q < num_sample_locations; ++q) {
      interpolated_value[q] = ua * S[q];
    }
    return interpolated_value;
  }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IsoparametricElement);

  /* Constructs an isoparametric element that performs calculations at the
   prescribed input `locations`. */
  IsoparametricElement(
      ArrayNumSamples<Vector<double, natural_dimension>> locations,
      ArrayNumSamples<Vector<T, num_nodes>> S,
      ArrayNumSamples<GradientInParentCoordinates> dSdxi)
      : locations_(std::move(locations)),
        S_(std::move(S)),
        dSdxi_(std::move(dSdxi)) {}

  /* Default implementations of "Calc" methods. Derived concrete elements should
   call these "DefaultCalc" methods unless they can provide an optimization.
   @{ */
  ArrayNumSamples<GradientInSpatialCoordinates>
  DefaultCalcGradientInSpatialCoordinates(
      const Eigen::Ref<const Eigen::Matrix<T, spatial_dimension, num_nodes>>&
          xa) const {
    ArrayNumSamples<GradientInSpatialCoordinates> dSdX;
    const auto& dSdxi = GetGradientInParentCoordinates();
    const auto dxidX = CalcJacobianPseudoinverse(CalcJacobian(xa));
    for (int q = 0; q < num_sample_locations; ++q) {
      dSdX[q] = dSdxi[q] * dxidX[q];
    }
    return dSdX;
  }

  ArrayNumSamples<JacobianMatrix> DefaultCalcJacobian(
      const Eigen::Ref<const Eigen::Matrix<T, spatial_dimension, num_nodes>>&
          xa) const {
    ArrayNumSamples<JacobianMatrix> dxdxi;
    const ArrayNumSamples<Eigen::Matrix<T, num_nodes, natural_dimension>>&
        dSdxi = GetGradientInParentCoordinates();
    for (int q = 0; q < num_sample_locations; ++q) {
      dxdxi[q] = xa * dSdxi[q];
    }
    return dxdxi;
  }

  ArrayNumSamples<PseudoinverseJacobianMatrix> DefaultCalcJacobianPseudoinverse(
      const ArrayNumSamples<JacobianMatrix>& jacobian) const {
    /* Suppose the Jacobian matrix J = dx/dξ is m×n where m >= n. We are looking
     for a n×m matrix A = dξ/dx. By the chain rule, AJ = I. In fact A is the
     pseudoinverse of J. To see that, observe that AJA = A, JAJ = J, (AJ)ᵀ = AJ.
     The only nontrivial fact is that (JA)ᵀ = JA. To see that, QR decompose J so
     that J = QR where Q is orthogonal and R is upper triangular, and define B =
     AQ. Here R and B are the dx/dξ and dξ/dx in the Q bases, and thus the m-n
     right-most columns of B are zero. Restricting B and R to their top-left n×n
     corners (call them B̂ and R̂̂), we get R̂B̂ = B̂ᵀR̂̂ᵀ = Iₙₓₙ. Padding zeros in the
     correct places, we see that BᵀRᵀ = RB. Therefore, QBᵀRᵀQᵀ = QRBQᵀ which
     implies AᵀJᵀ = JA. */
    ArrayNumSamples<PseudoinverseJacobianMatrix> dxidx;
    for (int q = 0; q < num_sample_locations; ++q) {
      // TODO(xuchenhan-tri): The rank revealing Eigen::JacobiSVD is used
      //  instead of Eigen::HouseholderQR to guard against rank-deficiency. This
      //  is fine for now as this function is only invoked in precomputes at the
      //  moment. Consider the more performant alternative when we need this in
      //  an inner loop.
      /* Thin unitaries are enough but Eigen does not allow them for fixed size
       matrix. */
      Eigen::JacobiSVD<JacobianMatrix> svd(
          jacobian[q], Eigen::ComputeFullU | Eigen::ComputeFullV);
      if (svd.rank() != natural_dimension) {
        throw std::runtime_error(
            "The element is degenerate and does not have a valid Jacobian "
            "pseudoinverse (the pseudoinverse is not the left inverse).");
      }
      /* Use SVD to solve for the least square system which gives the
       pseudoinverse. */
      dxidx[q] = svd.solve(
          Eigen::Matrix<T, spatial_dimension, spatial_dimension>::Identity());
    }
    return dxidx;
  }
  /* @} */

 private:
  const DerivedElement& derived() const {
    return static_cast<const DerivedElement&>(*this);
  }

  /* The locations at which to evaluate various quantities of this element. */
  ArrayNumSamples<Vector<double, natural_dimension>> locations_;
  /* Shape functions evaluated at points specified at construction. */
  ArrayNumSamples<Vector<T, num_nodes>> S_;
  /* Shape function derivatives evaluated at points specified at construction.
   */
  ArrayNumSamples<GradientInParentCoordinates> dSdxi_;
};

template <class Element>
struct is_isoparametric_element {
  static constexpr bool value =
      std::is_base_of_v<IsoparametricElement<Element, typename Element::Traits>,
                        Element>;
};
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
