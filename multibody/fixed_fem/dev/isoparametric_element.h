#pragma once
#include <array>
#include <utility>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** IsoparametricElement is a class that evaluates shape functions
 and their derivatives at prescribed locations. The shape function
 `S` located at a vertex `a` maps the parent domain to a scalar. The reference
 position `X` as well as `u(X)`, an arbitrary function on the reference domain,
 can be interpolated from nodal values `Xₐ` and `uₐ` to any location in the
 parent domain using the shape function, i.e.:

 <pre>
     X(ξ) = Sₐ(ξ)Xₐ, and
     u(X(ξ)) = Sₐ(ξ)uₐ,
 </pre>

 where ξ ∈ ℝᵈ is in the parent domain and d is its dimension (and we call it the
 natural dimension), which may be different from the dimension of X, which we
 call the spatial dimension (e.g. 2D membrane or shell element in 3D dynamics
 simulation has natural dimension 2 and spatial dimension 3). The constructor
 for this class takes in an array of locations at which we may evaluate and/or
 interpolate various quantities. If you need to evaluate and/or interpolate at
 other locations, construct another instance of %IsoparametricElement and pass
 the new locations into the constructor.

 %IsoparametricElement serves as the interface base class. Since shape
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
 @tparam DerivedElement The concrete isoparametric element that inherits
 from %IsoparametricElement through CRTP.
 @tparam DerivedTraits The traits class associated with the DerivedElement. */
template <class DerivedElement, class DerivedTraits>
class IsoparametricElement {
 public:
  /** The number of locations to evaluate various quantities in the
   element. */
  static constexpr int num_sample_locations() {
    return DerivedTraits::kNumSampleLocations;
  }

  /** The dimension of the parent domain. */
  static constexpr int natural_dimension() {
    return DerivedTraits::kNaturalDimension;
  }

  /** The dimension of the spatial domain. */
  static constexpr int spatial_dimension() {
    return DerivedTraits::kSpatialDimension;
  }

  /** The number of nodes in the element. E.g. 3 for linear triangles, 9 for
   quadratic quadrilaterals and 4 for linear tetrahedrons. */
  static constexpr int num_nodes() { return DerivedTraits::kNumNodes; }

  /** std::array of size `num_sample_locations()`. */
  template <typename U>
  using ArrayType = std::array<U, num_sample_locations()>;

  using T = typename DerivedTraits::Scalar;

  /** Type of locations at which to evaluate and/or interpolate numerical
   quantities from nodes. */
  using LocationsType = ArrayType<Vector<double, natural_dimension()>>;

  /** Fixed size matrix type to store the Jacobian matrix. */
  using JacobianMatrix =
      Eigen::Matrix<T, spatial_dimension(), natural_dimension()>;

  /** Fixed size matrix type to store the pseudoinverse Jacobian matrix. */
  using PseudoinverseJacobianMatrix =
      Eigen::Matrix<T, natural_dimension(), spatial_dimension()>;

  /** Returns the array of sample locations in the parent domain at which the
   isoparametric element evaluates elemental quantities, as provided at
   construction. */
  const LocationsType& locations() const { return locations_; }

  /** Obtains the shape function array
     <pre>
          S(ξ) = [S₀(ξ); S₁(ξ); ... Sₐ(ξ); ...; Sₙ₋₁(ξ)]
     </pre>
   at each location in the parent domain provided at construction.
   @returns an array of size equal to `num_sample_locations()`. The q-th entry
   contains the vector S(ξ), of size num_nodes(), evaluated at the
   q-th sample location in the parent domain provided at construction.
   The a-th component of S(ξ) corresponds to the shape function Sₐ(ξ) for node
   a. */
  const ArrayType<Vector<T, num_nodes()>>& GetShapeFunctions() const {
    const DerivedElement& derived = static_cast<const DerivedElement&>(*this);
    return derived.GetShapeFunctions();
  }

  /** Obtains the gradient of the shape functions in parent coordinates, dS/dξ,
   evaluated at each location in the parent domain provided at construction.
   @returns an array of size `num_sample_locations()`. The q-th entry contains
   the matrix dS/dξ, of size `num_nodes()`-by-`natural_dimension()`, evaluated
   at the q-th sample location in the parent domain provided at construction.
   The a-th row of the matrix gives the derivatives dSₐ/dξ for node a. */
  const ArrayType<Eigen::Matrix<T, num_nodes(), natural_dimension()>>&
  GetGradientInParentCoordinates() const {
    const DerivedElement& derived = static_cast<const DerivedElement&>(*this);
    return derived.GetGradientInParentCoordinates();
  }

  // TODO(xuchenhan-tri): Implement CalcGradientInSpatialCoordinates().

  /** Computes the Jacobian matrix, dx/dξ, at each location in the parent domain
   provided at construction.
   @param xa Spatial coordinates for each element node stored column-wise.
   @returns an array of size 'num_sample_locations()`. The q-th entry contains
   the Jacobian matrix dx/dξ, of size
   `spatial_dimension()`-by-`natural_dimension()`, evaluated at the q-th sample
   location in the parent domain provided at construction. */
  ArrayType<JacobianMatrix> CalcJacobian(
      const Eigen::Ref<
          const Eigen::Matrix<T, spatial_dimension(), num_nodes()>>& xa) const {
    ArrayType<JacobianMatrix> dxdxi;
    const ArrayType<Eigen::Matrix<T, num_nodes(), natural_dimension()>>& dSdxi =
        GetGradientInParentCoordinates();
    for (int q = 0; q < num_sample_locations(); ++q) {
      dxdxi[q] = xa * dSdxi[q];
    }
    return dxdxi;
  }

  /** Computes dξ/dx, the pseudoinverse Jacobian matrix, at each sample location
   in the parent domain provided at construction.
   @param xa Spatial coordinates for each element node stored column-wise.
   @returns an array of size `num_sample_locations()`. The q-th entry contains
   the pseudoinverse Jacobian dξ/dx, of size
   `natural_dimension()`-by-`spatial_dimension()`, evaluated at the q-th sample
   location in the parent domain provided at construction.
   @pre the element with node positions `xa` must not be degenerate.
   @see CalcJacobian().  */
  ArrayType<PseudoinverseJacobianMatrix> CalcJacobianPseudoinverse(
      const Eigen::Ref<
          const Eigen::Matrix<T, spatial_dimension(), num_nodes()>>& xa) const {
    ArrayType<JacobianMatrix> dxdxi = CalcJacobian(xa);
    return CalcJacobianPseudoinverse(dxdxi);
  }

  /** Preferred signature for computing dξ/dx when the Jacobian matrices are
   available so as to avoid recomputing the Jacobian matrices.
   @param jacobian An array of size `num_sample_locations()`. The q-th entry
   contains the Jacobian matrix dx/dξ, of size
   `spatial_dimension()`-by-`natural_dimension()`, evaluated at the q-th sample
   location in the parent domain provided at construction.
   @pre Each entry in `jacobian` must be full rank.
   @see CalcJacobian(). */
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
  // TODO(xuchenhan-tri): The rank revealing Eigen::JacobiSVD is used instead of
  //  Eigen::HouseholderQR to guard against rank-deficiency. This is fine for
  //  now as this function is only invoked in precomputes at the moment.
  //  Consider the more performant alternative when we need this in an inner
  //  loop.
  ArrayType<PseudoinverseJacobianMatrix> CalcJacobianPseudoinverse(
      const ArrayType<JacobianMatrix>& jacobian) const {
    ArrayType<PseudoinverseJacobianMatrix> dxidx;
    for (int q = 0; q < num_sample_locations(); ++q) {
      /* Thin unitaries are enough but Eigen does not allow them for fixed size
       matrix. */
      Eigen::JacobiSVD<JacobianMatrix> svd(
          jacobian[q], Eigen::ComputeFullU | Eigen::ComputeFullV);
      if (svd.rank() != natural_dimension()) {
        throw std::runtime_error(
            "The element is degenerate and does not have a valid Jacobian "
            "pseudoinverse (the pseudoinverse is not the left inverse).");
      }
      /* Use SVD to solve for the least square system which gives the
       pseudoinverse. */
      dxidx[q] = svd.solve(Eigen::Matrix<T, spatial_dimension(),
                                         spatial_dimension()>::Identity());
    }
    return dxidx;
  }

  /** Interpolates nodal values `ua` into u(ξ) at each sample location
   in the parent domain provided at construction.
   @param ua The value of function u at each element node stored column-wise.
   @returns an array of size `num_sample_locations()` of the interpolated
   values of u.
   @tparam dim The dimension of the value of u. */
  template <int dim>
  ArrayType<Vector<T, dim>> InterpolateNodalValues(
      const Eigen::Ref<const Eigen::Matrix<T, dim, num_nodes()>>& ua) const {
    const ArrayType<Vector<T, num_nodes()>>& S = GetShapeFunctions();
    ArrayType<Vector<T, dim>> interpolated_value;
    for (int q = 0; q < num_sample_locations(); ++q) {
      interpolated_value[q] = ua * S[q];
    }
    return interpolated_value;
  }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IsoparametricElement);

  /** Constructs an isoparametric element that performs calculations at the
   locations specified by the input `locations`. */
  explicit IsoparametricElement(LocationsType locations)
      : locations_(std::move(locations)) {}

 private:
  /* The locations at which to evaluate various quantities of this element. */
  LocationsType locations_;
};

template <class Element>
struct is_isoparametric_element {
  static constexpr bool value =
      std::is_base_of<IsoparametricElement<Element, typename Element::Traits>,
                      Element>::value;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
