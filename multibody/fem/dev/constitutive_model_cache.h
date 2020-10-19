#pragma once

#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fem {

/** Cached quantities that facilitates calculation such as energy density,
 stress and stress derivative in the constitutive model. There should be a
 one-to-one correspondence between constitutive model `Foo` and its cached
 quantities `FooCache`. The constitutive model takes its cache as an argument
 when performing various calculations.
 @tparam_nonsymbolic_scalar T.
 @tparam SpatialDim The spatial dimension of the domain. */
// TODO(xuchenhan-tri) link this doc to ConstitutiveModel when it is checked in.
template <typename T, int SpatialDim>
struct ConstitutiveModelCache {
  ConstitutiveModelCache(int element_index_in, int num_quads_in)
      : element_index(element_index_in), num_quads(num_quads_in) {}

  using MatrixD = Eigen::Matrix<T, SpatialDim, SpatialDim>;

  int element_index{-1};
  int num_quads{-1};
};

/** Cached quantities for the LinearElasticity constitutive model.
 @tparam_nonsymbolic_scalar T.
 @tparam SpatialDim The spatial dimension of the domain. */
template <typename T, int SpatialDim>
struct LinearElasticityCache : ConstitutiveModelCache<T, SpatialDim> {
  LinearElasticityCache(int element_index_in, int num_quads_in)
      : ConstitutiveModelCache<T, SpatialDim>(element_index_in, num_quads_in),
        F(num_quads_in),
        strain(num_quads_in),
        trace_strain(num_quads_in) {}
  using typename ConstitutiveModelCache<T, SpatialDim>::MatrixD;
  std::vector<MatrixD> F;       // Deformation gradient.
  std::vector<MatrixD> strain;  // Infinitesimal strain = 0.5 * (F + Fᵀ) - I.
  std::vector<T> trace_strain;  // Trace of `strain`.
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
