#include "drake/multibody/fem/dev/linear_elasticity.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace fem {
template <typename T, int SpatialDim>
LinearElasticity<T, SpatialDim>::LinearElasticity(T E, T nu) : E_(E), nu_(nu) {
  VerifyParameterValidity(E, nu);
  SetLameParameters(E, nu);
}

template <typename T, int SpatialDim>
std::vector<T> LinearElasticity<T, SpatialDim>::CalcPsi(
    const ConstitutiveModelCache<T, SpatialDim>& cache) const {
  const LinearElasticityCache<T, SpatialDim>& linear_cache =
      static_cast<const LinearElasticityCache<T, SpatialDim>&>(cache);
  std::vector<T> Psi(linear_cache.num_quads);
  for (int i = 0; i < linear_cache.num_quads; ++i) {
    const auto& strain = linear_cache.strain[i];
    const auto& trace_strain = linear_cache.trace_strain[i];
    Psi[i] = mu_ * strain.squaredNorm() +
             0.5 * lambda_ * trace_strain * trace_strain;
  }
  return Psi;
}

template <typename T, int SpatialDim>
std::vector<typename LinearElasticity<T, SpatialDim>::MatrixD>
LinearElasticity<T, SpatialDim>::CalcP(
    const ConstitutiveModelCache<T, SpatialDim>& cache) const {
  const LinearElasticityCache<T, SpatialDim>& linear_cache =
      static_cast<const LinearElasticityCache<T, SpatialDim>&>(cache);
  std::vector<MatrixD> P(linear_cache.num_quads);
  for (int i = 0; i < linear_cache.num_quads; ++i) {
    const auto& strain = linear_cache.strain[i];
    const auto& trace_strain = linear_cache.trace_strain[i];
    P[i] = 2.0 * mu_ * strain + lambda_ * trace_strain * MatrixD::Identity();
  }
  return P;
}

template <typename T, int SpatialDim>
void LinearElasticity<T, SpatialDim>::VerifyParameterValidity(T E, T nu) const {
  if (E < 0.0) {
    throw std::logic_error("Young's modulus must be nonnegative.");
  }
  if (nu >= 0.5 || nu <= -1) {
    throw std::logic_error("Poisson ratio must be in (-1, 0.5).");
  }
}

template <typename T, int SpatialDim>
void LinearElasticity<T, SpatialDim>::SetLameParameters(T E, T nu) {
  mu_ = E / (2.0 * (1.0 + nu));
  lambda_ = E * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
}

template class LinearElasticity<double, 2>;
template class LinearElasticity<double, 3>;
template class LinearElasticity<AutoDiffXd, 2>;
template class LinearElasticity<AutoDiffXd, 3>;
}  // namespace fem
}  // namespace multibody
}  // namespace drake
