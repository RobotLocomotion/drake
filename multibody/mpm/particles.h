#pragma once

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/mpm/bspline_weights.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename T>
struct Particle {
  Particle(T& m_in, Vector3<T>& x_in, Vector3<T>& v_in, Matrix3<T>& F_in,
           Matrix3<T>& C_in, Matrix3<T>& P_in, BSplineWeights<T>& bspline_in)
      : m(m_in), x(x_in), v(v_in), F(F_in), C(C_in), P(P_in), bspline(bspline_in) {}
  T& m;
  Vector3<T>& x;
  Vector3<T>& v;
  Matrix3<T>& F;
  Matrix3<T>& C;
  Matrix3<T>& P;
  BSplineWeights<T>& bspline;
};

// TODO(xuchenhan-tri): Compare with AOS.
template <typename T>
struct ParticleData {
  Particle<T> particle(int i) {
    return Particle<T>(m[i], x[i], v[i], F[i], C[i], P[i], bspline[i]);
  }

  std::vector<T> m;
  std::vector<Vector3<T>> x;
  std::vector<Vector3<T>> v;
  std::vector<Matrix3<T>> F;
  std::vector<Matrix3<T>> C;
  std::vector<Matrix3<T>> P;
  std::vector<BSplineWeights<T>> bspline;
};

template <typename T>
MassAndMomentum<T> ComputeTotalMassAndMomentum(const ParticleData<T>& particles,
                                               const T& dx) {
  MassAndMomentum<T> result;
  const T D = dx * dx * 0.25;
  const int num_particles = particles.m.size();
  for (int i = 0; i < num_particles; ++i) {
    result.mass += particles.m[i];
    result.linear_momentum += particles.m[i] * particles.v[i];
    const Matrix3<T> B = particles.C[i] * D;  // C = B * D^{-1}
    result.angular_momentum +=
        particles.m[i] *
        (particles.x[i].cross(particles.v[i]) + ContractWithLeviCivita(B));
  }
  return result;
}

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
