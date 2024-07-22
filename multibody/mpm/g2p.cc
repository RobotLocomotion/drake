#include <chrono>
#include <iostream>

#include "transfer.h"

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

using Eigen::Matrix3f;
using Eigen::Vector3f;

/* Creates a grid with `num_nodes_per_dim` nodes in each dimension. Samples
 `particles_per_cell` particles around a single grid node. 
 @param[in] dx grid spacing (meter).
 @param[out] particles Sampled particle data. */
void SetUp(int num_nodes_per_dim, int particles_per_cell, float dx,
           ParticleData<float>* particles) {
  for (int i = 0; i < num_nodes_per_dim; ++i) {
    for (int j = 0; j < num_nodes_per_dim; ++j) {
      for (int k = 0; k < num_nodes_per_dim; ++k) {
        const Vector3f base_node(dx * i, dx * j, dx * k);
        for (int p = 0; p < particles_per_cell; ++p) {
          particles->m.push_back(1e-5);
          const Vector3f x =
              base_node + static_cast<float>(p) * dx /
                              (static_cast<float>(particles_per_cell) + 1.0) *
                              Vector3f::Ones();
          particles->x.push_back(x);
          particles->v.push_back(Vector3f(1.0, 1.0, 1.0));
          particles->F.push_back(Matrix3f::Identity());
          particles->C.push_back(Matrix3f::Zero());
          particles->P.push_back(Matrix3f::Zero());
          particles->bspline.push_back(BSplineWeights<float>(x, dx));
        }
      }
    }
  }
}

int do_main() {
  /* Total number of particles is 32^3 * 8 = 262k. */
  int num_nodes_per_dim = 32;
  int particles_per_cell = 8;
  const float dx = 0.01;
  const float dt = 0.002;
  ParticleData<float> particles;
  SparseGrid<float> grid(dx);

  SetUp(num_nodes_per_dim, particles_per_cell, dx, &particles);
  Transfer<float> transfer(dt, &grid, &particles);
  transfer.ParticleToGrid(/* parallelize? */ false);
  grid.ExplicitVelocityUpdate(dt, Vector3f::Zero());

  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 300; ++i) {
    /* Use a fixed number of threads to get fair comparison across machines. */
    transfer.GridToParticle(Parallelism(12));
  }

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> duration = end - start;
  std::cout << "Each time step takes: " << duration.count() / 300.0 * 1000.0 
            << " milliseconds" << std::endl;
  return 0;
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::multibody::mpm::internal::do_main();
}