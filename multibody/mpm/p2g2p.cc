#include <chrono>
#include <iostream>

#include "transfer.h"

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;

void SetUp(int num_nodes_per_dim, int particles_per_cell, double dx,
           ParticleData<double>* particles) {
  // Create the particles.
  for (int i = 0; i < num_nodes_per_dim; ++i) {
    for (int j = 0; j < num_nodes_per_dim; ++j) {
      for (int k = 0; k < num_nodes_per_dim; ++k) {
        const Vector3d base_node(dx * i, dx * j, dx * k);
        for (int p = 0; p < particles_per_cell; ++p) {
          particles->m.push_back(1e-5);
          const Vector3d x =
              base_node + static_cast<double>(p) * dx /
                              (static_cast<double>(particles_per_cell) + 1.0) *
                              Vector3d::Ones();
          particles->x.push_back(x);
          particles->v.push_back(Vector3d(1.0, 1.0, 1.0));
          particles->F.push_back(Matrix3d::Identity());
          particles->C.push_back(Matrix3d::Zero());
          particles->P.push_back(Matrix3d::Zero());
          particles->bspline.push_back(BSplineWeights<double>(x, dx));
        }
      }
    }
  }
}

int do_main() {
  int num_nodes_per_dim = 32;
  int particles_per_cell = 8;
  const double dx = 0.01;
  const double dt = 0.002;
  ParticleData<double> particles;
  SparseGrid<double> grid(dx);

  SetUp(num_nodes_per_dim, particles_per_cell, dx, &particles);

  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < 100; ++i) {
    Transfer<double> transfer(dt, &grid, &particles);
    transfer.ParticleToGrid(false);
    grid.ExplicitVelocityUpdate(dt, Vector3d::Zero());
    MassAndMomentum<double> grid_stat = grid.ComputeTotalMassAndMomentum();
    transfer.GridToParticle(true);
    MassAndMomentum<double> particle_stat =
        ComputeTotalMassAndMomentum(particles, dx);
    DRAKE_DEMAND(std::abs(grid_stat.mass - particle_stat.mass) < 1E-10);
    DRAKE_DEMAND(CompareMatrices(grid_stat.linear_momentum,
                                 particle_stat.linear_momentum, 1E-10,
                                 MatrixCompareType::absolute));
    DRAKE_DEMAND(CompareMatrices(grid_stat.angular_momentum,
                                 particle_stat.angular_momentum, 1E-10,
                                 MatrixCompareType::absolute));
  }

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = end - start;
  std::cout << "Each time step takes: " << duration.count() * 10.0
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