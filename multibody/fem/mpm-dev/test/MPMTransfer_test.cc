#include "drake/multibody/fem/mpm-dev/MPMTransfer.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace mpm {

constexpr double kEps = 4.0 * std::numeric_limits<double>::epsilon();

class MPMTransferTest : public ::testing::Test {
 protected:
    void SetUp() { }

    void CheckSort1() {
        // First, construct a 3x3x3 grid centered at (0.0, 0.0, 0.0), with
        // h = 2:
        //         o - o - o
        //         |   |   |
        //     o - o - o - o
        //     |   |   |   |
        // o - o - o - o - o
        // |   |   |   |
        // o - o - o - o
        // |   |   |
        // o - o - o
        // And make 27 particles on the location of 27 grid points. The
        // particles' order are in a reversed lexiographical ordering.
        Vector3<int> num_grid_pt_1D, bottom_corner;
        double h;
        int num_grid_pt, num_particles, pc;

        num_grid_pt_1D = Vector3<int>(3, 3, 3);
        h = 2.0;
        bottom_corner = Vector3<int>(-1, -1, -1);

        Grid grid = Grid(num_grid_pt_1D, h, bottom_corner);
        num_particles = 27;
        Particles particles = Particles(num_particles);
        EXPECT_EQ(particles.get_num_particles(), num_particles);
        MPMTransfer mpm_transfer = MPMTransfer();
        num_grid_pt = grid.get_num_gridpt();

        EXPECT_EQ(num_grid_pt, 27);
        EXPECT_EQ(num_grid_pt_1D(0), 3);
        EXPECT_EQ(num_grid_pt_1D(1), 3);
        EXPECT_EQ(num_grid_pt_1D(2), 3);
        pc = num_grid_pt;
        for (int k = bottom_corner(2); k < bottom_corner(2)+num_grid_pt_1D(2);
                                                                        ++k) {
        for (int j = bottom_corner(1); j < bottom_corner(1)+num_grid_pt_1D(1);
                                                                        ++j) {
        for (int i = bottom_corner(0); i < bottom_corner(0)+num_grid_pt_1D(0);
                                                                        ++i) {
            particles.set_position(--pc, grid.get_position(i, j, k));
        }
        }
        }

        // Sanity check
        EXPECT_TRUE(CompareMatrices(particles.get_position(0),
                                    Vector3<double>(2.0, 2.0, 2.0),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(1),
                                    Vector3<double>(0.0, 2.0, 2.0),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(8),
                                    Vector3<double>(-2.0, -2.0, 2.0),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(13),
                                    Vector3<double>(0.0, 0.0, 0.0),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(26),
                                    Vector3<double>(-2.0, -2.0, -2.0),
                                    std::numeric_limits<double>::epsilon()));

        // Check particles in the correct ordering after sorting
        mpm_transfer.SortParticles(grid, &particles);

        EXPECT_TRUE(CompareMatrices(particles.get_position(0),
                                    Vector3<double>(-2.0, -2.0, -2.0),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(1),
                                    Vector3<double>(0.0, -2.0, -2.0),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(8),
                                    Vector3<double>(2.0, 2.0, -2.0),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(13),
                                    Vector3<double>(0.0, 0.0, 0.0),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(26),
                                    Vector3<double>(2.0, 2.0, 2.0),
                                    std::numeric_limits<double>::epsilon()));

        for (int i = 0; i < num_grid_pt; ++i) {
            EXPECT_EQ(mpm_transfer.batch_sizes_[i], 1);
        }
    }

    void CheckSort2() {
        // A more comprehensive test, where we add three particles in the batch
        // centered at (0, 0, 0) to the particles constructed in CheckSort1:
        // The number of particles in the batch shall look like:
        // (1, 1, ..., 4, 1, 1, ...)
        Vector3<int> num_grid_pt_1D, bottom_corner;
        double h;
        int num_grid_pt, num_particles, pc;

        num_grid_pt_1D = Vector3<int>(3, 3, 3);
        h = 2.0;
        bottom_corner = Vector3<int>(-1, -1, -1);

        num_particles = 30;
        Grid grid = Grid(num_grid_pt_1D, h, bottom_corner);
        Particles particles = Particles(num_particles);
        MPMTransfer mpm_transfer = MPMTransfer();
        num_grid_pt = grid.get_num_gridpt();

        particles.set_position(0, Vector3<double>(-0.5, 0.5, -0.5));
        particles.set_position(1, Vector3<double>(0.5, 0.5, 0.5));
        particles.set_position(2, Vector3<double>(0.5, -0.5, 0.5));
        pc = num_particles;
        for (int k = bottom_corner(2); k < bottom_corner(2)+num_grid_pt_1D(2);
                                                                        ++k) {
        for (int j = bottom_corner(1); j < bottom_corner(1)+num_grid_pt_1D(1);
                                                                        ++j) {
        for (int i = bottom_corner(0); i < bottom_corner(0)+num_grid_pt_1D(0);
                                                                        ++i) {
            particles.set_position(--pc, grid.get_position(i, j, k));
        }
        }
        }

        // Sanity check
        EXPECT_TRUE(CompareMatrices(particles.get_position(0),
                                    Vector3<double>(-0.5, 0.5, -0.5),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(1),
                                    Vector3<double>(0.5, 0.5, 0.5),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(2),
                                    Vector3<double>(0.5, -0.5, 0.5),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(3),
                                    Vector3<double>(2.0, 2.0, 2.0),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(29),
                                    Vector3<double>(-2.0, -2.0, -2.0),
                                    std::numeric_limits<double>::epsilon()));

        mpm_transfer.SortParticles(grid, &particles);

        // Check sorting
        EXPECT_TRUE(CompareMatrices(particles.get_position(0),
                                    Vector3<double>(-2.0, -2.0, -2.0),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(1),
                                    Vector3<double>(0.0, -2.0, -2.0),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(8),
                                    Vector3<double>(2.0, 2.0, -2.0),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(29),
                                    Vector3<double>(2.0, 2.0, 2.0),
                                    std::numeric_limits<double>::epsilon()));

        for (int i = 0; i < num_grid_pt; ++i) {
            if (i == 13) {
                EXPECT_EQ(mpm_transfer.batch_sizes_[i], 4);
            } else {
                EXPECT_EQ(mpm_transfer.batch_sizes_[i], 1);
            }
        }
    }

    void CheckSort3() {
        // A test with the same grid case 1 and 2, and we add a particle to
        // batches centered at (0, 0, 0), (1, 1, 1), and (-1, -1, -1).
        // The number of particles in the batch shall look like:
        // i=0           i=13          i=26
        // (1, 0, ..., 0, 1, 0, ..., 0, 1)
        Vector3<int> num_grid_pt_1D, bottom_corner;
        double h;
        int num_grid_pt, num_particles;

        num_grid_pt_1D = Vector3<int>(3, 3, 3);
        h = 2.0;
        bottom_corner = Vector3<int>(-1, -1, -1);

        num_particles = 9;
        Grid grid = Grid(num_grid_pt_1D, h, bottom_corner);
        Particles particles = Particles(num_particles);
        MPMTransfer mpm_transfer = MPMTransfer();
        num_grid_pt = grid.get_num_gridpt();

        particles.set_position(0, Vector3<double>(1.5, 1.5, 1.5));
        particles.set_position(1, Vector3<double>(-1.5, -1.5, -1.5));
        particles.set_position(2, Vector3<double>(-0.5, 0.5, -0.5));

        // Sanity check
        mpm_transfer.SortParticles(grid, &particles);

        // Check sorting
        EXPECT_TRUE(CompareMatrices(particles.get_position(0),
                                    Vector3<double>(-1.5, -1.5, -1.5),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(0),
                                    Vector3<double>(-0.5, 0.5, -0.5),
                                    std::numeric_limits<double>::epsilon()));
        EXPECT_TRUE(CompareMatrices(particles.get_position(0),
                                    Vector3<double>(1.5, 1.5, 1.5),
                                    std::numeric_limits<double>::epsilon()));

        for (int i = 0; i < num_grid_pt; ++i) {
            if (i == 0 || i == 13 || i == 26) {
                EXPECT_EQ(mpm_transfer.batch_sizes_[i], 1);
            } else {
                EXPECT_EQ(mpm_transfer.batch_sizes_[i], 0);
            }
        }
    }

    void checkPreallocation() {
        // Construct a grid of 5x5x5 on [-2,2]^3, and place 27 particles
        // on the centering 3x3x3 grid points.
        //                 -2  -1  0   1   2
        //                 o - o - o - o - o
        //                 |   |   |   |   |
        //             o - o - o - o - o - o
        //             |   |   |   |   |   |
        //         o - o - o - o - o - o - o
        //         |   |   |   |   |   |   |
        //     o - o - o - o - o - o - o - o
        //     |   |   |   |   |   |   |   |
        // o - o - o - o - o - o - o - o - o
        // |   |   |   |   |   |   |   |   |
        // o - o - o - o - o - o - o - o - o
        // |   |   |   |   |   |   |   |
        // o - o - o - o - o - o - o - o
        // |   |   |   |   |   |   |
        // o - o - o - o - o - o - o
        // |   |   |   |   |
        // o - o - o - o - o
        int pc;
        double sum_val;
        Vector3<double> xp, sum_gradient;
        std::vector<std::array<double, 27>> bases_val_particles;
        std::vector<std::array<Vector3<double>, 27>> bases_grad_particles;
        int h = 1.0;
        Vector3<int> num_gridpt_1D = { 5,  5,  5};
        Vector3<int> bottom_corner = {-2, -2, -2};
        Grid grid = Grid(num_gridpt_1D, h, bottom_corner);
        int num_particles = 27;
        Particles particles = Particles(num_particles);
        MPMTransfer mpm_transfer = MPMTransfer();

        // Set particles' positions to be on grid points
        pc = num_particles;
        for (int k = bottom_corner(2)+1;
                 k < bottom_corner(2)+num_gridpt_1D(2)-1; ++k) {
        for (int j = bottom_corner(1)+1;
                 j < bottom_corner(1)+num_gridpt_1D(1)-1; ++j) {
        for (int i = bottom_corner(0)+1;
                 i < bottom_corner(0)+num_gridpt_1D(0)-1; ++i) {
            particles.set_position(--pc, grid.get_position(i, j, k));
        }
        }
        }

        // Sort the particles and set up the batches and preallocate basis
        // evaluations
        mpm_transfer.SetUpTransfer(grid, &particles);

        // The particles are sorted, and for all particles, all bases that cover
        // the particle shall have evaluations sum to 1, and gradients sum to
        // 0, by the partition of unity property.
        for (int p = 0; p < num_particles; ++p) {
            EXPECT_EQ(mpm_transfer.bases_val_particles_[p][13], 0.75*0.75*0.75);
            xp = particles.get_position(p);
            sum_val = 0.0;
            sum_gradient = {0.0, 0.0, 0.0};
            for (int i = 0; i < 27; ++i) {
                sum_val += mpm_transfer.bases_val_particles_[p][i];
                sum_gradient += mpm_transfer.bases_grad_particles_[p][i];
            }
            EXPECT_EQ(sum_val, 1.0);
            EXPECT_TRUE(CompareMatrices(sum_gradient,
                                        Vector3<double>::Zero(), kEps));
        }
    }
};

namespace {

TEST_F(MPMTransferTest, SortParticlesTest) {
    CheckSort1();
    CheckSort2();
}

TEST_F(MPMTransferTest, SetUpTest) {
    checkPreallocation();
}

}  // namespace
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
