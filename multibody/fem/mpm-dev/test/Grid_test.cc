#include "drake/multibody/fem/mpm-dev/Grid.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

constexpr double kEps = 4.0 * std::numeric_limits<double>::epsilon();

GTEST_TEST(GridClassTest, TestSetGet) {
    Vector3<int> num_gridpt_1D = {6, 3, 4};
    double h = 1.0;
    Vector3<int> bottom_corner  = {0, 0, 0};
    Grid grid = Grid(num_gridpt_1D, h, bottom_corner);
    double tmpscaling = 1.0;
    // Test the geometry of the grid
    EXPECT_EQ(grid.get_num_gridpt(), 72);
    EXPECT_TRUE(CompareMatrices(grid.get_num_gridpt_1D(), Vector3<int>(6, 3, 4),
                                kEps));
    EXPECT_EQ(grid.get_h(), 1.0);
    EXPECT_TRUE(CompareMatrices(grid.get_bottom_corner(), bottom_corner,
                                kEps));

    // Check whether the grid point positions are populated correctly
    for (int k = 0; k < 4; ++k) {
    for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 6; ++i) {
        EXPECT_TRUE(CompareMatrices(grid.get_position(i, j, k),
                                    Vector3<double>(i, j, k),
                                    kEps));
        // Randomly put some values in
        tmpscaling = 20.0*k + 10.0*j + i;
        grid.set_mass(i, j, k, tmpscaling);
        grid.set_velocity(i, j, k, Vector3<double>(tmpscaling,
                                                  -tmpscaling,
                                                   tmpscaling));
        grid.set_force(i, j, k, Vector3<double>(-tmpscaling,
                                                 tmpscaling,
                                                -tmpscaling));
    }
    }
    }

    EXPECT_TRUE(CompareMatrices(grid.get_velocity(1, 1, 1),
                                Vector3<double>(31.0, -31.0, 31.0),
                                kEps));
    EXPECT_TRUE(CompareMatrices(grid.get_force(1, 1, 1),
                                Vector3<double>(-31.0, 31.0, -31.0),
                                kEps));
    EXPECT_EQ(grid.get_mass(1, 1, 1), 31.0);
    EXPECT_TRUE(CompareMatrices(grid.get_velocity(4, 2, 2),
                                Vector3<double>(64.0, -64.0, 64.0),
                                kEps));
    EXPECT_TRUE(CompareMatrices(grid.get_force(4, 2, 2),
                                Vector3<double>(-64.0, 64.0, -64.0),
                                kEps));
    EXPECT_EQ(grid.get_mass(4, 2, 2), 64.0);

    // Test on a new grid
    num_gridpt_1D = {3, 3, 3};
    h = 0.5;
    bottom_corner  = {-2, 2, -2};
    grid = Grid(num_gridpt_1D, h, bottom_corner);

    // Test the geometry of the grid
    EXPECT_EQ(grid.get_num_gridpt(), 27);
    EXPECT_TRUE(CompareMatrices(grid.get_num_gridpt_1D(), Vector3<int>(3, 3, 3),
                                kEps));
    EXPECT_EQ(grid.get_h(), 0.5);
    EXPECT_TRUE(CompareMatrices(grid.get_bottom_corner(), bottom_corner,
                                kEps));

    // Check whether the grid point positions are populated correctly
    EXPECT_TRUE(CompareMatrices(grid.get_position(-2, 2, -2),
                                Vector3<double>(-1.0, 1.0, -1.0), kEps));
    EXPECT_TRUE(CompareMatrices(grid.get_position(-1, 2, -2),
                                Vector3<double>(-0.5, 1.0, -1.0), kEps));
    EXPECT_TRUE(CompareMatrices(grid.get_position(-2, 3, -2),
                                Vector3<double>(-1.0, 1.5, -1.0), kEps));
    EXPECT_TRUE(CompareMatrices(grid.get_position(-1, 3, -2),
                                Vector3<double>(-0.5, 1.5, -1.0), kEps));
    EXPECT_TRUE(CompareMatrices(grid.get_position(-2, 2, -1),
                                Vector3<double>(-1.0, 1.0, -0.5), kEps));
    EXPECT_TRUE(CompareMatrices(grid.get_position(-1, 2, -1),
                                Vector3<double>(-0.5, 1.0, -0.5), kEps));
    EXPECT_TRUE(CompareMatrices(grid.get_position(-2, 3, -1),
                                Vector3<double>(-1.0, 1.5, -0.5), kEps));
    EXPECT_TRUE(CompareMatrices(grid.get_position(-1, 3, -1),
                                Vector3<double>(-0.5, 1.5, -0.5), kEps));
    EXPECT_TRUE(CompareMatrices(grid.get_position(0, 4, 0),
                                Vector3<double>(0.0, 2.0, 0.0), kEps));
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
