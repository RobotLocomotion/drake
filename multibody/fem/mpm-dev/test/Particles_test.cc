#include "drake/multibody/fem/mpm-dev/Particles.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

constexpr double kEps = 4.0 * std::numeric_limits<double>::epsilon();

GTEST_TEST(ParticlesClassTest, TestAddSetGet) {
    Vector3<double> pos1 = {1.0, 2.0, 3.0};
    Vector3<double> vel1 = {-1.0, -2.0, -3.0};
    double mass1 = 5.0;
    double vol1  = 10.0;
    Matrix3<double> F1 = pos1.asDiagonal();
    Matrix3<double> stress1 = vel1.asDiagonal();

    Vector3<double> pos2 = {3.0, -1.0, 6.0};
    Vector3<double> vel2 = {-9.0, 8.0, -2.0};
    double mass2 = 7.0;
    double vol2  = 3.0;
    Matrix3<double> F2 = pos2.asDiagonal();
    Matrix3<double> stress2 = vel2.asDiagonal();

    Particles particles = Particles();
    EXPECT_EQ(particles.get_num_particles(), 0);
    particles.AddParticle(pos1, vel1, mass1, vol1, F1, stress1);
    EXPECT_EQ(particles.get_num_particles(), 1);
    particles.AddParticle(pos2, vel2, mass2, vol2, F2, stress2);
    EXPECT_EQ(particles.get_num_particles(), 2);

    EXPECT_TRUE(CompareMatrices(particles.get_position(0), pos1,
                std::numeric_limits<double>::epsilon()));
    EXPECT_TRUE(CompareMatrices(particles.get_velocity(0), vel1,
                std::numeric_limits<double>::epsilon()));
    EXPECT_EQ(particles.get_mass(0), mass1);
    EXPECT_EQ(particles.get_reference_volume(0), vol1);
    EXPECT_TRUE(CompareMatrices(particles.get_deformation_gradient(0), F1,
                std::numeric_limits<double>::epsilon()));
    EXPECT_TRUE(CompareMatrices(particles.get_kirchhoff_stress(0), stress1,
                std::numeric_limits<double>::epsilon()));

    EXPECT_TRUE(CompareMatrices(particles.get_position(1), pos2,
                std::numeric_limits<double>::epsilon()));
    EXPECT_TRUE(CompareMatrices(particles.get_velocity(1), vel2,
                std::numeric_limits<double>::epsilon()));
    EXPECT_EQ(particles.get_mass(1), mass2);
    EXPECT_EQ(particles.get_reference_volume(1), vol2);
    EXPECT_TRUE(CompareMatrices(particles.get_deformation_gradient(1), F2,
                std::numeric_limits<double>::epsilon()));
    EXPECT_TRUE(CompareMatrices(particles.get_kirchhoff_stress(1), stress2,
                std::numeric_limits<double>::epsilon()));

    particles = Particles(2);
    EXPECT_EQ(particles.get_num_particles(), 2);
    particles.set_position(0, pos1);
    particles.set_velocity(0, vel1);
    particles.set_mass(0, mass1);
    particles.set_reference_volume(0, vol1);
    particles.set_deformation_gradient(0, F1);
    particles.set_kirchhoff_stress(0, stress1);
    particles.set_position(1, pos2);
    particles.set_velocity(1, vel2);
    particles.set_mass(1, mass2);
    particles.set_reference_volume(1, vol2);
    particles.set_deformation_gradient(1, F2);
    particles.set_kirchhoff_stress(1, stress2);

    EXPECT_TRUE(CompareMatrices(particles.get_position(0), pos1,
                std::numeric_limits<double>::epsilon()));
    EXPECT_TRUE(CompareMatrices(particles.get_velocity(0), vel1,
                std::numeric_limits<double>::epsilon()));
    EXPECT_EQ(particles.get_mass(0), mass1);
    EXPECT_EQ(particles.get_reference_volume(0), vol1);
    EXPECT_TRUE(CompareMatrices(particles.get_deformation_gradient(0), F1,
                std::numeric_limits<double>::epsilon()));
    EXPECT_TRUE(CompareMatrices(particles.get_kirchhoff_stress(0), stress1,
                std::numeric_limits<double>::epsilon()));

    EXPECT_TRUE(CompareMatrices(particles.get_position(1), pos2,
                std::numeric_limits<double>::epsilon()));
    EXPECT_TRUE(CompareMatrices(particles.get_velocity(1), vel2,
                std::numeric_limits<double>::epsilon()));
    EXPECT_EQ(particles.get_mass(1), mass2);
    EXPECT_EQ(particles.get_reference_volume(1), vol2);
    EXPECT_TRUE(CompareMatrices(particles.get_deformation_gradient(1), F2,
                std::numeric_limits<double>::epsilon()));
    EXPECT_TRUE(CompareMatrices(particles.get_kirchhoff_stress(1), stress2,
                std::numeric_limits<double>::epsilon()));

    particles.AddParticle(pos1, vel1, mass1, vol1, F1, stress1);
    EXPECT_EQ(particles.get_num_particles(), 3);
    EXPECT_TRUE(CompareMatrices(particles.get_position(2), pos1,
                std::numeric_limits<double>::epsilon()));
    EXPECT_TRUE(CompareMatrices(particles.get_velocity(2), vel1,
                std::numeric_limits<double>::epsilon()));
    EXPECT_EQ(particles.get_mass(2), mass1);
    EXPECT_EQ(particles.get_reference_volume(2), vol1);
    EXPECT_TRUE(CompareMatrices(particles.get_deformation_gradient(2), F1,
                std::numeric_limits<double>::epsilon()));
    EXPECT_TRUE(CompareMatrices(particles.get_kirchhoff_stress(2), stress1,
                std::numeric_limits<double>::epsilon()));
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
