#include "drake/multibody/fem/deformable_body_config.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace fem {
namespace {

GTEST_TEST(DeformableBodyConfigTest, DefaultValues) {
  DeformableBodyConfig<double> config;
  EXPECT_EQ(config.youngs_modulus(), 1e8);
  EXPECT_EQ(config.poissons_ratio(), 0.49);
  EXPECT_EQ(config.mass_damping_coefficient(), 0.0);
  EXPECT_EQ(config.stiffness_damping_coefficient(), 0.0);
  EXPECT_EQ(config.mass_density(), 1.5e3);
  EXPECT_EQ(config.material_model(), MaterialModel::kLinearCorotated);
  EXPECT_EQ(config.element_subdivision_count(), 0);
}

GTEST_TEST(DeformableBodyConfigTest, Setters) {
  DeformableBodyConfig<double> config;
  config.set_youngs_modulus(1e6);
  EXPECT_EQ(config.youngs_modulus(), 1e6);
  config.set_poissons_ratio(0.4);
  EXPECT_EQ(config.poissons_ratio(), 0.4);
  config.set_mass_damping_coefficient(1e-2);
  EXPECT_EQ(config.mass_damping_coefficient(), 1e-2);
  config.set_stiffness_damping_coefficient(1e-3);
  EXPECT_EQ(config.stiffness_damping_coefficient(), 1e-3);
  config.set_mass_density(1e3);
  EXPECT_EQ(config.mass_density(), 1e3);
  config.set_material_model(MaterialModel::kLinear);
  EXPECT_EQ(config.material_model(), MaterialModel::kLinear);
  config.set_element_subdivision_count(4);
  EXPECT_EQ(config.element_subdivision_count(), 4);
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
