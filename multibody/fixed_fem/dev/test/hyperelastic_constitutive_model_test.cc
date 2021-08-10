#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/fem/deformation_gradient_data.h"
#include "drake/multibody/fixed_fem/dev/constitutive_model.h"

// TODO(xuchenhan-tri): Rename this file to constitutive_model_test.cc.
namespace drake {
namespace multibody {
namespace fem {
namespace internal {

using Eigen::Matrix3d;
constexpr int kNumLocations = 2;

/* Minimal required data type to be used in the derived constitutive model
 traits. */
template <typename T, int num_locations_at_compile_time>
struct DummyData : public DeformationGradientData<
                       DummyData<T, num_locations_at_compile_time>> {};

struct InvalidModelTraits {
  using Scalar = double;
  using Data = DummyData<double, kNumLocations>;
};

/* An invalid ConstitutiveModel that is missing the
 CalcElasticEnergyDensityImpl(), CalcFirstPiolaStressImpl(), and
 CalcFirstPiolaStressDerivativeImpl() methods. This class is used to test that
 an exception is thrown in the case where the derived class of ConstitutiveModel
 doesn't shadow these methods. */
class InvalidModel
    : public ConstitutiveModel<InvalidModel, InvalidModelTraits> {};

namespace {
GTEST_TEST(ConstitutiveModelTest, InvalidModel) {
  const InvalidModel model;
  const DummyData<double, kNumLocations> data;
  std::array<double, DummyData<double, kNumLocations>::num_locations>
      energy_density;
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.CalcElasticEnergyDensity(data, &energy_density), std::exception,
      fmt::format("The derived class {} must provide a shadow definition of "
                  "CalcElasticEnergyDensityImpl.. to be correct.",
                  NiceTypeName::Get(model)));

  std::array<Matrix3d, DummyData<double, kNumLocations>::num_locations> P;
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.CalcFirstPiolaStress(data, &P), std::exception,
      fmt::format("The derived class {} must provide a shadow definition of "
                  "CalcFirstPiolaStressImpl.. to be correct.",
                  NiceTypeName::Get(model)));

  std::array<Eigen::Matrix<double, 9, 9>,
             DummyData<double, kNumLocations>::num_locations>
      dPdF;
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.CalcFirstPiolaStressDerivative(data, &dPdF), std::exception,
      fmt::format("The derived class {} must provide a shadow definition of "
                  "CalcFirstPiolaStressDerivativeImpl.. to be correct.",
                  NiceTypeName::Get(model)));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
