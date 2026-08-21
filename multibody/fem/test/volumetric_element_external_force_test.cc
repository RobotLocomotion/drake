#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/fem/corotated_model.h"
#include "drake/multibody/fem/fem_state.h"
#include "drake/multibody/fem/linear_simplex_element.h"
#include "drake/multibody/fem/simplex_gaussian_quadrature.h"
#include "drake/multibody/fem/tet_subdivision_quadrature.h"
#include "drake/multibody/fem/volumetric_element.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/force_density_field.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* A constant per current volume force density field used for testing. */
class ConstantForceDensityField final : public ForceDensityField<double> {
 public:
  explicit ConstantForceDensityField(const Vector3<double>& f) : f_(f) {}

 private:
  Vector3<double> DoEvaluateAt(const systems::Context<double>& context,
                               const Vector3<double>& p_WQ) const final {
    return f_;
  };

  std::unique_ptr<ForceDensityFieldBase<double>> DoClone() const final {
    return std::make_unique<ConstantForceDensityField>(*this);
  }

  const Vector3<double> f_;
};

constexpr int kNaturalDimension = 3;
constexpr int kSpatialDimension = 3;
constexpr int kQuadratureOrder = 1;
constexpr double kTolerance = 1e-14;
constexpr int kSubdivisions = 2;
using QuadratureType =
    internal::SimplexGaussianQuadrature<kNaturalDimension, kQuadratureOrder>;
static constexpr int kNumQuads = QuadratureType::num_quadrature_points;
using SubdQuadratureType = TetSubdivisionQuadrature<kSubdivisions>;
static constexpr int kNumSubdQuads = SubdQuadratureType::num_quadrature_points;
using IsoparametricElementType =
    internal::LinearSimplexElement<double, kNaturalDimension, kSpatialDimension,
                                   kNumQuads>;
using SubdIsoparametricElementType =
    internal::LinearSimplexElement<double, kNaturalDimension, kSpatialDimension,
                                   kNumSubdQuads>;
using ConstitutiveModelType = CorotatedModel<double>;
using DeformationGradientDataType =
    std::array<CorotatedModelData<double>, kNumQuads>;

const double kYoungsModulus{1};
const double kPoissonRatio{0.25};
const double kDensity{1.23};

class VolumetricElementTest : public ::testing::Test {
 protected:
  using ElementType =
      VolumetricElement<IsoparametricElementType, QuadratureType,
                        ConstitutiveModelType, SubdIsoparametricElementType,
                        SubdQuadratureType>;
  using Data = typename ElementType::Data;
  static constexpr int kNumDofs = ElementType::num_dofs;
  static constexpr int kNumNodes = ElementType::num_nodes;
  const std::array<FemNodeIndex, kNumNodes> kNodeIndices = {
      {FemNodeIndex(0), FemNodeIndex(1), FemNodeIndex(2), FemNodeIndex(3)}};

  void SetUp() override {
    SetupElement();
    fem_state_system_ = std::make_unique<internal::FemStateSystem<double>>(
        VectorX<double>::Zero(kNumDofs), VectorX<double>::Zero(kNumDofs),
        VectorX<double>::Zero(kNumDofs));
    /* Set up element data. */
    std::function<void(const systems::Context<double>&, std::vector<Data>*)>
        calc_element_data = [this](const systems::Context<double>& context,
                                   std::vector<Data>* element_data) {
          /* There's only one element in the system. */
          element_data->resize(1);
          const FemState<double> fem_state(fem_state_system_.get(), &context);
          const Vector3<double> dummy_weights(1, 2, 3);
          (*element_data)[0] =
              (this->elements_)[0].ComputeData(fem_state, dummy_weights);
        };

    cache_index_ =
        fem_state_system_
            ->DeclareCacheEntry("Element data",
                                systems::ValueProducer(calc_element_data))
            .cache_index();
  }

  void SetupElement() {
    const Eigen::Matrix<double, kSpatialDimension, kNumNodes> X =
        reference_positions();
    ConstitutiveModelType constitutive_model(kYoungsModulus, kPoissonRatio);
    DampingModel<double> damping_model(0, 0);
    elements_.emplace_back(kNodeIndices, std::move(constitutive_model), X,
                           kDensity, std::move(damping_model));
  }

  /* Makes an FemState to be consumed by the unit tests with the given q, and
   zeros for v and a as the state values. */
  std::unique_ptr<FemState<double>> MakeFemState(const VectorX<double>& q) {
    DRAKE_DEMAND(q.size() == kNumDofs);
    auto fem_state =
        std::make_unique<FemState<double>>(fem_state_system_.get());
    fem_state->SetPositions(q);
    fem_state->SetVelocities(Vector<double, kNumDofs>::Zero());
    fem_state->SetAccelerations(Vector<double, kNumDofs>::Zero());
    return fem_state;
  }

  /* Sets arbitrary reference positions with the requirement that the
   tetrahedron is not inverted. */
  Eigen::Matrix<double, kSpatialDimension, kNumNodes> reference_positions()
      const {
    Eigen::Matrix<double, kSpatialDimension, kNumNodes> X(kSpatialDimension,
                                                          kNumNodes);
    // clang-format off
    X << -0.10, 0.90, 0.02, 0.10,
         1.33,  0.23, 0.04, 0.01,
         0.20,  0.03, 2.31, -0.12;
    // clang-format on
    return X;
  }

  /* Returns the one and only element. */
  const ElementType& element() const {
    DRAKE_DEMAND(elements_.size() == 1);
    return elements_[0];
  }

  /* Evaluates the element data of the sole element. */
  const Data& EvalElementData(const FemState<double>& state) const {
    const std::vector<Data>& all_data =
        state.EvalElementData<Data>(cache_index_);
    DRAKE_DEMAND(all_data.size() == 1);
    return all_data[0];
  }

  /* Returns the volume evaluated at each quadrature point in the reference
   configuration of the only element. */
  const std::array<double, kNumQuads>& reference_volume() const {
    return element().reference_volume_;
  }

  std::unique_ptr<FemStateSystem<double>> fem_state_system_;
  systems::CacheIndex cache_index_;
  std::vector<ElementType> elements_;
};

namespace {

/* Tests that when applying a per current volume force density field, the result
 agrees with analytic solution. */
TEST_F(VolumetricElementTest, PerCurrentVolumeExternalForce) {
  /* We scale the reference positions to get the current positions so we have
   precise control of the current volume. */
  const double scale = 1.23;
  const Eigen::Matrix<double, kSpatialDimension, kNumNodes> X_matrix =
      reference_positions();
  const Eigen::Matrix<double, kSpatialDimension, kNumNodes> x_matrix =
      scale * X_matrix;
  const Vector<double, kNumDofs> x(Eigen::Map<const Vector<double, kNumDofs>>(
      x_matrix.data(), x_matrix.size()));
  std::unique_ptr<FemState<double>> fem_state = MakeFemState(x);
  const auto& data = EvalElementData(*fem_state);

  const Vector3<double> force_per_current_volume(4, 5, 6);
  const ConstantForceDensityField external_force_field(
      force_per_current_volume);
  /* The constant force field doesn't depend on Context, but a Context is needed
   formally. So we create a dummy Context that's otherwise unused. */
  MultibodyPlant<double> plant(0.01);
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  fem::FemPlantData<double> plant_data{*context, {&external_force_field}};

  VectorX<double> external_force = VectorX<double>::Zero(kNumDofs);
  element().AddScaledExternalForces(data, plant_data, 1.0, &external_force);
  Vector3<double> total_force = Vector3<double>::Zero();
  for (int i = 0; i < kNumNodes; ++i) {
    total_force += external_force.segment<3>(3 * i);
  }

  const double current_volume = reference_volume()[0] * scale * scale * scale;
  const Vector3<double> expected_force =
      current_volume * force_per_current_volume;
  EXPECT_TRUE(CompareMatrices(total_force, expected_force, kTolerance));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
