#include "drake/multibody/fem/volumetric_element.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/fem/corotated_model.h"
#include "drake/multibody/fem/fem_state.h"
#include "drake/multibody/fem/linear_simplex_element.h"
#include "drake/multibody/fem/simplex_gaussian_quadrature.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

using std::make_unique;
using std::unique_ptr;

constexpr int kNaturalDimension = 3;
constexpr int kSpatialDimension = 3;
constexpr int kQuadratureOrder = 1;
constexpr double kEpsilon = 1e-14;
using AD = AutoDiffXd;
using QuadratureType =
    internal::SimplexGaussianQuadrature<kNaturalDimension, kQuadratureOrder>;
static constexpr int kNumQuads = QuadratureType::num_quadrature_points;
using IsoparametricElementType =
    internal::LinearSimplexElement<AD, kNaturalDimension, kSpatialDimension,
                                   kNumQuads>;
using ConstitutiveModelType = CorotatedModel<AD, kNumQuads>;
using DeformationGradientDataType = CorotatedModelData<AD, kNumQuads>;

const AD kYoungsModulus{1};
const AD kPoissonRatio{0.25};
const AD kDensity{1.23};
const AD kMassDamping{1e-4};
const AD kStiffnessDamping{1e-3};

class VolumetricElementTest : public ::testing::Test {
 protected:
  using ElementType = VolumetricElement<IsoparametricElementType,
                                        QuadratureType, ConstitutiveModelType>;
  using Data = typename ElementType::Data;
  static constexpr int kNumDofs = ElementType::num_dofs;
  static constexpr int kNumNodes = ElementType::num_nodes;
  const std::array<FemNodeIndex, kNumNodes> kNodeIndices = {
      {FemNodeIndex(0), FemNodeIndex(1), FemNodeIndex(2), FemNodeIndex(3)}};

  void SetUp() override {
    SetupElement();
    fem_state_system_ = make_unique<internal::FemStateSystem<AD>>(
        VectorX<AD>::Zero(kNumDofs), VectorX<AD>::Zero(kNumDofs),
        VectorX<AD>::Zero(kNumDofs));
    /* Set up element data. */
    std::function<void(const systems::Context<AD>&, std::vector<Data>*)>
        calc_element_data = [this](const systems::Context<AD>& context,
                                   std::vector<Data>* element_data) {
          /* There's only one element in the system. */
          element_data->resize(1);
          const FemState<AD> fem_state(fem_state_system_.get(), &context);
          (*element_data)[0] = (this->elements_)[0].ComputeData(fem_state);
        };

    cache_index_ =
        fem_state_system_
            ->DeclareCacheEntry("Element data",
                                systems::ValueProducer(calc_element_data))
            .cache_index();
  }

  void SetupElement() {
    Eigen::Matrix<AD, kSpatialDimension, kNumNodes> X = reference_positions();
    ConstitutiveModelType constitutive_model(kYoungsModulus, kPoissonRatio);
    DampingModel<AD> damping_model(0, 0);
    elements_.emplace_back(kNodeIndices, std::move(constitutive_model), X,
                           kDensity, std::move(damping_model));
  }

  /* Makes an FemState to be consumed by the unit tests with the given q, v, and
   a as the state values. */
  unique_ptr<FemState<AD>> MakeFemState(const VectorX<AD>& q,
                                        const VectorX<AD>& v,
                                        const VectorX<AD>& a) {
    auto fem_state = make_unique<FemState<AD>>(fem_state_system_.get());
    fem_state->SetPositions(q);
    fem_state->SetVelocities(v);
    fem_state->SetAccelerations(a);
    return fem_state;
  }

  /* Set up the state and data of a deformed element. This makes the positions
   the independent variables of the problem when we take gradients. */
  unique_ptr<FemState<AD>> MakeDeformedState() {
    Vector<double, kNumDofs> perturbation;
    perturbation << 0.18, 0.63, 0.54, 0.13, 0.92, 0.17, 0.03, 0.86, 0.85, 0.25,
        0.53, 0.67;
    Eigen::Matrix<AD, kSpatialDimension, kNumNodes> X = reference_positions();
    Vector<double, kNumDofs> x =
        Eigen::Map<Vector<double, kNumDofs>>(math::DiscardGradient(X).data(),
                                             reference_positions().size()) +
        perturbation;
    Vector<AD, kNumDofs> x_autodiff;
    math::InitializeAutoDiff(x, &x_autodiff);
    /* Set up arbitrary velocity and acceleration. */
    const Vector<AD, kNumDofs> v_autodiff = -1.23 * perturbation;
    const Vector<AD, kNumDofs> a_autodiff = 4.56 * perturbation;
    return MakeFemState(x_autodiff, v_autodiff, a_autodiff);
  }

  /* Set up a state where the positions are the same as reference positions.
   This makes the positions the independent variables of the problem when we
   take gradients.*/
  unique_ptr<FemState<AD>> MakeReferenceState() {
    Eigen::Matrix<AD, kSpatialDimension, kNumNodes> X = reference_positions();
    Vector<double, kNumDofs> x(Eigen::Map<Vector<double, kNumDofs>>(
        math::DiscardGradient(X).data(), reference_positions().size()));
    Vector<AD, kNumDofs> x_autodiff;
    math::InitializeAutoDiff(x, &x_autodiff);
    const Vector<AD, kNumDofs> v_autodiff = Vector<AD, kNumDofs>::Zero();
    const Vector<AD, kNumDofs> a_autodiff = Vector<AD, kNumDofs>::Zero();
    return MakeFemState(x_autodiff, v_autodiff, a_autodiff);
  }

  /* Set arbitrary reference positions with the requirement that the tetrahedron
   is not inverted. */
  Eigen::Matrix<AD, kSpatialDimension, kNumNodes> reference_positions() const {
    Eigen::Matrix<AD, kSpatialDimension, kNumNodes> X(kSpatialDimension,
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
  const Data& EvalElementData(const FemState<AD>& state) const {
    const std::vector<Data>& all_data =
        state.EvalElementData<Data>(cache_index_);
    DRAKE_DEMAND(all_data.size() == 1);
    return all_data[0];
  }

  /* Calculates the negative elastic force acting on the nodes of the only
   element evaluated with the given `fem_state`. */
  Vector<AD, kNumDofs> CalcNegativeElasticForce(
      const FemState<AD>& fem_state) const {
    Vector<AD, kNumDofs> neg_force = Vector<AD, kNumDofs>::Zero();
    element().AddNegativeElasticForce(EvalElementData(fem_state), &neg_force);
    return neg_force;
  }

  /* Calculates the negative elastic force derivative with respect to positions
   for the only element evaluated with the given `fem_state`. */
  Eigen::Matrix<AD, kNumDofs, kNumDofs> CalcNegativeElasticForceDerivative(
      const FemState<AD>& fem_state) const {
    Eigen::Matrix<AD, kNumDofs, kNumDofs> neg_force_derivative =
        Eigen::Matrix<AD, kNumDofs, kNumDofs>::Zero();
    element().AddScaledElasticForceDerivative(EvalElementData(fem_state), -1,
                                              &neg_force_derivative);
    return neg_force_derivative;
  }

  /* Calculates the DeformationGradientData for the only element evaluated with
   the given node positions. */
  DeformationGradientDataType CalcDeformationGradientData(
      const VectorX<AD>& q, const VectorX<AD>& q0) const {
    const std::array<Matrix3<AD>, kNumQuads> F =
        element().CalcDeformationGradient(q);
    const std::array<Matrix3<AD>, kNumQuads> F0 =
        element().CalcDeformationGradient(q0);
    DeformationGradientDataType deformation_gradient_data;
    deformation_gradient_data.UpdateData(F, F0);
    return deformation_gradient_data;
  }

  /* Calculates and verifies the energy and elastic forces evaluated with the
   given `data` are zero. */
  void VerifyEnergyAndForceAreZero(const FemState<AD>& fem_state) const {
    AD energy = element().CalcElasticEnergy(EvalElementData(fem_state));
    EXPECT_NEAR(energy.value(), 0, kEpsilon);
    Vector<AD, kNumDofs> neg_elastic_force =
        CalcNegativeElasticForce(fem_state);
    EXPECT_TRUE(CompareMatrices(Vector<AD, kNumDofs>::Zero(), neg_elastic_force,
                                kEpsilon));
  }

  /* Returns the constitutive model of the only element. */
  const ConstitutiveModelType& constitutive_model() const {
    return element().constitutive_model();
  }

  /* Returns the density of the only element. */
  const AD& density(const ElementType& e) const { return e.density_; }

  /* Returns the volume evaluated at each quadrature point in the reference
   configuration of the only element. */
  const std::array<AD, kNumQuads>& reference_volume() const {
    return element().reference_volume_;
  }

  /* Calculates the mass matrix of the only element. */
  Eigen::Matrix<AD, kNumDofs, kNumDofs> CalcMassMatrix(
      const FemState<AD>& fem_state) const {
    Eigen::Matrix<AD, kNumDofs, kNumDofs> mass_matrix =
        Eigen::Matrix<AD, kNumDofs, kNumDofs>::Zero();
    element().AddScaledMassMatrix(EvalElementData(fem_state), 1, &mass_matrix);
    return mass_matrix;
  }

  unique_ptr<FemStateSystem<AD>> fem_state_system_;
  systems::CacheIndex cache_index_;
  std::vector<ElementType> elements_;
};

namespace {

TEST_F(VolumetricElementTest, Constructor) {
  EXPECT_EQ(element().node_indices(), kNodeIndices);
  EXPECT_EQ(density(element()), kDensity);
  ElementType move_constructed_element(std::move(elements_[0]));
  EXPECT_EQ(move_constructed_element.node_indices(), kNodeIndices);
  EXPECT_EQ(density(move_constructed_element), kDensity);
}

/* Any undeformed state gives zero energy and zero force. */
TEST_F(VolumetricElementTest, UndeformedState) {
  /* The initial state where the current position is equal to reference
   position is undeformed. */
  unique_ptr<FemState<AD>> fem_state = MakeReferenceState();
  VerifyEnergyAndForceAreZero(*fem_state);

  /* Any rigid transformation of a undeformed state is undeformed. */
  math::RigidTransform<AD> transform(math::RollPitchYaw<AD>(1, 2, 3),
                                     Vector3<AD>(0.1, 0.2, 0.3));
  Eigen::Matrix<AD, kSpatialDimension, kNumNodes> X = reference_positions();
  Eigen::Matrix<AD, kSpatialDimension, kNumNodes> rigid_transformed_X;
  for (int i = 0; i < kNumNodes; ++i) {
    rigid_transformed_X.col(i) = transform * X.col(i);
  }
  fem_state->SetPositions(Eigen::Map<Vector<AD, kNumDofs>>(
      rigid_transformed_X.data(), rigid_transformed_X.size()));
  VerifyEnergyAndForceAreZero(*fem_state);
}

/* Tests that in a deformed state, the energy and forces agrees with
 hand-calculated results. */
TEST_F(VolumetricElementTest, DeformedState) {
  unique_ptr<FemState<AD>> fem_state = MakeReferenceState();
  /* Deform the element by scaling the initial position by a factor of 2. */
  fem_state->SetPositions(fem_state->GetPositions() * 2.0);
  const auto deformation_gradient_data = CalcDeformationGradientData(
      fem_state->GetPositions(), fem_state->GetPreviousStepPositions());
  std::array<AD, kNumQuads> energy_density_array;
  constitutive_model().CalcElasticEnergyDensity(deformation_gradient_data,
                                                &energy_density_array);
  const double energy_density = ExtractDoubleOrThrow(energy_density_array[0]);
  /* Set up a matrix to help with calculating volume of the element. */
  Matrix4<double> matrix_for_volume_calculation;
  matrix_for_volume_calculation.bottomRows<1>() = Vector4<double>::Ones();
  const auto X = reference_positions();
  const auto X_double = math::DiscardGradient(X);
  matrix_for_volume_calculation.topRows<3>() = X_double;
  const double reference_volume =
      1.0 / 6.0 * std::abs(matrix_for_volume_calculation.determinant());
  const double analytical_energy = energy_density * reference_volume;
  /* Verify calculated energy is close to energy calculated analytically. */
  EXPECT_NEAR(element().CalcElasticEnergy(EvalElementData(*fem_state)).value(),
              analytical_energy, kEpsilon);

  const auto neg_elastic_force_autodiff = CalcNegativeElasticForce(*fem_state);
  Vector<double, kNumDofs> neg_elastic_force =
      math::DiscardGradient(neg_elastic_force_autodiff);
  /* Force on node 0. */
  const Vector3<double> force0 = -neg_elastic_force.head<3>();
  /* The first piola stress. */
  std::array<Matrix3<AD>, kNumQuads> P_array;
  constitutive_model().CalcFirstPiolaStress(deformation_gradient_data,
                                            &P_array);
  const Matrix3<double>& P = math::DiscardGradient(P_array[0]);
  /* The directional face area of each face in the reference configuration. The
   indices are carefully ordered so that the direction is pointing to the inward
   face normal. */
  const Vector3<double> face012 =
      0.5 * (X_double.col(1) - X_double.col(0))
                .cross(X_double.col(2) - X_double.col(0));
  const Vector3<double> face013 =
      0.5 * (X_double.col(3) - X_double.col(0))
                .cross(X_double.col(1) - X_double.col(0));
  const Vector3<double> face023 =
      0.5 * (X_double.col(2) - X_double.col(0))
                .cross(X_double.col(3) - X_double.col(0));
  /* The analytic force exerted on node 0 is the average of the total force
   exerted on the faces incidenting node 0 because the linear isoparametric
   element is used. */
  const Vector3<double> force0_expected =
      P * (face012 + face013 + face023) / 3.0;
  EXPECT_TRUE(CompareMatrices(force0, force0_expected, kEpsilon));
}

/* Tests that at any given state, the negative elastic force is the derivative
 elastic energy with respect to the generalized positions. */
TEST_F(VolumetricElementTest, NegativeElasticForceIsEnergyDerivative) {
  unique_ptr<FemState<AD>> fem_state = MakeDeformedState();
  AD energy = element().CalcElasticEnergy(EvalElementData(*fem_state));
  Vector<AD, kNumDofs> neg_elastic_force = CalcNegativeElasticForce(*fem_state);
  EXPECT_TRUE(
      CompareMatrices(energy.derivatives(), neg_elastic_force, kEpsilon));
}

/* Tests that at any given state, CalcNegativeElasticForceDerivative() does in
 fact calculate the derivative of the negative elastic force. */
TEST_F(VolumetricElementTest, ElasticForceCompatibleWithItsDerivative) {
  unique_ptr<FemState<AD>> fem_state = MakeDeformedState();
  Vector<AD, kNumDofs> neg_elastic_force = CalcNegativeElasticForce(*fem_state);
  Eigen::Matrix<AD, kNumDofs, kNumDofs> neg_elastic_force_derivative =
      CalcNegativeElasticForceDerivative(*fem_state);
  for (int i = 0; i < kNumDofs; ++i) {
    EXPECT_TRUE(CompareMatrices(neg_elastic_force(i).derivatives().transpose(),
                                neg_elastic_force_derivative.row(i), kEpsilon));
  }
}

/* In each dimension, the entries of the mass matrix should sum up to the
 total mass assigned to the element. */
TEST_F(VolumetricElementTest, MassMatrixSumUpToTotalMass) {
  unique_ptr<FemState<AD>> deformed_fem_state = MakeDeformedState();
  const Eigen::Matrix<AD, kNumDofs, kNumDofs>& mass_matrix =
      CalcMassMatrix(*deformed_fem_state);
  const double mass_matrix_sum = mass_matrix.sum().value();
  double total_mass = 0;
  for (int q = 0; q < kNumQuads; ++q) {
    total_mass += (reference_volume()[q] * kDensity).value();
  }
  /* The mass matrix repeats the mass in each spatial dimension and needs to
   be scaled accordingly. */
  EXPECT_NEAR(mass_matrix_sum, total_mass * kSpatialDimension, kEpsilon);
}

TEST_F(VolumetricElementTest, AddScaledGravityForce) {
  const Vector3<AD> gravity_vector(1, 2, 3);
  VectorX<AD> scaled_gravity_force = VectorX<AD>::Zero(kNumDofs);
  const double scale = 2.0;
  unique_ptr<FemState<AD>> fem_state = MakeDeformedState();
  const Data& data = EvalElementData(*fem_state);
  /* Calculate the gravity force using the implementation in VolumetricElement.
   */
  element().AddScaledGravityForce(data, scale, gravity_vector,
                                  &scaled_gravity_force);
  /* Use the implementation in the base class as the reference. */
  VectorX<AD> expected_scaled_gravity_force = VectorX<AD>::Zero(kNumDofs);
  const FemElement<ElementType>& base_fem_element = element();
  base_fem_element.AddScaledGravityForce(data, scale, gravity_vector,
                                         &expected_scaled_gravity_force);
  EXPECT_TRUE(CompareMatrices(expected_scaled_gravity_force,
                              scaled_gravity_force, kEpsilon));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
