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

constexpr int kNaturalDimension = 3;
constexpr int kSpatialDimension = 3;
constexpr int kQuadratureOrder = 1;
constexpr double kEpsilon = 1e-14;
const ElementIndex kZeroIndex(0);
using T = AutoDiffXd;
using QuadratureType =
    internal::SimplexGaussianQuadrature<kNaturalDimension, kQuadratureOrder>;
static constexpr int kNumQuads = QuadratureType::num_quadrature_points;
using IsoparametricElementType =
    internal::LinearSimplexElement<T, kNaturalDimension, kSpatialDimension,
                                   kNumQuads>;
using ConstitutiveModelType = CorotatedModel<T, kNumQuads>;
using DeformationGradientDataType = CorotatedModelData<T, kNumQuads>;

class VolumetricElementTest : public ::testing::Test {
 protected:
  using ElementType = VolumetricElement<IsoparametricElementType,
                                        QuadratureType, ConstitutiveModelType>;
  static constexpr int kNumDofs = ElementType::num_dofs;
  static constexpr int kNumNodes = ElementType::num_nodes;
  const std::array<NodeIndex, kNumNodes> kNodeIndices = {
      {NodeIndex(0), NodeIndex(1), NodeIndex(2), NodeIndex(3)}};
  const T kYoungsModulus{1};
  const T kPoissonRatio{0.25};
  const T kDensity{1.23};
  const T kMassDamping{1e-4};
  const T kStiffnessDamping{1e-3};

  void SetUp() override { SetupElement(); }

  void SetupElement() {
    Eigen::Matrix<T, kSpatialDimension, kNumNodes> X = reference_positions();
    ConstitutiveModelType constitutive_model(kYoungsModulus, kPoissonRatio);
    DampingModel<T> damping_model(0, 0);
    elements_.emplace_back(kZeroIndex, kNodeIndices, constitutive_model, X,
                           kDensity, damping_model);
  }

  /* Set up a state so that the element is deformed. */
  FemStateImpl<ElementType> SetupDeformedState() {
    Vector<double, kNumDofs> perturbation;
    perturbation << 0.18, 0.63, 0.54, 0.13, 0.92, 0.17, 0.03, 0.86, 0.85, 0.25,
        0.53, 0.67;
    Eigen::Matrix<T, kSpatialDimension, kNumNodes> X = reference_positions();
    Vector<double, kNumDofs> x =
        Eigen::Map<Vector<double, kNumDofs>>(math::DiscardGradient(X).data(),
                                             reference_positions().size()) +
        perturbation;
    Vector<T, kNumDofs> x_autodiff;
    math::InitializeAutoDiff(x, &x_autodiff);
    /* Set up arbitrary velocity and acceleration. */
    const Vector<T, kNumDofs> v_autodiff = -1.23 * perturbation;
    const Vector<T, kNumDofs> a_autodiff = 4.56 * perturbation;
    FemStateImpl<ElementType> state(x_autodiff, v_autodiff, a_autodiff);
    state.MakeElementData(elements_);
    return state;
  }

  /* Set up a state where the positions are the same as reference positions. */
  FemStateImpl<ElementType> SetupInitialState() {
    Eigen::Matrix<T, kSpatialDimension, kNumNodes> X = reference_positions();
    Vector<double, kNumDofs> x(Eigen::Map<Vector<double, kNumDofs>>(
        math::DiscardGradient(X).data(), reference_positions().size()));
    Vector<T, kNumDofs> x_autodiff;
    math::InitializeAutoDiff(x, &x_autodiff);
    const Vector<T, kNumDofs> v_autodiff = Vector<T, kNumDofs>::Zero();
    const Vector<T, kNumDofs> a_autodiff = Vector<T, kNumDofs>::Zero();
    FemStateImpl<ElementType> state(x_autodiff, v_autodiff, a_autodiff);
    state.MakeElementData(elements_);
    return state;
  }

  /* Set arbitrary reference positions with the requirement that the tetrahedron
   is not inverted. */
  Eigen::Matrix<T, kSpatialDimension, kNumNodes> reference_positions() const {
    Eigen::Matrix<T, kSpatialDimension, kNumNodes> X(kSpatialDimension,
                                                     kNumNodes);
    // clang-format off
    X << -0.10, 0.90, 0.02, 0.10,
         1.33,  0.23, 0.04, 0.01,
         0.20,  0.03, 2.31, -0.12;
    // clang-format on
    return X;
  }

  /* Get the one and only element. */
  const ElementType& element() const {
    DRAKE_DEMAND(elements_.size() == 1);
    return elements_[0];
  }

  /* Calculates the negative elastic force acting on the nodes of the only
   element evaluated at the given `state`. */
  Vector<T, kNumDofs> CalcNegativeElasticForce(
      const FemStateImpl<ElementType>& state) const {
    Vector<T, kNumDofs> neg_force = Vector<T, kNumDofs>::Zero();
    element().AddNegativeElasticForce(state, &neg_force);
    return neg_force;
  }

  /* Calculates the negative elastic force derivative with respect to positions
   for the only element evaluated at the given `state`. */
  Eigen::Matrix<T, kNumDofs, kNumDofs> CalcNegativeElasticForceDerivative(
      const FemStateImpl<ElementType>& state) const {
    Eigen::Matrix<T, kNumDofs, kNumDofs> neg_force_derivative =
        Eigen::Matrix<T, kNumDofs, kNumDofs>::Zero();
    element().AddScaledElasticForceDerivative(state, -1, &neg_force_derivative);
    return neg_force_derivative;
  }

  /* Calculates the DeformationGradientData for the only element evaluated at
   the given `state`. */
  DeformationGradientDataType CalcDeformationGradientData(
      const FemStateImpl<ElementType>& state) const {
    const std::array<Matrix3<T>, kNumQuads> F =
        element().CalcDeformationGradient(state);
    DeformationGradientDataType deformation_gradient_data;
    deformation_gradient_data.UpdateData(F);
    return deformation_gradient_data;
  }

  /* Calculates and verifies the energy and elastic forces evaluated at the
   given `state` are zero. */
  void VerifyEnergyAndForceAreZero(
      const FemStateImpl<ElementType>& state) const {
    T energy = element().CalcElasticEnergy(state);
    EXPECT_NEAR(energy.value(), 0, std::numeric_limits<double>::epsilon());
    Vector<T, kNumDofs> neg_elastic_force = CalcNegativeElasticForce(state);
    EXPECT_TRUE(CompareMatrices(Vector<T, kNumDofs>::Zero(), neg_elastic_force,
                                std::numeric_limits<double>::epsilon()));
  }

  /* Returns the constitutive model of the only element. */
  const ConstitutiveModelType& constitutive_model() const {
    return element().constitutive_model();
  }

  /* Returns the density of the only element. */
  const T& density(const ElementType& e) const { return e.density_; }

  /* Returns the volume evaluated at each quadrature point in the reference
   configuration of the only element. */
  const std::array<T, kNumQuads>& reference_volume() const {
    return element().reference_volume_;
  }

  /* Returns the mass matrix of the only element. */
  const Eigen::Matrix<T, kNumDofs, kNumDofs>& get_mass_matrix() const {
    return element().mass_matrix_;
  }

  /* Returns the gravity force acting on the nodes of the only element at
   the given `state`. */
  Vector<T, kNumDofs> CalcGravityForce(
      const FemStateImpl<ElementType>& state) const {
    Vector<T, kNumDofs> gravity_force = Vector<T, kNumDofs>::Zero();
    element().AddScaledGravityForce(state, 1.0, &gravity_force);
    return gravity_force;
  }

  std::vector<ElementType> elements_;
};

namespace {

TEST_F(VolumetricElementTest, Constructor) {
  EXPECT_EQ(element().node_indices(), kNodeIndices);
  EXPECT_EQ(element().element_index(), kZeroIndex);
  EXPECT_EQ(density(element()), kDensity);
  ElementType move_constructed_element(std::move(elements_[0]));
  EXPECT_EQ(move_constructed_element.node_indices(), kNodeIndices);
  EXPECT_EQ(move_constructed_element.element_index(), kZeroIndex);
  EXPECT_EQ(density(move_constructed_element), kDensity);
}

/* Any undeformed state gives zero energy and zero force. */
TEST_F(VolumetricElementTest, UndeformedState) {
  /* The initial state where the current position is equal to reference
   position is undeformed. */
  FemStateImpl<ElementType> state = SetupInitialState();
  VerifyEnergyAndForceAreZero(state);

  /* Any rigid transformation of a undeformed state is undeformed. */
  math::RigidTransform<T> transform(math::RollPitchYaw<T>(1, 2, 3),
                                    Vector3<T>(0.314, 0.159, 0.265));
  Eigen::Matrix<T, kSpatialDimension, kNumNodes> X = reference_positions();
  Eigen::Matrix<T, kSpatialDimension, kNumNodes> rigid_transformed_X;
  for (int i = 0; i < kNumNodes; ++i) {
    rigid_transformed_X.col(i) = transform * X.col(i);
  }
  state.SetPositions(Eigen::Map<Vector<T, kNumDofs>>(
      rigid_transformed_X.data(), rigid_transformed_X.size()));
  VerifyEnergyAndForceAreZero(state);
}

/* Tests that in a deformed state, the energy and forces agrees with
 hand-calculated results. */
TEST_F(VolumetricElementTest, DeformedState) {
  FemStateImpl<ElementType> state = SetupInitialState();
  /* Deform the element by scaling the initial position by a factor of 2. */
  state.SetPositions(state.GetPositions() * 2.0);
  const auto deformation_gradient_data = CalcDeformationGradientData(state);
  std::array<T, kNumQuads> energy_density_array;
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
  EXPECT_NEAR(element().CalcElasticEnergy(state).value(), analytical_energy,
              kEpsilon);

  const auto neg_elastic_force_autodiff = CalcNegativeElasticForce(state);
  Vector<double, kNumDofs> neg_elastic_force =
      math::DiscardGradient(neg_elastic_force_autodiff);
  /* Force on node 0. */
  const Vector3<double> force0 = -neg_elastic_force.head<3>();
  /* The first piola stress. */
  std::array<Matrix3<T>, kNumQuads> P_array;
  constitutive_model().CalcFirstPiolaStress(deformation_gradient_data,
                                            &P_array);
  const Matrix3<double>& P = math::DiscardGradient(P_array[0]);
  /* The directional face area of the face formed by node 0, 1, and 2 in the
   reference configuration. The indices are carefully ordered so that the
   direction is pointing to the inward face normal. */
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
   exerted the faces incidenting node 0. */
  const Vector3<double> force0_expected =
      P * (face012 + face013 + face023) / 3.0;
  EXPECT_TRUE(CompareMatrices(force0, force0_expected, kEpsilon));
}

/* Tests that at any given state, the negative elastic force is the derivative
 elastic energy with respect to the generalized positions. */
TEST_F(VolumetricElementTest, NegativeElasticForceIsEnergyDerivative) {
  FemStateImpl<ElementType> state = SetupDeformedState();
  T energy = element().CalcElasticEnergy(state);
  Vector<T, kNumDofs> neg_elastic_force = CalcNegativeElasticForce(state);
  EXPECT_TRUE(
      CompareMatrices(energy.derivatives(), neg_elastic_force, kEpsilon));
}

/* Tests that at any given state, CalcNegativeElasticForceDerivative() does in
 fact calculates the derivative of the negative elastic force. */
TEST_F(VolumetricElementTest, ElasticForceCompatibleWithItsDerivative) {
  FemStateImpl<ElementType> state = SetupDeformedState();
  Vector<T, kNumDofs> neg_elastic_force = CalcNegativeElasticForce(state);
  Eigen::Matrix<T, kNumDofs, kNumDofs> neg_elastic_force_derivative =
      CalcNegativeElasticForceDerivative(state);
  for (int i = 0; i < kNumDofs; ++i) {
    EXPECT_TRUE(CompareMatrices(neg_elastic_force(i).derivatives().transpose(),
                                neg_elastic_force_derivative.row(i), kEpsilon));
  }
}

/* In each dimension, the entries of the mass matrix should sum up to the
 total mass assigned to the element. */
TEST_F(VolumetricElementTest, MassMatrixSumUpToTotalMass) {
  const Eigen::Matrix<T, kNumDofs, kNumDofs>& mass_matrix = get_mass_matrix();
  const double mass_matrix_sum = mass_matrix.sum().value();
  double total_mass = 0;
  for (int q = 0; q < kNumQuads; ++q) {
    total_mass += (reference_volume()[q] * kDensity).value();
  }
  /* The mass matrix repeats the mass in each spatial dimension and needs to
   be scaled accordingly. */
  EXPECT_EQ(mass_matrix_sum, total_mass * kSpatialDimension);
}

/* Tests that the gravity forces match the expected value. */
TEST_F(VolumetricElementTest, Gravity) {
  const Eigen::Matrix<T, kNumDofs, kNumDofs>& mass_matrix = get_mass_matrix();
  Vector<T, kNumDofs> element_gravity_acceleration;
  for (int i = 0; i < kNumNodes; ++i) {
    element_gravity_acceleration.template segment<kSpatialDimension>(
        i * kSpatialDimension) = element().gravity_vector();
  }
  const Vector<T, kNumDofs> expected_gravity_force =
      mass_matrix * element_gravity_acceleration;

  const FemStateImpl<ElementType> initial_state = SetupInitialState();
  EXPECT_TRUE(
      CompareMatrices(expected_gravity_force, CalcGravityForce(initial_state)));
  const FemStateImpl<ElementType> deformed_state = SetupDeformedState();
  EXPECT_TRUE(CompareMatrices(expected_gravity_force,
                              CalcGravityForce(deformed_state)));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
