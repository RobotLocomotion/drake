#include "drake/multibody/fixed_fem/dev/elasticity_element.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/fem/linear_simplex_element.h"
#include "drake/multibody/fem/simplex_gaussian_quadrature.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"

namespace drake {
namespace multibody {
namespace fem {
constexpr int kNaturalDimension = 3;
constexpr int kSpatialDimension = 3;
constexpr int kQuadratureOrder = 1;
const ElementIndex kZeroIndex(0);
using QuadratureType =
    internal::SimplexGaussianQuadrature<kNaturalDimension, kQuadratureOrder>;
static constexpr int kNumQuads = QuadratureType::num_quadrature_points;
using IsoparametricElementType =
    internal::LinearSimplexElement<AutoDiffXd, kNaturalDimension,
                                   kSpatialDimension, kNumQuads>;
using ConstitutiveModelType =
    internal::LinearConstitutiveModel<AutoDiffXd, kNumQuads>;

/* The traits for the DummyElasticityElement. `kOdeOrder` is set to zero to
 avoid states irrelevant to the tests. */
struct DummyElasticityElementTraits
    : ElasticityElementTraits<IsoparametricElementType, QuadratureType,
                              ConstitutiveModelType> {
  static constexpr int kOdeOrder = 0;
};

/* A simple ElasticityElement implementation. The calculation methods are
 implemented as returning the values calculated in ElasticityElement. */
class DummyElasticityElement final
    : public ElasticityElement<IsoparametricElementType, QuadratureType,
                               ConstitutiveModelType, DummyElasticityElement,
                               DummyElasticityElementTraits> {
 public:
  using Traits = DummyElasticityElementTraits;

  DummyElasticityElement(const DummyElasticityElement&) = delete;
  DummyElasticityElement(DummyElasticityElement&&) = default;
  const DummyElasticityElement& operator=(const DummyElasticityElement&) =
      delete;
  DummyElasticityElement&& operator=(const DummyElasticityElement&&) = delete;
  DummyElasticityElement(
      ElementIndex element_index,
      const std::array<NodeIndex, Traits::kNumNodes>& node_indices,
      const ConstitutiveModelType& constitutive_model,
      const Eigen::Ref<const Eigen::Matrix<T, Traits::kSolutionDimension,
                                           Traits::kNumNodes>>&
          reference_positions,
      const T& density, const Vector<T, Traits::kSpatialDimension>& gravity)
      : ElasticityElement<IsoparametricElementType, QuadratureType,
                          ConstitutiveModelType, DummyElasticityElement,
                          Traits>(element_index, node_indices,
                                  constitutive_model, reference_positions,
                                  density, gravity) {}

  /* Calculates the negative elastic force evaluted at `state`. */
  Vector<T, Traits::kNumDofs> CalcNegativeElasticForce(
      const FemState<DummyElasticityElement>& state) const {
    Vector<T, Traits::kNumDofs> neg_force = Vector<T, Traits::kNumDofs>::Zero();
    AddNegativeElasticForce(state, &neg_force);
    return neg_force;
  }

  /* Calculates the negative elastic force derivatives with respect to positions
   evaluted at `state`. */
  Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>
  CalcNegativeElasticForceDerivative(
      const FemState<DummyElasticityElement>& state) const {
    Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs> neg_force_derivative =
        Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>::Zero();
    AddNegativeElasticForceDerivative(state, &neg_force_derivative);
    return neg_force_derivative;
  }
};

class ElasticityElementTest : public ::testing::Test {
 protected:
  using T = AutoDiffXd;
  using ElementType = DummyElasticityElement;
  static constexpr int kNumDofs = ElementType::Traits::kNumDofs;
  static constexpr int kNumNodes = ElementType::Traits::kNumNodes;
  const std::array<NodeIndex, kNumNodes> dummy_node_indices = {
      {NodeIndex(0), NodeIndex(1), NodeIndex(2), NodeIndex(3)}};
  const T kYoungsModulus{1};
  const T kPoissonRatio{0.25};
  const T kDummyDensity{1.23};
  const Vector3<T> kGravity_W{0, 0, -9.8};

  void SetUp() override { SetupElement(); }

  void SetupElement() {
    Eigen::Matrix<T, kSpatialDimension, kNumNodes> X = reference_positions();
    ConstitutiveModelType model(kYoungsModulus, kPoissonRatio);
    elements_.emplace_back(kZeroIndex, dummy_node_indices, model, X,
                           kDummyDensity, kGravity_W);
  }

  /* Set up a state so that the element is deformed. */
  FemState<ElementType> SetupDeformedState() {
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
    FemState<ElementType> state(x_autodiff);
    state.MakeElementData(elements_);
    return state;
  }

  /* Set up a state where the positions are the same as reference positions. */
  FemState<ElementType> SetupInitialState() {
    Eigen::Matrix<T, kSpatialDimension, kNumNodes> X = reference_positions();
    Vector<double, kNumDofs> x(Eigen::Map<Vector<double, kNumDofs>>(
        math::DiscardGradient(X).data(), reference_positions().size()));
    Vector<T, kNumDofs> x_autodiff;
    math::InitializeAutoDiff(x, &x_autodiff);
    FemState<ElementType> state(x_autodiff);
    state.MakeElementData(elements_);
    return state;
  }

  /* Set arbitrary reference positions such that the tetrahedron is not
   inverted. */
  Eigen::Matrix<T, kSpatialDimension, kNumNodes> reference_positions() const {
    Eigen::Matrix<T, kSpatialDimension, kNumNodes> X(kSpatialDimension,
                                                     kNumNodes);
    // clang-format off
    X << -0.10, 0.90, 0.02, 0.10,
         1.33, 0.23, 0.04, 0.01,
         0.20, 0.03, 2.31, -0.12;
    // clang-format on
    return X;
  }

  /* Get the one and only element. */
  const ElementType& element() const {
    DRAKE_ASSERT(elements_.size() == 1);
    return elements_[0];
  }

  void VerifyEnergyAndForceAreZero(const FemState<ElementType>& state) const {
    T energy = element().CalcElasticEnergy(state);
    EXPECT_NEAR(energy.value(), 0, std::numeric_limits<double>::epsilon());
    Vector<T, kNumDofs> neg_elastic_force =
        element().CalcNegativeElasticForce(state);
    EXPECT_TRUE(CompareMatrices(Vector<T, kNumDofs>::Zero(), neg_elastic_force,
                                std::numeric_limits<double>::epsilon()));
  }

  const T& density(const ElementType& e) const { return e.density_; }

  const std::array<T, kNumQuads>& reference_volume() const {
    return element().reference_volume();
  }

  const Eigen::Matrix<T, kNumDofs, kNumDofs>& get_mass_matrix() const {
    return element().mass_matrix();
  }

  const Vector<T, kNumDofs>& gravity_force() const {
    return element().gravity_force();
  }

  std::vector<ElementType> elements_;
};

namespace {
TEST_F(ElasticityElementTest, Constructor) {
  EXPECT_EQ(element().node_indices(), dummy_node_indices);
  EXPECT_EQ(element().element_index(), kZeroIndex);
  EXPECT_EQ(density(element()), kDummyDensity);
  ElementType move_constructed_element(std::move(elements_[0]));
  EXPECT_EQ(move_constructed_element.node_indices(), dummy_node_indices);
  EXPECT_EQ(move_constructed_element.element_index(), kZeroIndex);
  EXPECT_EQ(density(move_constructed_element), kDummyDensity);
}

/* Any undeformed state gives zero energy and zero force. */
TEST_F(ElasticityElementTest, UndeformedState) {
  /* The initial state where the current position is equal to reference position
   is undeformed. */
  FemState<ElementType> state = SetupInitialState();
  VerifyEnergyAndForceAreZero(state);

  // TODO(xuchenhan-tri): In general, the elastic energy and force should be
  //  zero under any rigid transformation (not just translation) for any
  //  hyperelastic constitutive model. Linear elasticity is an exception in that
  //  it gives nonzero energy under rotation. Make the rigid transform a general
  //  one when we have isotropic nonlinear constitutive models.
  /* Any rigid transformation of a undeformed state is undeformed. */
  math::RigidTransform<T> transform(Vector3<T>(0.314, 0.159, 0.265));
  Eigen::Matrix<T, kSpatialDimension, kNumNodes> X = reference_positions();
  Eigen::Matrix<T, kSpatialDimension, kNumNodes> rigid_transformed_X;
  for (int i = 0; i < kNumNodes; ++i) {
    rigid_transformed_X.col(i) = transform * X.col(i);
  }
  state.SetQ(Eigen::Map<Vector<T, kNumDofs>>(rigid_transformed_X.data(),
                                             rigid_transformed_X.size()));
  VerifyEnergyAndForceAreZero(state);
}

/* Tests that in a deformed state, the energy and forces agrees with
 hand-calculated results. */
TEST_F(ElasticityElementTest, DeformedState) {
  FemState<ElementType> state = SetupInitialState();
  /* Deform the element by scaling the initial position by a factor of 2. The
   resulting deformation gradient would be a diagonal matrix with 2 on the
   diagonals. The linear strain then would the identity matrix. */
  state.SetQ(state.q() * 2.0);
  const auto strain = Matrix3<double>::Identity();
  const double trace_strain = strain.trace();

  /* Find the Lame parameters. */
  const double mu =
      kYoungsModulus.value() / (2.0 * (1.0 + kPoissonRatio.value()));
  const double lambda =
      kYoungsModulus.value() * kPoissonRatio.value() /
      ((1.0 + kPoissonRatio.value()) * (1.0 - 2.0 * kPoissonRatio.value()));

  const double energy_density =
      mu * strain.squaredNorm() + 0.5 * lambda * trace_strain * trace_strain;
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
              6.0 * std::numeric_limits<double>::epsilon());

  const auto neg_elastic_force_autodiff =
      element().CalcNegativeElasticForce(state);
  Vector<double, kNumDofs> neg_elastic_force =
      math::DiscardGradient(neg_elastic_force_autodiff);
  /* Force on node 0. */
  const Vector3<double> force0 = -neg_elastic_force.head<3>();
  /* Analytically calculated first piola stress. */
  const Matrix3<double> P =
      (2.0 * mu + trace_strain * lambda) * Matrix3<double>::Identity();
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
  const Vector3<double> force0_analytic =
      P * (face012 + face013 + face023) / 3.0;
  EXPECT_TRUE(CompareMatrices(force0, force0_analytic,
                              3.0 * std::numeric_limits<double>::epsilon()));
}

/* Tests that at any given state, the negative elastic force is the derivative
 elastic energy with respect to the generalized positions. */
TEST_F(ElasticityElementTest, NegativeElasticForceIsEnergyDerivative) {
  FemState<ElementType> state = SetupDeformedState();
  T energy = element().CalcElasticEnergy(state);
  Vector<T, kNumDofs> neg_elastic_force =
      element().CalcNegativeElasticForce(state);
  EXPECT_TRUE(CompareMatrices(energy.derivatives(), neg_elastic_force,
                              std::numeric_limits<double>::epsilon()));
}

/* Tests that at any given state, CalcNegativeElasticForceDerivative() does in
 fact calculates the derivative of the negative elastic force. */
TEST_F(ElasticityElementTest, ElasticForceCompatibleWithItsDerivative) {
  FemState<ElementType> state = SetupDeformedState();
  Vector<T, kNumDofs> neg_elastic_force =
      element().CalcNegativeElasticForce(state);
  Eigen::Matrix<T, kNumDofs, kNumDofs> neg_elastic_force_derivative =
      element().CalcNegativeElasticForceDerivative(state);
  for (int i = 0; i < kNumDofs; ++i) {
    EXPECT_TRUE(CompareMatrices(neg_elastic_force(i).derivatives().transpose(),
                                neg_elastic_force_derivative.row(i),
                                std::numeric_limits<double>::epsilon()));
  }
}

/* In each dimension, the entries of the mass matrix should sum up to the total
 mass assigned to the element. */
TEST_F(ElasticityElementTest, MassMatrixSumUpToTotalMass) {
  const Eigen::Matrix<T, kNumDofs, kNumDofs> mass_matrix = get_mass_matrix();
  const double mass_matrix_sum = mass_matrix.sum().value();
  double total_mass = 0;
  for (int q = 0; q < kNumQuads; ++q) {
    total_mass += (reference_volume()[q] * kDummyDensity).value();
  }
  /* The mass matrix repeats the mass in each spatial dimension and needs to be
   scaled accordingly. */
  EXPECT_EQ(mass_matrix_sum, total_mass * kSpatialDimension);
}

/* Tests that the gravity forces match the expected value. */
TEST_F(ElasticityElementTest, Gravity) {
  const Eigen::Matrix<T, kNumDofs, kNumDofs> mass_matrix = get_mass_matrix();
  Vector<T, kNumDofs> element_gravity_acceleration;
  for (int i = 0; i < kNumNodes; ++i) {
    element_gravity_acceleration.template segment<kSpatialDimension>(
        i * kSpaceDimension) = kGravity_W;
  }
  const Vector<T, kNumDofs> expected_gravity_force =
      mass_matrix * element_gravity_acceleration;
  EXPECT_TRUE(CompareMatrices(expected_gravity_force, gravity_force()));
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
