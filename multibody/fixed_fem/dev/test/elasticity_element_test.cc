#include "drake/multibody/fixed_fem/dev/elasticity_element.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/linear_simplex_element.h"
#include "drake/multibody/fixed_fem/dev/simplex_gaussian_quadrature.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace {
constexpr int kNaturalDimension = 3;
constexpr int kSpatialDimension = 3;
constexpr int kQuadratureOrder = 1;
const ElementIndex kZeroIndex(0);
using QuadratureType =
    SimplexGaussianQuadrature<kNaturalDimension, kQuadratureOrder>;
static constexpr int kNumQuads = QuadratureType::num_quadrature_points();
using IsoparametricElementType =
    LinearSimplexElement<AutoDiffXd, kNaturalDimension, kSpatialDimension,
                         kNumQuads>;
using ConstitutiveModelType = LinearConstitutiveModel<AutoDiffXd, kNumQuads>;

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
          reference_positions)
      : ElasticityElement<IsoparametricElementType, QuadratureType,
                          ConstitutiveModelType, DummyElasticityElement,
                          Traits>(element_index, node_indices,
                                  constitutive_model, reference_positions) {}

  /* Calculates the negative elastic force evaluted at `state_`. */
  Vector<T, Traits::kNumDofs> CalcNegativeElasticForce(
      const FemState<DummyElasticityElement>& state) const {
    Vector<T, Traits::kNumDofs> neg_force = Vector<T, Traits::kNumDofs>::Zero();
    AddNegativeElasticForce(state, &neg_force);
    return neg_force;
  }

  /* Calculates the negative elastic force derivatives with respect to positions
   evaluted at `state_`. */
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
  using ElementType = DummyElasticityElement;
  static constexpr int kNumDofs = ElementType::Traits::kNumDofs;
  static constexpr int kNumNodes = ElementType::Traits::kNumNodes;
  const std::array<NodeIndex, kNumNodes> dummy_node_indices = {
      {NodeIndex(0), NodeIndex(1), NodeIndex(2), NodeIndex(3)}};

  void SetUp() override { SetupElement(); }

  void SetupElement() {
    Eigen::Matrix<AutoDiffXd, kSpatialDimension, kNumNodes> X =
        reference_positions();
    ConstitutiveModelType model(1, 0.25);
    elements_.emplace_back(kZeroIndex, dummy_node_indices, model, X);
  }

  /* Set up a state where the positions are random. */
  void SetupRandomState() {
    // Set arbitrary node positions and the gradient.
    Vector<double, kNumDofs> x;
    x << 0.18, 0.63, 0.54, 0.13, 0.92, 0.17, 0.03, 0.86, 0.85, 0.25, 0.53, 0.67;
    Vector<AutoDiffXd, kNumDofs> x_autodiff;
    math::initializeAutoDiff(x, x_autodiff);
    state_ = std::make_unique<FemState<ElementType>>(x_autodiff);
    state_->MakeElementData(elements_);
  }

  /* Set up a state where the positions are the same as reference positions. */
  void SetupInitialState() {
    Eigen::Matrix<AutoDiffXd, kSpatialDimension, kNumNodes> X =
        reference_positions();
    Vector<double, kNumDofs> x(Eigen::Map<Vector<double, kNumDofs>>(
        math::DiscardGradient(X).data(), reference_positions().size()));
    Vector<AutoDiffXd, kNumDofs> x_autodiff;
    math::initializeAutoDiff(x, x_autodiff);
    state_ = std::make_unique<FemState<ElementType>>(x_autodiff);
    state_->MakeElementData(elements_);
  }

  /* Set arbitrary reference positions such that the tetrahedron is not
   inverted. */
  Eigen::Matrix<AutoDiffXd, kSpatialDimension, kNumNodes> reference_positions()
      const {
    Eigen::Matrix<AutoDiffXd, kSpatialDimension, kNumNodes> X(kSpatialDimension,
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

  void VerifyEnergyAndForceAreZero() const {
    AutoDiffXd energy = element().CalcElasticEnergy(*state_);
    EXPECT_NEAR(energy.value(), 0, std::numeric_limits<double>::epsilon());
    Vector<AutoDiffXd, kNumDofs> neg_elastic_force =
        element().CalcNegativeElasticForce(*state_);
    EXPECT_TRUE(CompareMatrices(Vector<AutoDiffXd, kNumDofs>::Zero(),
                                neg_elastic_force,
                                std::numeric_limits<double>::epsilon()));
  }

  std::vector<ElementType> elements_;
  std::unique_ptr<FemState<ElementType>> state_;
};

TEST_F(ElasticityElementTest, Constructor) {
  EXPECT_EQ(element().node_indices(), dummy_node_indices);
  EXPECT_EQ(element().element_index(), kZeroIndex);
  ElementType move_constructed_element(std::move(elements_[0]));
  EXPECT_EQ(move_constructed_element.node_indices(), dummy_node_indices);
  EXPECT_EQ(move_constructed_element.element_index(), kZeroIndex);
}

/* Any undeformed state gives zero energy and zero force. */
TEST_F(ElasticityElementTest, UndeformedState) {
  /* The initial state where the current position is equal to reference position
   is undeformed. */
  SetupInitialState();
  VerifyEnergyAndForceAreZero();

  // TODO(xuchenhan-tri): In general, the elastic energy and force should be
  //  zero under any rigid transformation (not just translation) for any
  //  hyperelastic constitutive model. Linear elasticity is an exception in that
  //  it gives nonzero energy under rotation. Make the rigid transform a general
  //  one when we have isotropic nonlinear constitutive models.
  /* Any rigid transformation of a undeformed state is undeformed. */
  math::RigidTransform<AutoDiffXd> transform(
      Vector3<AutoDiffXd>(0.314, 0.159, 0.265));
  Eigen::Matrix<AutoDiffXd, kSpatialDimension, kNumNodes> X =
      reference_positions();
  Eigen::Matrix<AutoDiffXd, kSpatialDimension, kNumNodes> rigid_transformed_X;
  for (int i = 0; i < kNumNodes; ++i) {
    rigid_transformed_X.col(i) = transform * X.col(i);
  }
  state_->set_q(Eigen::Map<Vector<AutoDiffXd, kNumDofs>>(
      rigid_transformed_X.data(), rigid_transformed_X.size()));
  VerifyEnergyAndForceAreZero();
}

TEST_F(ElasticityElementTest, NegativeElasticForceIsEnergyDerivative) {
  SetupRandomState();
  AutoDiffXd energy = element().CalcElasticEnergy(*state_);
  Vector<AutoDiffXd, kNumDofs> neg_elastic_force =
      element().CalcNegativeElasticForce(*state_);
  EXPECT_TRUE(CompareMatrices(energy.derivatives(), neg_elastic_force,
                              std::numeric_limits<double>::epsilon()));
}

TEST_F(ElasticityElementTest, ElasticForceCompatibleWithItsDerivative) {
  SetupRandomState();
  Vector<AutoDiffXd, kNumDofs> neg_elastic_force =
      element().CalcNegativeElasticForce(*state_);
  Eigen::Matrix<AutoDiffXd, kNumDofs, kNumDofs> neg_elastic_force_derivative =
      element().CalcNegativeElasticForceDerivative(*state_);
  for (int i = 0; i < kNumDofs; ++i) {
    EXPECT_TRUE(CompareMatrices(neg_elastic_force(i).derivatives().transpose(),
                                neg_elastic_force_derivative.row(i),
                                std::numeric_limits<double>::epsilon()));
  }
}
}  // namespace
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
