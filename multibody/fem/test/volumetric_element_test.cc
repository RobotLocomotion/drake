#include "drake/multibody/fem/volumetric_element.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/fem/fem_state.h"
#include "drake/multibody/fem/linear_corotated_model.h"
#include "drake/multibody/fem/linear_simplex_element.h"
#include "drake/multibody/fem/simplex_gaussian_quadrature.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/force_density_field.h"

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
using ConstitutiveModelType = LinearCorotatedModel<AD>;
using DeformationGradientDataType =
    std::array<LinearCorotatedModelData<AD>, kNumQuads>;

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
          const Vector3<AD> dummy_weights(1, 2, 3);
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
    /* Set q0 to be the same position as q, but drop the derivatives. */
    const Eigen::VectorXd q0_double = math::ExtractValue(q);
    VectorX<AD> q0(q0_double.size());
    for (int i = 0; i < q0_double.size(); ++i) {
      q0(i) = q0_double(i);
    }
    fem_state->SetTimeStepPositions(q0);
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
    const Data& data = EvalElementData(fem_state);
    element().AddNegativeElasticForce(data.P, &neg_force);
    return neg_force;
  }

  /* Calculates the negative elastic force derivative with respect to positions
   for the only element evaluated with the given `fem_state`. */
  Eigen::Matrix<AD, kNumDofs, kNumDofs> CalcNegativeElasticForceDerivative(
      const FemState<AD>& fem_state) const {
    Eigen::Matrix<AD, kNumDofs, kNumDofs> neg_force_derivative =
        Eigen::Matrix<AD, kNumDofs, kNumDofs>::Zero();
    const Data& data = EvalElementData(fem_state);
    element().AddScaledElasticForceDerivative(data.dPdF, -1,
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
    for (int i = 0; i < kNumQuads; ++i) {
      deformation_gradient_data[i].UpdateData(F[i], F0[i]);
    }
    return deformation_gradient_data;
  }

  /* Calculates and verifies the energy and elastic forces evaluated with the
   given `data` are zero. */
  void VerifyEnergyAndForceAreZero(const FemState<AD>& fem_state) const {
    const Data& data = EvalElementData(fem_state);
    AD energy = element().CalcElasticEnergy(data.Psi);
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
    return element().mass_matrix_;
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
  EXPECT_TRUE(element().is_linear);
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
  fem_state->SetTimeStepPositions(fem_state->GetPositions());
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
  for (int q = 0; q < kNumQuads; ++q) {
    constitutive_model().CalcElasticEnergyDensity(deformation_gradient_data[q],
                                                  &energy_density_array[q]);
  }
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
  EXPECT_NEAR(
      element().CalcElasticEnergy(EvalElementData(*fem_state).Psi).value(),
      analytical_energy, kEpsilon);

  const auto neg_elastic_force_autodiff = CalcNegativeElasticForce(*fem_state);
  Vector<double, kNumDofs> neg_elastic_force =
      math::DiscardGradient(neg_elastic_force_autodiff);
  /* Force on node 0. */
  const Vector3<double> force0 = -neg_elastic_force.head<3>();
  /* The first piola stress. */
  std::array<Matrix3<AD>, kNumQuads> P_array;
  for (int q = 0; q < kNumQuads; ++q) {
    constitutive_model().CalcFirstPiolaStress(deformation_gradient_data[q],
                                              &P_array[q]);
  }
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
  AD energy = element().CalcElasticEnergy(EvalElementData(*fem_state).Psi);
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

/* Tests that when applying the gravity force density field, the result agrees
 with analytic solution. */
TEST_F(VolumetricElementTest, PerReferenceVolumeExternalForce) {
  unique_ptr<FemState<AD>> fem_state = MakeReferenceState();
  const auto& data = EvalElementData(*fem_state);
  const AD mass_density = 2.7;
  Vector3<AD> gravity_vector(0, 0, -9.81);
  GravityForceField<AD> gravity_field(gravity_vector, mass_density);
  /* The gravity force field doesn't depend on Context, but a Context is needed
   formally. So we create a dummy Context that's otherwise
   unused. */
  MultibodyPlant<AD> plant(0.01);
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  fem::FemPlantData<AD> plant_data{*context, {&gravity_field}};

  VectorX<AD> gravity_force = VectorX<AD>::Zero(kNumDofs);
  const AD scale = 1.0;
  element().AddScaledExternalForces(data, plant_data, scale, &gravity_force);
  Vector3<AD> total_force = Vector3<AD>::Zero();
  for (int i = 0; i < kNumNodes; ++i) {
    total_force += gravity_force.segment<3>(3 * i);
  }

  const Vector3<AD> expected_gravity_force =
      reference_volume()[0] * gravity_vector * mass_density;
  EXPECT_TRUE(CompareMatrices(total_force, expected_gravity_force, kEpsilon));
}

/* Tests that when applying a per current volume force density field, the result
 agrees with analytic solution. The only reason we use AutoDiffXd here is
 because we are reusing an AutoDiffXd fixture previously used to compute
 gradients. This unit test in particular does not use AutoDiffXd to
 compute gradients. */
TEST_F(VolumetricElementTest, PerCurrentVolumeExternalForce) {
  /* We scale the reference positions to get the current positions so we have
   precise control of the current volume. */
  const double scale = 1.23;
  const Eigen::Matrix<AD, kSpatialDimension, kNumNodes> X_matrix =
      reference_positions();
  const Eigen::Matrix<AD, kSpatialDimension, kNumNodes> x_matrix =
      scale * X_matrix;
  Vector<double, kNumDofs> x_double(Eigen::Map<Vector<double, kNumDofs>>(
      math::DiscardGradient(x_matrix).data(), x_matrix.size()));
  Vector<AD, kNumDofs> x(x_double);
  const Vector<AD, kNumDofs> v = Vector<AD, kNumDofs>::Zero();
  const Vector<AD, kNumDofs> a = Vector<AD, kNumDofs>::Zero();
  unique_ptr<FemState<AD>> fem_state = MakeFemState(x, v, a);
  const auto& data = EvalElementData(*fem_state);

  const Vector3<AD> force_per_current_volume(4, 5, 6);
  /* A constant per current volume force density field. */
  class ConstantForceDensityField final : public ForceDensityField<AD> {
   public:
    explicit ConstantForceDensityField(const Vector3<AD>& f) : f_(f) {}

   private:
    Vector3<AD> DoEvaluateAt(const systems::Context<AD>& context,
                             const Vector3<AD>& p_WQ) const final {
      return f_;
    };

    std::unique_ptr<ForceDensityFieldBase<AD>> DoClone() const final {
      return std::make_unique<ConstantForceDensityField>(*this);
    }

    const Vector3<AD> f_;
  };
  ConstantForceDensityField external_force_field(force_per_current_volume);
  /* The constant force field doesn't depend on Context, but a Context is needed
   formally. So we create a dummy Context that's otherwise unused. */
  MultibodyPlant<AD> plant(0.01);
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  fem::FemPlantData<AD> plant_data{*context, {&external_force_field}};

  VectorX<AD> external_force = VectorX<AD>::Zero(kNumDofs);
  element().AddScaledExternalForces(data, plant_data, 1.0, &external_force);
  Vector3<AD> total_force = Vector3<AD>::Zero();
  for (int i = 0; i < kNumNodes; ++i) {
    total_force += external_force.segment<3>(3 * i);
  }

  const AD current_volume = reference_volume()[0] * scale * scale * scale;
  const Vector3<AD> expected_force = current_volume * force_per_current_volume;
  EXPECT_TRUE(CompareMatrices(total_force, expected_force, kEpsilon));
}

TEST_F(VolumetricElementTest, CalcMassTimesPositionForQuadraturePoints) {
  unique_ptr<FemState<AD>> fem_state = MakeReferenceState();
  const auto& data = EvalElementData(*fem_state);

  Vector3<AD> moment = element().CalcMassTimesPositionForQuadraturePoints(data);

  /* For a single linear tet, there's only one quadrature point at the centroid.
   The reference_volume_[0] is the volume of the tetrahedron. */
  const AD expected_mass = density(element()) * reference_volume()[0];
  const Vector3<AD> expected_moment =
      expected_mass * data.quadrature_positions[0];
  EXPECT_TRUE(CompareMatrices(moment, expected_moment, kEpsilon));
}

TEST_F(VolumetricElementTest, CalcTranslationalMomentumForQuadraturePoints) {
  unique_ptr<FemState<AD>> fem_state = MakeReferenceState();
  // Set some non-zero velocities for testing momentum.
  VectorX<AD> v_all = fem_state->GetVelocities();
  for (int i = 0; i < kNumNodes; ++i) {
    v_all.segment<3>(i * 3) = Vector3<AD>(i + 1.0, i + 2.0, i + 3.0);
  }
  fem_state->SetVelocities(v_all);
  const auto& data = EvalElementData(*fem_state);

  Vector3<AD> translational_momentum =
      element().CalcTranslationalMomentumForQuadraturePoints(data);

  const AD expected_mass_term = density(element()) * reference_volume()[0];
  const Vector3<AD> expected_translational_momentum =
      expected_mass_term * data.quadrature_velocities[0];
  EXPECT_TRUE(CompareMatrices(translational_momentum,
                              expected_translational_momentum, kEpsilon));
}

TEST_F(VolumetricElementTest, CalcAngularMomentumAboutWorldOrigin) {
  /* Test with a pure translational motion for the element. */
  unique_ptr<FemState<AD>> fem_state = MakeReferenceState();
  VectorX<AD> v = VectorX<AD>::Zero(kNumDofs);
  const Vector3<AD> translational_velocity(1.0, 2.0, 3.0);
  for (int i = 0; i < kNumNodes; ++i) {
    v.segment<3>(i * 3) = translational_velocity;
  }
  fem_state->SetVelocities(v);
  const auto& data = EvalElementData(*fem_state);

  Vector3<AD> angular_momentum =
      element().CalcAngularMomentumAboutWorldOrigin(data);

  const Vector3<AD> p_WQ = data.quadrature_positions[0];
  const Vector3<AD> v_WQ = data.quadrature_velocities[0];
  const AD mass = density(element()) * reference_volume()[0];
  const Vector3<AD> expected_angular_momentum = mass * p_WQ.cross(v_WQ);

  EXPECT_TRUE(
      CompareMatrices(angular_momentum, expected_angular_momentum, kEpsilon));
}

TEST_F(VolumetricElementTest, CalcRotationalInertiaAboutWorldOrigin) {
  unique_ptr<FemState<AD>> fem_state = MakeReferenceState();
  const auto& data = EvalElementData(*fem_state);

  Matrix3<AD> inertia = element().CalcRotationalInertiaAboutWorldOrigin(data);

  const Vector3<AD> p_WQ = data.quadrature_positions[0];
  const AD mass = density(element()) * reference_volume()[0];
  const Matrix3<AD> expected_inertia =
      mass * p_WQ.dot(p_WQ) * Matrix3<AD>::Identity() -
      mass * p_WQ * p_WQ.transpose();

  EXPECT_TRUE(CompareMatrices(inertia, expected_inertia, kEpsilon));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
