#include "drake/multibody/fem/volumetric_model.h"

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/fem/acceleration_newmark_scheme.h"
#include "drake/multibody/fem/fem_state.h"
#include "drake/multibody/fem/linear_constitutive_model.h"
#include "drake/multibody/fem/linear_simplex_element.h"
#include "drake/multibody/fem/simplex_gaussian_quadrature.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using std::unique_ptr;

constexpr int kNaturalDimension = 3;
constexpr int kSpatialDimension = 3;
constexpr int kQuadratureOrder = 1;
using QuadratureType =
    internal::SimplexGaussianQuadrature<kNaturalDimension, kQuadratureOrder>;
constexpr int kNumQuads = QuadratureType::num_quadrature_points;

using AutoDiffIsoparametricElement =
    internal::LinearSimplexElement<AutoDiffXd, kNaturalDimension,
                                   kSpatialDimension, kNumQuads>;
using AutoDiffConstitutiveModel = internal::LinearConstitutiveModel<AutoDiffXd>;
using AutoDiffElement =
    VolumetricElement<AutoDiffIsoparametricElement, QuadratureType,
                      AutoDiffConstitutiveModel>;

using DoubleIsoparametricElement =
    internal::LinearSimplexElement<double, kNaturalDimension, kSpatialDimension,
                                   kNumQuads>;
using DoubleConstitutiveModel = internal::LinearConstitutiveModel<double>;
using DoubleElement =
    VolumetricElement<DoubleIsoparametricElement, QuadratureType,
                      DoubleConstitutiveModel>;

const double kYoungsModulus = 12345;
const double kPoissonRatio = 0.456;
const double kDensity = 0.789;
/* The geometry of the model under test is a cube and it has 8 vertices and 6
 elements. */
constexpr int kNumCubeVertices = 8;
constexpr int kNumDofs = kNumCubeVertices * kSpatialDimension;
constexpr int kNumElements = 6;
/* Parameters for Newmark scheme. */
const double kDt = 1e-3;
const double kGamma = 0.5;
const double kBeta = 0.25;
/* Parameters for the damping model. */
const double kMassDamping = 0.01;
const double kStiffnessDamping = 0.02;

class VolumetricModelTest : public ::testing::Test {
 protected:
  static constexpr double kBoxLength = 0.1;

  /* Makes a box and subdivides it into 6 tetrahedra. */
  geometry::VolumeMesh<double> MakeBoxTetMesh() {
    geometry::Box box(kBoxLength, kBoxLength, kBoxLength);
    geometry::VolumeMesh<double> mesh =
        geometry::internal::MakeBoxVolumeMesh<double>(box, kBoxLength);
    DRAKE_DEMAND(mesh.num_elements() == 6);
    return mesh;
  }

  /* Adds a FEM model of a box discretized into 6 tetrahedra into the given
   `fem_model`. */
  template <typename FemModelType>
  void AddBoxToModel(FemModelType* fem_model) {
    geometry::VolumeMesh<double> mesh = MakeBoxTetMesh();
    using T = typename FemModelType::T;
    const typename FemModelType::ConstitutiveModel constitutive_model(
        kYoungsModulus, kPoissonRatio);
    const DampingModel<T> damping_model(kMassDamping, kStiffnessDamping);
    typename FemModelType::VolumetricBuilder builder(fem_model);
    builder.AddLinearTetrahedralElements(mesh, constitutive_model, kDensity,
                                         damping_model);
    builder.Build();
  }

  void SetUp() override { AddBoxToModel(&model_); }

  /* Returns an arbitrary vector of the given size. */
  static VectorX<double> perturbation(int size) {
    VectorX<double> delta(size);
    for (int i = 0; i < size; ++i) {
      delta(i) = 0.01 * i;
    }
    return delta;
  }

  /* Returns an arbitrary FEM data whose generalized positions are different
   from reference positions and whose velocities and accelerations are nonzero.
   In addition, set up autodiff derivatives for accelerations if the scalar type
   is AutoDiffXd. */
  template <typename FemModelType>
  unique_ptr<FemState<typename FemModelType::T>> MakeDeformedFemState(
      const FemModelType& fem_model) {
    using T = typename FemModelType::T;
    const int num_dofs = fem_model.num_dofs();
    if constexpr (std::is_same_v<T, AutoDiffXd>) {
      unique_ptr<FemState<T>> reference_fem_state = fem_model.MakeFemState();
      unique_ptr<FemState<T>> deformed_fem_state = fem_model.MakeFemState();
      /* Perturb a. */
      const VectorX<double> perturbed_a =
          math::ExtractValue(reference_fem_state->GetAccelerations()) +
          perturbation(num_dofs);
      /* Set up AutodiffXd derivatives. */
      VectorX<AutoDiffXd> perturbed_a_autodiff(num_dofs);
      math::InitializeAutoDiff(perturbed_a, &perturbed_a_autodiff);
      /* It's important to set up the `deformed_fem_state` with
       AdvanceOneTimeStep() so that the derivatives such as dq/da are set up. */
      autodiff_integrator_.AdvanceOneTimeStep(
          *reference_fem_state, perturbed_a_autodiff, deformed_fem_state.get());
      return deformed_fem_state;
    } else {
      unique_ptr<FemState<T>> reference_fem_state = fem_model.MakeFemState();
      unique_ptr<FemState<T>> deformed_fem_state = fem_model.MakeFemState();
      /* Perturb a. */
      const VectorX<double> perturbed_a =
          reference_fem_state->GetAccelerations() + perturbation(num_dofs);
      double_integrator_.AdvanceOneTimeStep(*reference_fem_state, perturbed_a,
                                            deformed_fem_state.get());
      return deformed_fem_state;
    }
    DRAKE_UNREACHABLE();
  }

  AccelerationNewmarkScheme<AutoDiffXd> autodiff_integrator_{kDt, kGamma,
                                                             kBeta};
  AccelerationNewmarkScheme<double> double_integrator_{kDt, kGamma, kBeta};
  /* The model under test. */
  VolumetricModel<AutoDiffElement> model_{autodiff_integrator_.GetWeights()};
};

/* Tests the mesh has been successfully converted to elements. */
TEST_F(VolumetricModelTest, Geometry) {
  EXPECT_EQ(model_.num_nodes(), kNumCubeVertices);
  EXPECT_EQ(model_.num_elements(), kNumElements);
}

/* Tests that the tangent matrix of the model is the derivative of the residual
 with respect to the change in a. Note that this is only true for the linear
 model used in this test (since in general the contribution from damping terms
 is only an approximation). */
TEST_F(VolumetricModelTest, TangentMatrixIsResidualDerivative) {
  unique_ptr<FemState<AutoDiffXd>> autodiff_state =
      MakeDeformedFemState(model_);
  VectorX<AutoDiffXd> residual(autodiff_state->num_dofs());
  const systems::LeafContext<AutoDiffXd> dummy_context;
  const FemPlantData<AutoDiffXd> dummy_data{dummy_context, {}};
  model_.CalcResidual(*autodiff_state, dummy_data, &residual);

  VolumetricModel<DoubleElement> double_model(double_integrator_.GetWeights());
  AddBoxToModel(&double_model);
  unique_ptr<FemState<double>> double_state =
      MakeDeformedFemState(double_model);
  auto tangent_matrix = double_model.MakeTangentMatrix();
  double_model.CalcTangentMatrix(*double_state, tangent_matrix.get());
  const MatrixXd tangent_matrix_dense = tangent_matrix->MakeDenseMatrix();
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(residual),
                              tangent_matrix_dense,
                              4 * std::numeric_limits<double>::epsilon()));
}

/* Adds two copies of the same set of elements to test that the node offsets in
 AddLinearTetrahedralElements() are working as intended. */
TEST_F(VolumetricModelTest, MultipleMesh) {
  /* Add a second box mesh to the model using a separate builder. */
  AddBoxToModel(&model_);
  EXPECT_EQ(model_.num_nodes(), 2 * kNumCubeVertices);
  EXPECT_EQ(model_.num_elements(), 2 * kNumElements);
  unique_ptr<FemState<AutoDiffXd>> fem_state = MakeDeformedFemState(model_);
  EXPECT_EQ(fem_state->num_dofs(), 2 * kNumDofs);

  /* Add two more meshes to the model using a the same builder. */
  geometry::VolumeMesh<double> mesh = MakeBoxTetMesh();
  const AutoDiffConstitutiveModel constitutive_model(kYoungsModulus,
                                                     kPoissonRatio);
  const DampingModel<AutoDiffXd> damping_model(kMassDamping, kStiffnessDamping);
  VolumetricModel<AutoDiffElement>::VolumetricBuilder builder(&model_);
  builder.AddLinearTetrahedralElements(mesh, constitutive_model, kDensity,
                                       damping_model);
  builder.AddLinearTetrahedralElements(mesh, constitutive_model, kDensity,
                                       damping_model);
  builder.Build();
  /* Now there should be 4 boxes in the model. */
  EXPECT_EQ(model_.num_nodes(), 4 * kNumCubeVertices);
  EXPECT_EQ(model_.num_elements(), 4 * kNumElements);
  unique_ptr<FemState<AutoDiffXd>> reference_fem_state = model_.MakeFemState();
  EXPECT_EQ(reference_fem_state->num_dofs(), 4 * kNumDofs);
  const VectorX<AutoDiffXd>& q = reference_fem_state->GetPositions();
  /* The reference positions of the four boxes should be the same. */
  EXPECT_TRUE(
      CompareMatrices(q.head<kNumDofs>(), q.segment<kNumDofs>(kNumDofs)));
  EXPECT_TRUE(
      CompareMatrices(q.head<kNumDofs>(), q.segment<kNumDofs>(2 * kNumDofs)));
  EXPECT_TRUE(
      CompareMatrices(q.head<kNumDofs>(), q.segment<kNumDofs>(3 * kNumDofs)));
}

TEST_F(VolumetricModelTest, ElasticEnergy) {
  using DoubleModel = VolumetricModel<DoubleElement>;
  DoubleModel double_model(double_integrator_.GetWeights());
  AddBoxToModel(&double_model);
  unique_ptr<FemState<double>> state = double_model.MakeFemState();
  /* Inflate the elements so that the infinitestimal strain E = I. */
  state->SetPositions(state->GetPositions() * 2.0);
  const Matrix3d strain = Matrix3d::Identity();
  const double energy = double_model.CalcElasticEnergy(*state);
  /* The expected energy is the energy density integrated over the reference
   volume. */
  const double volume = kBoxLength * kBoxLength * kBoxLength;
  const DoubleModel::ConstitutiveModel constitutive_model(kYoungsModulus,
                                                          kPoissonRatio);
  const double mu = constitutive_model.shear_modulus();
  const double lambda = constitutive_model.lame_first_parameter();
  const double energy_density = mu * strain.squaredNorm() +
                                0.5 * lambda * strain.trace() * strain.trace();
  const double expected_energy = energy_density * volume;
  EXPECT_DOUBLE_EQ(energy, expected_energy);
}

TEST_F(VolumetricModelTest, Clone) {
  using DoubleModel = VolumetricModel<DoubleElement>;
  DoubleModel double_model(double_integrator_.GetWeights());
  AddBoxToModel(&double_model);

  std::unique_ptr<FemModel<double>> clone = double_model.Clone();
  const DoubleModel* double_clone =
      dynamic_cast<const DoubleModel*>(clone.get());
  ASSERT_NE(double_clone, nullptr);

  std::unique_ptr<FemState<double>> state = MakeDeformedFemState(double_model);
  std::unique_ptr<FemState<double>> clone_state =
      MakeDeformedFemState(*double_clone);

  EXPECT_EQ(state->GetPositions(), clone_state->GetPositions());
  EXPECT_EQ(state->GetPreviousStepPositions(),
            clone_state->GetPreviousStepPositions());
  EXPECT_EQ(state->GetVelocities(), clone_state->GetVelocities());
  EXPECT_EQ(state->GetAccelerations(), clone_state->GetAccelerations());
  EXPECT_EQ(state->num_dofs(), clone_state->num_dofs());
  EXPECT_EQ(state->num_nodes(), clone_state->num_nodes());
}

/* Tests the get_total_mass() function for VolumetricModel. */
TEST_F(VolumetricModelTest, TotalMass) {
  using DoubleModel = VolumetricModel<DoubleElement>;
  DoubleModel single_box_model(double_integrator_.GetWeights());
  AddBoxToModel(&single_box_model);
  /* Calculate expected mass: density * volume. */
  const double volume = kBoxLength * kBoxLength * kBoxLength;
  const double expected_mass = kDensity * volume;
  EXPECT_DOUBLE_EQ(single_box_model.get_total_mass(), expected_mass);
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
