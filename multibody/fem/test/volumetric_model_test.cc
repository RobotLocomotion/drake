#include "drake/multibody/fem/volumetric_model.h"

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
using AutoDiffConstitutiveModel =
    internal::LinearConstitutiveModel<AutoDiffXd, kNumQuads>;
using AutoDiffElement =
    VolumetricElement<AutoDiffIsoparametricElement, QuadratureType,
                      AutoDiffConstitutiveModel>;

using DoubleIsoparametricElement =
    internal::LinearSimplexElement<double, kNaturalDimension, kSpatialDimension,
                                   kNumQuads>;
using DoubleConstitutiveModel =
    internal::LinearConstitutiveModel<double, kNumQuads>;
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
  /* Makes a box and subdivides it into 6 tetrahedra. */
  template <typename T>
  geometry::VolumeMesh<T> MakeBoxTetMesh() {
    const double length = 0.1;
    geometry::Box box(length, length, length);
    geometry::VolumeMesh<T> mesh =
        geometry::internal::MakeBoxVolumeMesh<T>(box, length);
    DRAKE_DEMAND(mesh.num_elements() == 6);
    return mesh;
  }

  /* Adds a FEM model of a box discretized into 6 tetrahedra into the given
   `fem_model`. */
  template <typename FemModelType>
  void AddBoxToModel(FemModelType* fem_model) {
    using T = typename FemModelType::T;
    geometry::VolumeMesh<T> mesh = MakeBoxTetMesh<T>();
    const typename FemModelType::ConstitutiveModel constitutive_model(
        kYoungsModulus, kPoissonRatio);
    const DampingModel<T> damping_model(kMassDamping, kStiffnessDamping);
    typename FemModelType::VolumetricBuilder builder(fem_model);
    builder.AddVolumetricElementsFromTetMesh(mesh, constitutive_model, kDensity,
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
   from reference positions and whose velocities and acclerations are nonzero.
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

  /* The model under test. */
  VolumetricModel<AutoDiffElement> model_{};
  AccelerationNewmarkScheme<AutoDiffXd> autodiff_integrator_{kDt, kGamma,
                                                             kBeta};
  AccelerationNewmarkScheme<double> double_integrator_{kDt, kGamma, kBeta};
};

/* Tests the mesh has been successfully converted to elements. */
TEST_F(VolumetricModelTest, Geometry) {
  EXPECT_EQ(model_.num_nodes(), kNumCubeVertices);
  EXPECT_EQ(model_.num_elements(), kNumElements);
}

/* Tests that the tangent matrix of the model is the derivative of the residual
 with respect to the change in a. */
TEST_F(VolumetricModelTest, TangentMatrixIsResidualDerivative) {
  unique_ptr<FemState<AutoDiffXd>> autodiff_state =
      MakeDeformedFemState(model_);
  VectorX<AutoDiffXd> residual(autodiff_state->num_dofs());
  model_.CalcResidual(*autodiff_state, &residual);

  VolumetricModel<DoubleElement> double_model;
  AddBoxToModel(&double_model);
  unique_ptr<FemState<double>> double_state =
      MakeDeformedFemState(double_model);
  auto tangent_matrix =
      double_model.MakePetscSymmetricBlockSparseTangentMatrix();
  double_model.CalcTangentMatrix(*double_state, double_integrator_.GetWeights(),
                                 tangent_matrix.get());
  tangent_matrix->AssembleIfNecessary();
  const MatrixXd tangent_matrix_dense = tangent_matrix->MakeDenseMatrix();

  for (int i = 0; i < autodiff_state->num_dofs(); ++i) {
    /* The tangent matrix should be the derivative of the residual. Notice that
     here we are comparing a VectorX<double> and the value of a
     VectorX<AutoDiffXd>. */
    EXPECT_TRUE(CompareMatrices(residual(i).derivatives(),
                                tangent_matrix_dense.col(i),
                                4 * std::numeric_limits<double>::epsilon()));
  }
}

/* Adds two copies of the same set of elements to test that the node offsets in
 AddVolumetricElementsFromTetMesh() are working as intended. */
TEST_F(VolumetricModelTest, MultipleMesh) {
  /* Add a second box mesh to the model. */
  AddBoxToModel(&model_);
  EXPECT_EQ(model_.num_nodes(), 2 * kNumCubeVertices);
  /* Each cube is split into 6 tetrahedra. */
  EXPECT_EQ(model_.num_elements(), 2 * kNumElements);

  unique_ptr<FemState<AutoDiffXd>> fem_state = MakeDeformedFemState(model_);
  EXPECT_EQ(fem_state->num_dofs(), 2 * kNumDofs);
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
