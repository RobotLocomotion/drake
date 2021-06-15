#include "drake/multibody/contact_solvers/contact_solver.h"

#include <iostream>
#include <memory>

#include <Eigen/SparseCore>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"
#include "drake/multibody/contact_solvers/test/multibody_sim_driver.h"
#include "drake/multibody/plant/contact_results.h"

namespace drake {
namespace multibody {

using test::MultibodySimDriver;

namespace contact_solvers {
namespace internal {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransformd;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// This class implements a ContactSolver that can only solve a very specific
// problem, that of a mass particle making contact with the ground.
// This allow us to exercise ContactSolver's APIs and test some internals such
// as the computation of the Delassus operator.
// In these tests we also verify the proper data flow from MultibodyPlant into
// the contact solver and out of MultibodyPlant as contact results.
template <typename T>
class ParticleSolver final : public ContactSolver<T> {
 public:
  // This solver knows about the problem it is trying to solve. Therefore its
  // constructor takes as input the mass of the particle in the problem.
  explicit ParticleSolver(double mass) : mass_(mass) {}

  virtual ~ParticleSolver() = default;

  // This implementation of SolveWithGuess() in addition makes a number of tests
  // on the data supplied by MultibodyPlant. We must perform these tests at this
  // scope to ensure data references are still "alive".
  ContactSolverStatus SolveWithGuess(const T& time_step,
                                     const SystemDynamicsData<T>& dynamics_data,
                                     const PointContactData<T>& contact_data,
                                     const VectorX<T>& v_guess,
                                     ContactSolverResults<T>* results) final {
    const int nv = dynamics_data.num_velocities();
    const int nc = contact_data.num_contacts();

    // Aliases to problem data.
    const LinearOperator<T>& Ainv = dynamics_data.get_Ainv();
    const VectorX<T>& v_star = dynamics_data.get_v_star();
    const LinearOperator<T>& Jc = contact_data.get_Jc();

    // Verify the expected Delassus operator is computed by
    // ContactSolver::FormDelassusOperatorMatrix().
    Eigen::SparseMatrix<T> Ws(3 * nc, 3 * nc);
    ContactSolver<T>::FormDelassusOperatorMatrix(Jc, Ainv, Jc, &Ws);
    // Only one out of nine coefficients is non-zero.
    EXPECT_EQ(Ws.nonZeros(), 3);
    const Matrix3<T> W = Matrix3<T>(Ws);
    const Matrix3d W_expected = Matrix3<T>::Identity() / mass_;
    EXPECT_TRUE(
        CompareMatrices(W, W_expected, kEpsilon, MatrixCompareType::absolute));

    // Generalized velocities when contact forces are zero.
    VectorX<T> vc_star(3 * nc);
    Jc.Multiply(v_star, &vc_star);  // vc_star = Jc⋅v_star
    const T vn_star = vc_star(2);   // Normal velocity.
    const T Wnn = W(2, 2);          // Normal equation.
    // We now need to solve the 1D  problem:
    //   0 ≤ Wnn π + vₙ* ⊥ π ≥ 0
    // Which, given Wnn > 0, has unique solution:
    //   π = -vₙ*/Wnn, if vₙ* < 0.
    //   π = 0       , if vₙ* > 0.
    const T pi = vn_star < 0 ? -vn_star / Wnn : 0.0;
    const T vn = vn_star < 0 ? 0.0 : vn_star;

    // With pi known, now we can solve for the tangential component, beta.
    EXPECT_EQ(contact_data.get_mu().size(), 1);  // since nc = 1.
    const T mu = contact_data.get_mu()(0);
    const Vector2<T> vt_star = vc_star.template head<2>();
    // First, assume stiction: vt=0 ==> beta = -vt_star/Wtt.
    const T Wtt = W(0, 0);  // For this case W(0, 0) = W(1, 1).
    Vector2<T> beta = -vt_star / Wtt;
    if (beta.norm() > mu * pi) {
      // The stiction assumption is violated, then we are in sliding.
      const Vector2<T> vt_star_hat = vt_star.normalized();
      beta = -mu * pi * vt_star_hat;
    }
    const Vector2<T> vt = Wtt * beta + vt_star;

    // Update the solver's state.
    gamma_ = Vector3<T>(beta(0), beta(1), pi);

    Jc.MultiplyByTranspose(gamma_, &jc_);
    Ainv.Multiply(jc_, &v_);  // v_ = M⁻¹⋅τc
    v_ += v_star;             // v_ = v* + M⁻¹⋅τc
    vc_ = Vector3<T>(vt(0), vt(1), vn);

    // Pack solution as ContactSolverResults.
    results->Resize(nv, nc);
    results->v_next = v_;
    ExtractNormal(vc_, &results->vn);
    ExtractTangent(vc_, &results->vt);
    ExtractNormal(gamma_, &results->fn);
    ExtractTangent(gamma_, &results->ft);
    // N.B. While contact solver works with impulses, results are reported as
    // forces.
    results->fn /= time_step;
    results->ft /= time_step;
    results->tau_contact = jc_ / time_step;

    return ContactSolverStatus::kSuccess;  // It always succeeds.
  }

 private:
  // The solver's state.
  VectorX<T> gamma_{3};  // Contact impulse.
  VectorX<T> v_{6};      // Generalized velocity, in this cases equals vn.
  // Cache, i.e. state dependent quantities.
  VectorX<T> jc_{6};  // Generalized contact impulse.
  VectorX<T> vc_{3};  // Contact velocity.

  // Problem parameters, for unit testing, an actual solver won't know
  // these.
  const T mass_{0.5};
};

class ParticleTest : public ::testing::Test {
 public:
  void SetUp() override {
    const std::string model_file =
        "drake/multibody/contact_solvers/test/"
        "particle_with_infinite_inertia.sdf";
    driver_.BuildModel(dt_, model_file);
    const auto& plant = driver_.plant();
    const auto& particle = plant.GetBodyByName("particle");
    // Add the ground, with the same friction as specified in the SDF file for
    // the particle.
    const double mu = driver_.GetDynamicFrictionCoefficients(particle)[0];
    driver_.AddGround(0.0 /*not used*/, 0.0 /*not used*/, mu);
    driver_.Initialize();
    const int nq = plant.num_positions();
    const int nv = plant.num_velocities();
    // Assert plant sizes.
    ASSERT_EQ(nq, 7);  // 4 dofs for a quaternion, 3 dofs for translation.
    ASSERT_EQ(nv, 6);  // 6 dofs for angular and translational velocity.
    auto solver = std::make_unique<ParticleSolver<double>>(kParticleMass);
    solver_ = solver.get();
    driver_.mutable_plant().SetContactSolver(std::move(solver));

    // Verify that solvers/test/particle.sdf is in sync with this test.
    const double mass = particle.get_default_mass();
    ASSERT_NEAR(mass, kParticleMass, kEpsilon);

    // MultibodyPlant state.
    SetInitialState();
  }

  // Set the particle to be in contact with the ground.
  void SetInitialState() {
    const auto& plant = driver_.plant();
    const auto& particle = plant.GetBodyByName("particle");
    auto& context = driver_.mutable_plant_context();
    const Vector3d p_WB(0, 0, -kPenetrationDistance);
    plant.SetFreeBodyPose(&context, particle, math::RigidTransformd(p_WB));
    plant.SetFreeBodySpatialVelocity(&context, particle,
                                     SpatialVelocity<double>::Zero());
  }

  // Helper to retrieve contact results. Notice that this method will trigger
  // the computation of contact results through MultibodyPlant and therefore,
  // unless results were previously cached by MultibodyPlant, this method will
  // result in the invocation of ParticleSolver::SolveWithGuess().
  // This allow us to verify:
  //  - Proper invocation of the contact solver.
  //  - The proper data flow out from MultibodyPlant as contact results.
  const PointPairContactInfo<double>& EvalPointPairInfo() const {
    // Only one contact pair.
    const ContactResults<double>& contact_results = driver_.GetContactResults();
    DRAKE_DEMAND(contact_results.num_point_pair_contacts() == 1u);
    const PointPairContactInfo<double>& point_pair_contact_info =
        contact_results.point_pair_contact_info(0);
    return point_pair_contact_info;
  }

  // Helper to verify contact results.
  // @param f_Pc_W_expected expected contact force on the particle P at the
  // contact point C, expressed in the world frame W.
  // @param slip_expected The expected slip velocity.
  void VerifyContactInfo(const Vector3d& f_Pc_W_expected,
                         double slip_expected) const {
    const auto& particle = driver_.plant().GetBodyByName("particle");
    // We'll verify the expected results and therefore that MultibodyPlant was
    // able to properly load the contact results.
    const PointPairContactInfo<double>& info = EvalPointPairInfo();
    double direction = info.bodyB_index() == particle.index() ? 1.0 : -1.0;
    // Force on the particle P at contact point C.
    const Vector3d f_Pc_W = direction * info.contact_force();

    // Verify values.
    EXPECT_TRUE(CompareMatrices(f_Pc_W, f_Pc_W_expected, kEpsilon,
                                MatrixCompareType::absolute));
    EXPECT_NEAR(info.slip_speed(), slip_expected, kEpsilon);
    EXPECT_NEAR(info.separation_speed(), 0.0, kEpsilon);
  }

  // In addition to contact results, MultibodyPlant also reports contact
  // computations as generalized forces. This helper verifies those results.
  // @param f_Pc_W_expected expected contact force on the particle P at the
  // contact point C, expressed in world frame W.
  void VerifyGeneralizedContactForces(const Vector3d& f_Pc_W_expected) const {
    const auto& particle = driver_.plant().GetBodyByName("particle");
    const auto tau_c = driver_.plant()
                           .get_generalized_contact_forces_output_port(
                               particle.model_instance())
                           .Eval(driver_.plant_context());
    // The last three components should correspond to the expected force on P.
    EXPECT_TRUE(CompareMatrices(tau_c.tail<3>(), f_Pc_W_expected, kEpsilon,
                                MatrixCompareType::absolute));

    // The rotational component is non-zero given the non-zero offset location
    // of the contact point.
    const double phi0 = kPenetrationDistance;
    // MultibodyPlant places C midway between the two interacting bodies.
    const Vector3d p_PoC_W(0, 0, phi0 / 2.0);
    const Vector3d t_P_W = p_PoC_W.cross(f_Pc_W_expected);
    EXPECT_TRUE(CompareMatrices(tau_c.head<3>(), t_P_W, kEpsilon,
                                MatrixCompareType::absolute));
  }

 protected:
  const double dt_{1.0e-3};
  const double kParticleMass{0.5};
  const double kPenetrationDistance{1.2e-2};
  MultibodySimDriver driver_;
  ParticleSolver<double>* solver_{nullptr};
};

// We test two conditions, stiction and sliding to verify the proper data flow.
TEST_F(ParticleTest, Stiction) {
  // Apply an in plane force:
  const Vector3d fapplied_P_W(2.0, 0.0, 0.0);
  const auto& particle = driver_.plant().GetBodyByName("particle");
  driver_.FixAppliedForce(particle, fapplied_P_W);

  // In stiction we expect the contact force to exactly balance the external
  // forces.
  const Vector3d weight_P_W(0.0, 0.0, -5.0);
  const Vector3d f_Pc_W_expected = -(weight_P_W + fapplied_P_W);

  VerifyContactInfo(f_Pc_W_expected, 0.0);
  VerifyGeneralizedContactForces(f_Pc_W_expected);
}

TEST_F(ParticleTest, Sliding) {
  const Vector3d fapplied_P_W(3.0, 0.0, 0.0);
  const auto& particle = driver_.plant().GetBodyByName("particle");
  driver_.FixAppliedForce(particle, fapplied_P_W);

  // The maximum friction force is mu * Weight = 2.5 N.
  const Vector3d weight_P_W(0.0, 0.0, -5.0);
  const Vector3d friction_P_W(-2.5, 0.0, 0.0);
  const Vector3d f_Pc_W_expected = -weight_P_W + friction_P_W;

  // Since the maximum friction force is mu * Weight = 2.5 N, the total force in
  // x will be 3N - 2.5N = 0.5N. Over a dt = 1e-3 sec, will cause velocity to
  // accelerate from zero to v = 1e-3 m/s (mass = 0.5 Kg) in the x direction.
  double slip_expected = 1.0e-3;

  VerifyContactInfo(f_Pc_W_expected, slip_expected);
  VerifyGeneralizedContactForces(f_Pc_W_expected);
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
