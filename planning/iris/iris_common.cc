#include "drake/planning/iris/iris_common.h"

#include <common_robotics_utilities/parallelism.hpp>

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"

namespace drake {
namespace planning {

IrisParameterizationFunction::IrisParameterizationFunction(
    const multibody::RationalForwardKinematics* kin,
    const Eigen::Ref<const Eigen::VectorXd>& q_star_val) {
  const int dimension = kin->plant().num_positions();
  DRAKE_DEMAND(dimension > 0);

  parameterization_double_ =
      [kin, q_star_captured =
                Eigen::VectorXd(q_star_val)](const Eigen::VectorXd& s_val) {
        return kin->ComputeQValue(s_val, q_star_captured);
      };
  parameterization_autodiff_ = [kin,
                                q_star_captured = Eigen::VectorXd(q_star_val)](
                                   const Eigen::VectorX<AutoDiffXd>& s_val) {
    return kin->ComputeQValue(s_val, q_star_captured);
  };

  parameterization_is_threadsafe_ = true;
  parameterization_dimension_ = dimension;
}

IrisParameterizationFunction::IrisParameterizationFunction(
    const Eigen::VectorX<symbolic::Expression>& expression_parameterization,
    const Eigen::VectorX<symbolic::Variable>& variables) {
  // First, we check that the variables in expression_parameterization match
  // the user-supplied variables.
  symbolic::Variables expression_variables;
  for (const auto& expression : expression_parameterization) {
    expression_variables.insert(expression.GetVariables());
  }
  symbolic::Variables user_supplied_variables(variables);
  DRAKE_THROW_UNLESS(expression_variables == user_supplied_variables);

  // Check for duplicates in variables.
  DRAKE_THROW_UNLESS(variables.size() == ssize(user_supplied_variables));

  int dimension = ssize(expression_variables);

  // Note that in this lambda, we copy the shared_ptr variables, ensuring that
  // variables is kept alive without making a copy of the individual Variable
  // objects (which would break the substitution machinery).
  parameterization_double_ =
      [expression_parameterization_captured =
           Eigen::VectorX<symbolic::Expression>(expression_parameterization),
       variables_captured = Eigen::VectorX<symbolic::Variable>(variables)](
          const Eigen::VectorXd& q) {
        DRAKE_ASSERT(q.size() == variables_captured.size());
        symbolic::Environment env;
        for (int i = 0; i < q.size(); ++i) {
          env.insert(variables_captured[i], q[i]);
        }
        Eigen::VectorXd out = expression_parameterization_captured.unaryExpr(
            [&env](const symbolic::Expression& expression) {
              return expression.Evaluate(env);
            });
        return out;
      };
  // Since Expression cannot be evaluated on type AutoDiffXd, we currently
  // cannot support a VectorX<AutoDiffXd> parameterization.
  parameterization_autodiff_ = nullptr;

  parameterization_is_threadsafe_ = true;
  parameterization_dimension_ = dimension;
}

namespace internal {

using common_robotics_utilities::parallelism::DegreeOfParallelism;
using common_robotics_utilities::parallelism::DynamicParallelForIndexLoop;
using common_robotics_utilities::parallelism::ParallelForBackend;
using common_robotics_utilities::parallelism::StaticParallelForRangeLoop;
using common_robotics_utilities::parallelism::ThreadWorkRange;
using geometry::optimization::Hyperellipsoid;
using geometry::optimization::VPolytope;

int unadaptive_test_samples(double epsilon, double delta, double tau) {
  return static_cast<int>(-2 * std::log(delta) / (tau * tau * epsilon) + 0.5);
}
float calc_delta_min(double delta, int max_iterations) {
  return delta * 6 / (M_PI * M_PI * max_iterations * max_iterations);
}

// Add the tangent to the (scaled) ellipsoid at @p point as a
// constraint.
void AddTangentToPolytope(
    const geometry::optimization::Hyperellipsoid& E,
    const Eigen::Ref<const Eigen::VectorXd>& point,
    double configuration_space_margin, bool relax_margin,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>* A,
    Eigen::VectorXd* b, int* num_constraints) {
  while (*num_constraints >= A->rows()) {
    // Increase pre-allocated polytope size.
    A->conservativeResize(A->rows() * 2, A->cols());
    b->conservativeResize(b->rows() * 2);
  }
  const Eigen::VectorXd a_face =
      (E.A().transpose() * E.A() * (point - E.center())).normalized();
  double b_point = a_face.transpose() * point;
  double b_face = b_point - configuration_space_margin;
  double b_center = a_face.transpose() * E.center();

  // Check if the face cuts off the center.
  if (b_center > b_face) {
    if (relax_margin) {
      // If the user has allowed relaxing the configuration space margin, we set
      // the hyperplane halfway between the point and the center.
      // | b-margin  ...  O| center, b-margin+relaxation ...
      b_face = (b_point + b_center) / 2.0;
    } else {
      throw std::logic_error(
          "The current center of the IRIS region is within "
          "sampled_iris_options.configuration_space_margin of being "
          "infeasible. Check your sample point and/or any additional "
          "constraints you've passed in via the options. The configuration "
          "space surrounding the sample point must have an interior.");
    }
  }

  A->row(*num_constraints) = a_face.transpose();
  (*b)[*num_constraints] = b_face;
  *num_constraints += 1;

  // Resize A matrix if we need more faces.
  if (A->rows() <= *num_constraints) {
    A->conservativeResize(A->rows() * 2, A->cols());
    b->conservativeResize(b->rows() * 2);
  }
}

void AddTangentToPolytope(
    const geometry::optimization::Hyperellipsoid& E,
    const Eigen::Ref<const Eigen::VectorXd>& point, const VPolytope& cvxh_vpoly,
    const solvers::SolverInterface& solver, double configuration_space_margin,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>* A,
    Eigen::VectorXd* b, int* num_constraints, double* max_relaxation) {
  Eigen::VectorXd a_face =
      internal::ComputeFaceTangentToDistCvxh(E, point, cvxh_vpoly, solver);
  a_face.normalize();
  double b_face = a_face.transpose() * point - configuration_space_margin;

  // Relax cspace margin to contain points.
  const Eigen::VectorXd result = a_face.transpose() * cvxh_vpoly.vertices();
  const double relaxation = result.maxCoeff() - b_face;
  if (relaxation > 0) {
    b_face += relaxation;
    if (*max_relaxation < relaxation) {
      *max_relaxation = relaxation;
    }
  }
  A->row(*num_constraints) = a_face.transpose();
  (*b)(*num_constraints) = b_face;
  *num_constraints += 1;

  // Resize A matrix if we need more faces.
  if (A->rows() <= *num_constraints) {
    A->conservativeResize(A->rows() * 2, A->cols());
    b->conservativeResize(b->rows() * 2);
  }
}

bool CheckProgConstraints(const solvers::MathematicalProgram* prog_ptr,
                          const Eigen::VectorXd& particle, const double tol) {
  if (!prog_ptr) {
    return true;
  }
  for (const auto& binding : prog_ptr->GetAllConstraints()) {
    DRAKE_ASSERT(binding.evaluator() != nullptr);
    if (!binding.evaluator()->CheckSatisfied(particle, tol)) {
      return false;
    }
  }
  return true;
}

std::vector<uint8_t> CheckProgConstraintsParallel(
    const solvers::MathematicalProgram* prog_ptr,
    const std::vector<Eigen::VectorXd>& particles,
    const Parallelism& parallelism, const double tol,
    std::optional<int> end_index) {
  const int actual_end_index = end_index.value_or(ssize(particles));
  DRAKE_DEMAND(actual_end_index >= 0 && actual_end_index <= ssize(particles));
  std::vector<uint8_t> is_valid(actual_end_index, 1);
  if (!prog_ptr) {
    return is_valid;
  }
  const auto check_particle_work = [&prog_ptr, &particles, &tol, &is_valid](
                                       const int thread_num,
                                       const int64_t index) {
    unused(thread_num);
    is_valid[index] = static_cast<uint8_t>(
        CheckProgConstraints(prog_ptr, particles[index], tol));
  };

  DynamicParallelForIndexLoop(DegreeOfParallelism(parallelism.num_threads()), 0,
                              actual_end_index, check_particle_work,
                              ParallelForBackend::BEST_AVAILABLE);
  return is_valid;
}

VPolytope ParseAndCheckContainmentPoints(
    const CollisionChecker& checker,
    const CommonSampledIrisOptions& sampled_iris_options,
    const IrisParameterizationFunction& parameterization,
    const geometry::optimization::Hyperellipsoid& starting_ellipsoid,
    const double constraints_tol) {
  if (!sampled_iris_options.containment_points.has_value()) {
    return {};
  }

  VPolytope cvxh_vpoly(sampled_iris_options.containment_points.value());
  if (parameterization.get_parameterization_dimension().has_value()) {
    DRAKE_THROW_UNLESS(
        parameterization.get_parameterization_dimension().value() ==
        sampled_iris_options.containment_points->rows());
  }

  constexpr float kPointInSetTol = 1e-5;
  if (!cvxh_vpoly.PointInSet(starting_ellipsoid.center(), kPointInSetTol)) {
    throw std::runtime_error(
        "Iris precondition failure: The center of the starting ellipsoid lies "
        "outside of the convex hull of the containment points.");
  }

  cvxh_vpoly = cvxh_vpoly.GetMinimalRepresentation();

  std::vector<Eigen::VectorXd> cont_vec;
  cont_vec.reserve((sampled_iris_options.containment_points->cols()));

  for (int col = 0; col < sampled_iris_options.containment_points->cols();
       ++col) {
    Eigen::VectorXd conf = sampled_iris_options.containment_points->col(col);
    cont_vec.emplace_back(parameterization.get_parameterization_double()(conf));
    DRAKE_ASSERT(cont_vec.back().size() == checker.plant().num_positions());
  }

  std::vector<uint8_t> containment_point_col_free =
      checker.CheckConfigsCollisionFree(cont_vec,
                                        sampled_iris_options.parallelism);
  for (const auto col_free : containment_point_col_free) {
    if (!col_free) {
      throw std::runtime_error(
          "Iris precondition failure: One or more containment points are in "
          "collision!");
    }
  }
  for (int i = 0; i < sampled_iris_options.containment_points->cols(); ++i) {
    if (!CheckProgConstraints(
            sampled_iris_options.prog_with_additional_constraints,
            sampled_iris_options.containment_points->col(i), constraints_tol)) {
      throw std::runtime_error(
          "Iris precondition failure: One or more containment points violates "
          "a constraint in "
          "sampled_iris_options.prog_with_additional_constraints!");
    }
  }
  return cvxh_vpoly;
}

Eigen::VectorXd ComputeFaceTangentToDistCvxh(
    const Hyperellipsoid& E, const Eigen::Ref<const Eigen::VectorXd>& point,
    const VPolytope& cvxh_vpoly, const solvers::SolverInterface& solver) {
  Eigen::VectorXd a_face = E.A().transpose() * E.A() * (point - E.center());
  double b_face = a_face.transpose() * point;

  // Return standard iris face if either the face does not chop off any
  // containment points or collision lies inside of the convex hull of the
  // containment points.
  if (cvxh_vpoly.PointInSet(point) ||
      (a_face.transpose() * cvxh_vpoly.vertices()).maxCoeff() - b_face <= 0) {
    return a_face;
  } else {
    solvers::MathematicalProgram prog;
    const int dim = cvxh_vpoly.ambient_dimension();
    const auto x = prog.NewContinuousVariables(dim);
    cvxh_vpoly.AddPointInSetConstraints(&prog, x);
    const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(dim, dim);
    prog.AddQuadraticErrorCost(identity, point, x);
    solvers::MathematicalProgramResult result;
    solver.Solve(prog, std::nullopt, std::nullopt, &result);
    DRAKE_THROW_UNLESS(result.is_success());
    a_face = point - result.GetSolution(x);
    return a_face;
  }
}

void PopulateParticlesByUniformSampling(
    const geometry::optimization::HPolyhedron& P, int number_to_sample,
    int mixing_steps, std::vector<RandomGenerator>* generators,
    std::vector<Eigen::VectorXd>* particles) {
  DRAKE_THROW_UNLESS(number_to_sample <= ssize(*particles));
  const int num_threads = ssize(*generators);

  const auto hit_and_run_sample_work =
      [&P, &particles, &generators,
       &mixing_steps](const ThreadWorkRange& work_range) {
        const int64_t start_index = work_range.GetRangeStart();
        const int64_t end_index = work_range.GetRangeEnd();
        const int64_t thread_num = work_range.GetThreadNum();
        RandomGenerator* generator = &(generators->at(thread_num));
        (*particles)[start_index] = P.UniformSample(generator, mixing_steps);
        for (int j = start_index + 1; j < end_index; ++j) {
          (*particles)[j] =
              P.UniformSample(generator, (*particles)[j - 1], mixing_steps);
        }
      };

  StaticParallelForRangeLoop(DegreeOfParallelism(num_threads), 0,
                             number_to_sample, hit_and_run_sample_work,
                             ParallelForBackend::BEST_AVAILABLE);
}

}  // namespace internal
}  // namespace planning
}  // namespace drake
