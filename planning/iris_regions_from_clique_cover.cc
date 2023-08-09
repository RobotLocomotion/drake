#include "drake/planning/iris_regions_from_clique_cover.h"

#include <forward_list>
#include <memory>

#include "drake/common/ssize.h"
#include "drake/planning/visibility_graph.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {

using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::Triplet;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperellipsoid;
using geometry::optimization::IrisInConfigurationSpace;
using geometry::optimization::IrisOptions;
using solvers::Binding;
using solvers::LinearConstraint;
using solvers::MathematicalProgram;
using solvers::Solve;
using symbolic::Variable;

namespace {

VectorX<bool> MaximumClique(
    SparseMatrix<bool> adjacency_matrix,
    std::vector<int> indices_that_must_be_contained = {}) {
  const auto n = adjacency_matrix.rows();
  DRAKE_DEMAND(adjacency_matrix.cols() == n);
  MathematicalProgram prog;
  auto v = prog.NewBinaryVariables(n, "v");
  prog.AddLinearCost(Eigen::VectorXd::Constant(n, -1), v);
  // Make the constraint once, and use it many times.
  auto constraint = std::make_shared<LinearConstraint>(
      Eigen::RowVector2d{1, 1}, Vector1d{0}, Vector1d{1});  // [1, 1] x <= 1.
  // Note: even though we only need the lower (or upper) triangular component of
  // the adjacency matrix to define the cliques, adding the redundant
  // constraints significantly improves the performance of branch and bound.
  for (int j = 0; j < n; ++j) {
    // If adjacency_matrix[i, j] == 0, then v[i] + v[j] <= 1. We accomplish this
    // by looping over the non-zero elements, but adding constraints for the
    // (missing) zero elements.
    int zero_index = 0;
    for (SparseMatrix<bool>::InnerIterator it(adjacency_matrix, j); it; ++it) {
      while (zero_index < it.row()) {
        prog.AddConstraint(Binding<LinearConstraint>(
            constraint, Vector2<Variable>{v[zero_index++], v[j]}));
      }
      zero_index = it.row() + 1;
    }
    while (zero_index < n) {
      prog.AddConstraint(Binding<LinearConstraint>(
          constraint, Vector2<Variable>{v[zero_index++], v[j]}));
    }
  }
  for (int i : indices_that_must_be_contained) {
    prog.AddConstraint(v[i] == 1);
  }

  auto result = Solve(prog);
  return result.GetSolution(v).cast<bool>();
}

SparseMatrix<bool> RemoveRowsAndColumns(const SparseMatrix<bool>& mat,
                                        const VectorX<bool>& mask) {
  const int mat_size = mat.rows();
  DRAKE_DEMAND(mat.rows() == mat.cols());
  DRAKE_DEMAND(mask.rows() == mat.rows());
  VectorX<int> old_to_new = VectorX<int>::Constant(mat_size, -1);
  int n = 0;
  for (int i = 0; i < mat_size; ++i) {
    if (!mask[i]) {
      old_to_new[i] = n++;
    }
  }
  const int new_size = n;
  std::vector<Triplet<bool>> tripletList;
  tripletList.reserve(mat.nonZeros());  // This is an upper-bound.
  for (int j = 0; j < mat.outerSize(); ++j) {
    if (mask[j]) {
      continue;
    }
    for (SparseMatrix<bool>::InnerIterator it(mat, j); it; ++it) {
      if (mask[it.index()]) {
        continue;
      }
      tripletList.push_back(Triplet<bool>(old_to_new[it.row()],
                                          old_to_new[it.col()], it.value()));
    }
  }
  SparseMatrix<bool> new_mat(new_size, new_size);
  new_mat.setFromTriplets(tripletList.begin(), tripletList.end());
  return new_mat;
}

}  // namespace

std::vector<HPolyhedron> IrisRegionsFromCliqueCover(
    const CollisionChecker& checker,
    const Eigen::Ref<const Eigen::MatrixXd>& original_points,
    const IrisOptions& options, int minimum_clique_size, bool parallelize) {
  const int num_positions = original_points.rows();

  // Remove points in collision.
  int num_points = 0;
  std::vector<uint8_t> points_free(original_points.cols(), 0x00);
#if defined(_OPENMP)
  const int num_threads_to_use = checker.GetNumberOfThreads(parallelize);
#pragma omp parallel for num_threads(num_threads_to_use)
#endif
  for (int i = 0; i < original_points.cols(); ++i) {
    points_free[i] = static_cast<uint8_t>(
        checker.CheckConfigCollisionFree(original_points.col(i)));
    if (points_free[i]) {
      ++num_points;
    }
  }
  Eigen::MatrixXd points(num_positions, num_points);
  for (int i = 0, j = 0; i < original_points.cols(); ++i) {
    if (points_free[i]) {
      points.col(j++) = original_points.col(i);
    }
  }

  // Compute visibility graph.
  SparseMatrix<bool> visibility_graph =
      planning::VisibilityGraph(checker, points, parallelize);
  SparseMatrix<bool> remaining_visibility_graph = visibility_graph;

  std::vector<Hyperellipsoid> starting_ellipse;

  // point_indices = [0, ... , num_points-1].
  std::forward_list<int> point_indices;
  auto it = point_indices.before_begin();
  for (int i = 0; i < num_points; ++i) {
    it = point_indices.insert_after(it, i);
  }

  while (visibility_graph.rows() > 0) {
    VectorX<bool> clique = MaximumClique(remaining_visibility_graph);
    int clique_size = clique.cast<int>().sum();
    if (clique_size < minimum_clique_size) {
      break;
    }

    // Extract (and remove) the clique_indices from point_indices.
    std::vector<int> clique_indices(clique_size);
    auto prev = point_indices.before_begin();
    auto curr = point_indices.begin();
    int index = 0;
    int count = 0;
    while (curr != point_indices.end()) {
      if (clique[index++]) {
        clique_indices[count++] = *curr;
        curr = point_indices.erase_after(prev);
      } else {
        prev = curr;
        ++curr;
      }
    }

    // Expand the clique using the original visibility graph.
    clique = MaximumClique(visibility_graph, clique_indices);
    clique_size = clique.cast<int>().sum();

    // Collect the points in the clique.
    Eigen::MatrixXd clique_points(num_positions, clique_size);
    index = 0;
    for (int i : clique_indices) {
      clique_points.col(index++) = points.col(i);
    }
    log()->debug("found clique:\n {}", fmt_eigen(clique_points));

    if (clique_size == 1) {
      // If the clique is a single point, then we can just use a ball.
      starting_ellipse.push_back(
          Hyperellipsoid::MakeHypersphere(0.001, clique_points.col(0)));
    } else {
      // Compute the Loewner-John ellipsoid.
      Hyperellipsoid E =
          Hyperellipsoid::MinimumVolumeCircumscribedEllipsoid(clique_points);
      if (!checker.CheckConfigCollisionFree(E.center())) {
        // TODO(russt): Find a nice solution here.  Perhaps even add the center
        // to points and call the method again (to restart from the top)?
        throw std::runtime_error(
            "The center of the circumscribed ellipsoid is in collision. This "
            "is technically possible (but should be rare); we still need to "
            "handle it better. As a workaround, providing a more dense set of "
            "points should help.");
      }
      // Add a small term to the diagonal to ensure that the ellipsoid has
      // volume.
      starting_ellipse.push_back(Hyperellipsoid(
          E.A() + 0.001 * MatrixXd::Identity(num_positions, num_positions),
          E.center()));
    }
    // Remove the clique from visibility_graph.
    visibility_graph = RemoveRowsAndColumns(visibility_graph, clique);
  }

  // Compute the IRIS regions.
  std::vector<HPolyhedron> regions(starting_ellipse.size());
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads_to_use)
#endif
  for (int i = 0; i < ssize(starting_ellipse); ++i) {
    IrisOptions o = options;
    o.starting_ellipse = starting_ellipse[i];
    o.iteration_limit = 1;
    regions[i] = IrisInConfigurationSpace(
        checker.plant(), checker.UpdatePositions(starting_ellipse[i].center()),
        o);
  }
  return regions;
}

}  // namespace planning
}  // namespace drake
