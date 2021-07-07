#include "drake/geometry/optimization/iris.h"

#include <algorithm>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::VectorXd;

HPolyhedron Iris(const ConvexSets& obstacles, const Ref<const VectorXd>& sample,
                 const HPolyhedron& domain, const IrisOptions& options) {
  const int dim = sample.size();
  const int N = obstacles.size();
  DRAKE_DEMAND(domain.ambient_dimension() == dim);
  for (int i = 0; i < N; ++i) {
    DRAKE_DEMAND(obstacles[i]->ambient_dimension() == dim);
  }
  DRAKE_DEMAND(domain.IsBounded());
  const double kEpsilonEllipsoid = 1e-2;
  Hyperellipsoid E = Hyperellipsoid::MakeHypersphere(kEpsilonEllipsoid, sample);
  HPolyhedron P = domain;

  // On each iteration, we will build the collision-free polytope represented as
  // {x | A * x <= b}.  Here we pre-allocate matrices of the maximum size.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(
      domain.A().rows() + N, dim);
  VectorXd b(domain.A().rows() + N);
  A.topRows(domain.A().rows()) = domain.A();
  b.head(domain.A().rows()) = domain.b();
  // Use pairs {scale, index}, so that I can back out the indices after a sort.
  std::vector<std::pair<double, int>> scaling(N);
  MatrixXd closest_points(dim, N);

  double best_volume = E.Volume();
  int iteration = 0;
  MatrixXd tangent_matrix;

  while (true) {
    // Find separating hyperplanes
    for (int i = 0; i < N; ++i) {
      const auto touch = E.MinimumUniformScalingToTouch(*obstacles[i]);
      scaling[i].first = touch.first;
      scaling[i].second = i;
      closest_points.col(i) = touch.second;
    }
    std::sort(scaling.begin(), scaling.end());

    int num_constraints = domain.A().rows();
    tangent_matrix = 2.0 * E.A().transpose() * E.A();
    for (int i = 0; i < N; ++i) {
      const VectorXd point = closest_points.col(scaling[i].second);
      // Only add a constraint if this point is still within the set that has
      // been constructed so far on this iteration.
      if (((A.topRows(num_constraints) * point).array() <=
           b.head(num_constraints).array())
              .all()) {
        // Add the tangent to the (scaled) ellipsoid at this point as a
        // constraint.
        A.row(num_constraints) =
            (tangent_matrix * (point - E.center())).normalized();
        b[num_constraints] = A.row(num_constraints) * point;
        num_constraints++;
      }
    }

    if (options.require_sample_point_is_contained &&
        ((A.topRows(num_constraints) * sample).array() >=
         b.head(num_constraints).array())
            .any()) {
      break;
    }
    P = HPolyhedron(A.topRows(num_constraints), b.head(num_constraints));

    iteration++;
    if (iteration >= options.iteration_limit) {
      break;
    }

    E = P.MaximumVolumeInscribedEllipsoid();
    const double volume = E.Volume();
    if (volume - best_volume <= options.termination_threshold) {
      break;
    }
    best_volume = volume;
  }

  return P;
}

namespace {
// Constructs a ConvexSet for each supported Shape and adds it to the set.
class IrisConvexSetMaker final : public ShapeReifier {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IrisConvexSetMaker)

  IrisConvexSetMaker(const QueryObject<double>& query,
                     std::optional<FrameId> reference_frame)
      : query_{query}, reference_frame_{reference_frame} {};

  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Sphere&, void* data) {
    GeometryId& geom_id = *static_cast<GeometryId*>(data);
    sets_.emplace_back(
        std::make_unique<Hyperellipsoid>(query_, geom_id, reference_frame_));
  }

  void ImplementGeometry(const HalfSpace&, void* data) {
    GeometryId& geom_id = *static_cast<GeometryId*>(data);
    sets_.emplace_back(
        std::make_unique<HPolyhedron>(query_, geom_id, reference_frame_));
  }

  void ImplementGeometry(const Box&, void* data) {
    GeometryId& geom_id = *static_cast<GeometryId*>(data);
    // Note: We choose HPolyhedron over VPolytope here, but the IRIS paper
    // discusses a significant performance improvement using a "least-distance
    // programming" instance from CVXGEN that exploited the VPolytope
    // representation.  So we may wish to revisit this.
    sets_.emplace_back(
        std::make_unique<HPolyhedron>(query_, geom_id, reference_frame_));
  }

  void ImplementGeometry(const Ellipsoid&, void* data) {
    GeometryId& geom_id = *static_cast<GeometryId*>(data);
    sets_.emplace_back(
        std::make_unique<Hyperellipsoid>(query_, geom_id, reference_frame_));
  }

  // This must be called last; it transfers ownership.
  ConvexSets claim_sets() { return std::move(sets_); }

 private:
  const QueryObject<double>& query_{};
  std::optional<FrameId> reference_frame_{};
  ConvexSets sets_{};
};

}  // namespace

ConvexSets MakeIrisObstacles(const QueryObject<double>& query_object,
                             std::optional<FrameId> reference_frame) {
  const SceneGraphInspector<double>& inspector = query_object.inspector();
  const GeometrySet all_ids(inspector.GetAllGeometryIds());
  const std::unordered_set<GeometryId> geom_ids =
      inspector.GetGeometryIds(all_ids, Role::kProximity);

  IrisConvexSetMaker maker(query_object, reference_frame);
  for (GeometryId geom_id : geom_ids) {
    inspector.GetShape(geom_id).Reify(&maker, &geom_id);
  }
  return maker.claim_sets();
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
