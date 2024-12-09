#include "drake/geometry/poisson_disk.h"

#include <array>
#include <memory>
#include <utility>

#include <tph_poisson.h>

#include "drake/common/overloaded.h"
#include "drake/geometry/proximity/aabb.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"

using Eigen::Vector3d;

namespace drake {
namespace geometry {
namespace internal {
namespace {

/* Generates a set of samples within a axis-aligned bounding box (AABB)
 (including boundary) such that no two samples are closer than a user-specified
 distance.
 @pre radius > 0. */
std::vector<Vector3d> PoissonDiskSampling(double radius, const Aabb& box) {
  /* The AABB is around a shape in its geometry frame, so it's never too far
   away from the origin. Hence, we don't worry about the floating point
   precision here, even when the TPH library uses single precision by default.
  */
  Vector3<float> bounds_min = box.lower().cast<float>();
  Vector3<float> bounds_max = box.upper().cast<float>();
  tph_poisson_args args = {};
  args.radius = radius;
  args.ndims = 3;
  args.seed = 0;  // Ensure deterministic results.
  args.bounds_min = bounds_min.data();
  args.bounds_max = bounds_max.data();
  args.max_sample_attempts = 100;

  auto deleter = [](tph_poisson_sampling* s) {
    tph_poisson_destroy(s);
    delete s;
  };
  std::unique_ptr<tph_poisson_sampling, decltype(deleter)> sampling(
      new tph_poisson_sampling{}, deleter);

  /* Populate sampling with points. */
  if (const int ret = tph_poisson_create(
          &args, /* Use default allocator */ nullptr, sampling.get());
      ret != TPH_POISSON_SUCCESS) {
    throw std::runtime_error("PoissonDiskSampling: Sample creation failed!");
  }

  /* Retrieve sampling points. */
  const float* data = tph_poisson_get_samples(sampling.get());
  if (data == nullptr) {
    throw std::runtime_error("PoissonDiskSampling: Sample retrieval failed!");
  }

  const int num_points = sampling->nsamples;
  std::vector<Vector3d> results;
  for (int i = 0; i < num_points; ++i) {
    const Vector3d p{data[i * 3], data[i * 3 + 1], data[i * 3 + 2]};
    results.push_back(p);
  }
  return results;
}

/* Given a list of candidate point positions in the geometry's frame, returns a
 list of points that are inside (or on the boudary of) the given `shape`. */
std::vector<Vector3d> FilterPoints(const std::vector<Vector3d>& q_GPs,
                                   const geometry::Shape& shape) {
  std::vector<Vector3d> results;
  /* Ensures the geometries we add don't get hydroelastic representations
   because it's unnecessary and may be expensive. */
  const SceneGraphConfig config{
      .default_proximity_properties = {.compliance_type = "undefined"}};
  SceneGraph<double> scene_graph(config);
  const SourceId source_id = scene_graph.RegisterSource();
  const FrameId frame_id = scene_graph.world_frame_id();
  auto geometry_instance = std::make_unique<GeometryInstance>(
      math::RigidTransform<double>::Identity(), shape, "shape");
  geometry_instance->set_proximity_properties(ProximityProperties());
  scene_graph.RegisterGeometry(source_id, frame_id,
                               std::move(geometry_instance));
  auto context = scene_graph.CreateDefaultContext();
  const auto& query_object =
      scene_graph.get_query_output_port().Eval<QueryObject<double>>(*context);
  for (const Vector3d& q_GP : q_GPs) {
    const std::vector<SignedDistanceToPoint<double>> signed_distances =
        query_object.ComputeSignedDistanceToPoint(q_GP, 0.0 /* threshold */);
    DRAKE_DEMAND(signed_distances.size() <= 1);
    if (!signed_distances.empty()) {
      DRAKE_DEMAND(signed_distances[0].distance <= 0);
      results.push_back(q_GP);
    }
  }
  return results;
}

/* Computes a tight Aabb for the given shape.
 @throws std::exception if the derived type hasn't overloaded this
  implementation (yet). */
Aabb CalcAabb(const Shape& shape) {
  // TODO(xuchenhan-tri): Implement the rest of the shapes.
  return shape.Visit(overloaded{
      [](const Box& box) {
        return Aabb(Vector3d::Zero(), box.size() / 2.0);
      },
      [](const HalfSpace&) {
        throw std::runtime_error(
            "Computing Aabb for half space is not supported.");
        DRAKE_UNREACHABLE();
        return Aabb(Vector3d::Zero(), Vector3d::Zero());
      },
      [](const MeshcatCone&) {
        throw std::runtime_error(
            "Computing Aabb for Meshcat Cone shape is not supported.");
        DRAKE_UNREACHABLE();
        return Aabb(Vector3d::Zero(), Vector3d::Zero());
      },
      [](const auto& unsupported) {
        throw std::logic_error(
            fmt::format("Computing Aabb for {} is not supported yet.",
                        unsupported.type_name()));
        DRAKE_UNREACHABLE();
        return Aabb(Vector3d::Zero(), Vector3d::Zero());
      }});
}

}  // namespace

std::vector<Vector3d> PoissonDiskSampling(double radius, const Shape& shape) {
  DRAKE_THROW_UNLESS(radius > 0);
  const Aabb aabb = CalcAabb(shape);
  const std::vector<Vector3d> q_GPs = PoissonDiskSampling(radius, aabb);
  /* Reject points that fall outside of the shape. */
  return FilterPoints(q_GPs, shape);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
