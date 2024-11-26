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
std::vector<Vector3<double>> PoissonDiskSampling(double radius,
                                                 const Aabb& box) {
  Vector3<float> bounds_min = box.lower().cast<float>();
  Vector3<float> bounds_max = box.upper().cast<float>();
  tph_poisson_args args = {};
  args.radius = radius;
  args.ndims = 3;
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
  std::vector<Vector3<double>> results;
  for (int i = 0; i < num_points; ++i) {
    Vector3<double> p;
    for (int d = 0; d < 3; ++d) {
      p[d] = data[i * 3 + d];
    }
    results.push_back(p);
  }
  return results;
}

/* Given a list of candidate point positions in the geometry's frame, returns a
 list of points that are inside (or on the boudary of) the given `shape`. */
std::vector<Vector3<double>> FilterPoints(
    const std::vector<Vector3<double>>& q_GPs, const geometry::Shape& shape) {
  std::vector<Vector3<double>> results;
  SceneGraph<double> scene_graph;
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
  for (const Vector3<double>& q_GP : q_GPs) {
    DRAKE_DEMAND(query_object.ComputeSignedDistanceToPoint(q_GP).size() == 1);
    if (query_object.ComputeSignedDistanceToPoint(q_GP)[0].distance <= 0) {
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
      [](const Capsule&) {
        throw std::runtime_error(
            "Computing Aabb for capsule is not supported yet.");
        DRAKE_UNREACHABLE();
        return Aabb(Vector3d::Zero(), Vector3d::Zero());
      },
      [](const Convex&) {
        throw std::runtime_error(
            "Computing Aabb for convex shape is not supported yet.");
        DRAKE_UNREACHABLE();
        return Aabb(Vector3d::Zero(), Vector3d::Zero());
      },
      [](const Cylinder&) {
        throw std::runtime_error(
            "Computing Aabb for cylinder is not supported yet.");
        DRAKE_UNREACHABLE();
        return Aabb(Vector3d::Zero(), Vector3d::Zero());
      },
      [](const Ellipsoid&) {
        throw std::runtime_error(
            "Computing Aabb for ellipsoid is not supported yet.");
        DRAKE_UNREACHABLE();
        return Aabb(Vector3d::Zero(), Vector3d::Zero());
      },
      [](const HalfSpace&) {
        throw std::runtime_error(
            "Computing Aabb for half space is not supported.");
        DRAKE_UNREACHABLE();
        return Aabb(Vector3d::Zero(), Vector3d::Zero());
      },
      [](const Mesh&) {
        throw std::runtime_error(
            "Computing Aabb for Mesh shape is not supported yet.");
        DRAKE_UNREACHABLE();
        return Aabb(Vector3d::Zero(), Vector3d::Zero());
      },
      [](const MeshcatCone&) {
        throw std::runtime_error(
            "Computing Aabb for Meshcat Cone shape is not supported.");
        DRAKE_UNREACHABLE();
        return Aabb(Vector3d::Zero(), Vector3d::Zero());
      },
      [](const Sphere&) {
        throw std::runtime_error(
            "Computing Aabb for sphere is not supported yet.");
        DRAKE_UNREACHABLE();
        return Aabb(Vector3d::Zero(), Vector3d::Zero());
      }});
}

}  // namespace

std::vector<Vector3<double>> PoissonDiskSampling(double radius,
                                                 const Shape& shape) {
  const Aabb aabb = CalcAabb(shape);
  const std::vector<Vector3<double>> q_GPs = PoissonDiskSampling(radius, aabb);
  /* Reject points that fall outside of the shape. */
  return FilterPoints(q_GPs, shape);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
