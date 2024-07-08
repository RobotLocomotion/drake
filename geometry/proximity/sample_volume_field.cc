#include "drake/geometry/proximity/sample_volume_field.h"

#include <limits>

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using math::RigidTransformd;

namespace {
class BvhVisitor {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BvhVisitor);

  static bool SampleAtPoint(
      const Vector3d& p_WQ,
      const VolumeMeshFieldLinear<double, double>& field_W,
      const Bvh<Obb, VolumeMesh<double>>::NodeType& node_W, double* sample) {
    BvhVisitor visitor{p_WQ, field_W};
    const bool is_inside = visitor.Visit(node_W);
    if (is_inside) *sample = visitor.sample_;
    return is_inside;
  }

 private:
  BvhVisitor(const Vector3d& p_WQ,
             const VolumeMeshFieldLinear<double, double>& field_W)
      : p_WQ_{p_WQ}, field_W_{field_W}, mesh_W_{field_W.mesh()} {}

  // Returns `true` iff point p_WQ is inside node_W.
  // Moreover, it evaluates the volume field at p_WQ.
  bool Visit(const Bvh<Obb, VolumeMesh<double>>::NodeType& node_W) {
    if (node_W.is_leaf()) {
      for (int i = 0; i < node_W.num_element_indices(); ++i) {
        const int e = node_W.element_index(i);
        const Vector4<double> b = mesh_W_.CalcBarycentric(p_WQ_, e);
        const bool p_is_inside_e = b.minCoeff() >= 0;
        if (p_is_inside_e) {
          sample_ = field_W_.Evaluate(e, b);
          return true;
        }
      }
      return false;
    }

    // If inside this OBB recursively search both children. Otherwise report
    // that point is outside.

    const Vector3d box_half_width_B = node_W.bv().half_width();
    const RigidTransformd& X_WB = node_W.bv().pose();
    const Vector3d p_BQ = X_WB.inverse() * p_WQ_;

    auto is_inside_box = [&h = box_half_width_B](const Vector3d& p) {
      for (int i = 0; i < 3; ++i) {
        if (std::abs(p(i)) > h(i)) {
          return false;
        }
      }
      return true;
    };

    if (is_inside_box(p_BQ)) {
      // We couldn't prune the subtree, recursively search both children.
      if (this->Visit(node_W.left())) return true;
      if (this->Visit(node_W.right())) return true;
    }
    return false;
  }

 private:
  const Vector3<double>& p_WQ_;
  const VolumeMeshFieldLinear<double, double>& field_W_;
  const VolumeMesh<double>& mesh_W_;
  double sample_{std::numeric_limits<double>::quiet_NaN()};
};
}  // namespace

double SampleVolumeFieldOrThrow(const VolumeMeshFieldLinear<double, double>& f,
                                const Bvh<Obb, VolumeMesh<double>> bvh,
                                const Vector3<double>& p) {
  double sample;
  const bool is_inside =
      BvhVisitor::SampleAtPoint(p, f, bvh.root_node(), &sample);
  if (!is_inside) throw std::logic_error("Point is outside the volume.");
  return sample;
}

double SampleVolumeFieldOrThrow(const VolumeMeshFieldLinear<double, double>& f,
                                const Vector3<double>& p) {
  // Barycentric coordinates will be positive when the point p is inside a
  // tetrahedron. When p is right on the boundary of a tetrahedron, some of the
  // barycentric coordinates will become zero. In practice however, round-off
  // errors can lead to small (order machine epsilon) negative values. To handle
  // this case robustly we use a slop.
  const double kSlop = 8 * std::numeric_limits<double>::epsilon();
  const VolumeMesh<double>& mesh = f.mesh();
  for (int e = 0; e < mesh.num_elements(); ++e) {
    const Vector4<double> b = mesh.CalcBarycentric(p, e);
    const bool p_is_inside_e = b.minCoeff() >= -kSlop;
    if (p_is_inside_e) {
      return f.Evaluate(e, b);
    }
  }
  throw std::logic_error("Point is outside the volume.");
  DRAKE_UNREACHABLE();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
