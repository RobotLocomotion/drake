#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "drake/geometry/meshcat.h"
#include "drake/geometry/rgba.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace geometry {

/** MeshcatPointCloudVisualizer is a systems::LeafSystem that publishes a
perception::PointCloud from its input port to Meshcat.

@system
name: MeshcatPointCloudVisualizer
input_ports:
- cloud
- X_ParentCloud (optional)
@endsystem

The PointCloud on the `cloud` input port must have XYZ values.  RGB values are
optional.  The optional input port `X_ParentCloud` sets the MeshcatTransform at
the path representing the `cloud`.  If it is not connected, then we set
`X_ParentCloud` to the identity transform.

@tparam_nonsymbolic_scalar
*/
template <typename T>
class MeshcatPointCloudVisualizer final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatPointCloudVisualizer)

  /** Creates an instance of %MeshcatPointCloudVisualizer

  @param meshcat is a shared Meshcat instance.
  @param path is the Meshcat path (see @ref meshcat_path)
  @param publish_period is the duration (in simulation seconds) between updates
  sent to the visualizer.  It must be non-negative.
  */
  MeshcatPointCloudVisualizer(std::shared_ptr<Meshcat> meshcat,
                              std::string path,
                              double publish_period = 1 / 32.0);

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion.
   It should only be used to convert _from_ double _to_ other scalar types.
   */
  template <typename U>
  explicit MeshcatPointCloudVisualizer(
      const MeshcatPointCloudVisualizer<U>& other);

  /** Sets the size of each point in the cloud.  The default is 0.001. The
   * units are undocumented in threejs
   * (https://threejs.org/docs/index.html?q=PointsMaterial#api/en/materials/PointsMaterial.size),
   * but we believe they are in meters. */
  void set_point_size(double point_size) { point_size_ = point_size; }

  /** Sets the default color, which is applied to all points only if
  `has_rgbs() == false` for the cloud on the input port. */
  void set_default_rgba(const Rgba& rgba) { default_rgba_ = rgba; }

  /** Calls Meschat::Delete(path), where `path` is the value passed in the
   constructor. */
  void Delete() const;

  /** Returns the RigidTransform-valued input port. */
  const systems::InputPort<T>& cloud_input_port() const {
    return this->get_input_port(cloud_input_port_);
  }

  /** Returns the PointCloud-valued input port. */
  const systems::InputPort<T>& pose_input_port() const {
    return this->get_input_port(pose_input_port_);
  }

 private:
  /* MeshcatPointCloudVisualizer of different scalar types can all access each
   other's data. */
  template <typename>
  friend class MeshcatPointCloudVisualizer;

  /* The periodic event handler which publishes the cloud to Meshcat.  */
  systems::EventStatus UpdateMeshcat(const systems::Context<T>& context) const;

  /* Input ports. */
  int cloud_input_port_{};
  int pose_input_port_{};

  /* Meshcat is mutable because we must send messages (a non-const operation)
   from a const System (e.g. during simulation).  We use shared_ptr instead of
   unique_ptr to facilitate sharing ownership through scalar conversion;
   creating a new Meshcat object during the conversion is not a viable option.
   */
  mutable std::shared_ptr<Meshcat> meshcat_{};

  /* The Meshcat path where the cloud is set. */
  std::string path_;

  /* Visualization parameters. */
  double point_size_{0.001};
  Rgba default_rgba_{.9, .9, .9, 1.0};

  /* We store the arguments passed in the constructor to support scalar
  conversion. */
  double publish_period_;
};

/** A convenient alias for the MeshcatPointCloudVisualizer class when using the
`double` scalar type. */
using MeshcatPointCloudVisualizerd = MeshcatPointCloudVisualizer<double>;

}  // namespace geometry

// Define the conversion trait to *only* allow double -> AutoDiffXd conversion.
// Symbolic is not supported yet, and AutoDiff -> double doesn't "make sense".
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<geometry::MeshcatPointCloudVisualizer>
    : public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems

}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::MeshcatPointCloudVisualizer)
