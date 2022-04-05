#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/geometry/meshcat.h"
#include "drake/multibody/meshcat/contact_visualizer_params.h"

namespace drake {
namespace multibody {
namespace meshcat {
namespace internal {

/* Like multibody::HydroelasticContactInfo, but only the visualization info.
   TODO(joemasterjohn): Add the mesh geometry and pressure values. */
struct HydroelasticContactVisualizerItem {
  std::string body_A;
  std::string body_B;
  Eigen::Vector3d centroid_W;
  Eigen::Vector3d force_C_W;
  Eigen::Vector3d moment_C_W;
};

/* HydroelasticContactVisualizer publishes hydroelastic contact results in
MeshCat. It draws double-sided arrows at the location of the contact force with
length scaled by the magnitude of the contact force.

This is unit tested via contact_visualizer_test overall; there is currently no
hydroelastic-contact-specific unit test.
*/
class HydroelasticContactVisualizer {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HydroelasticContactVisualizer)

  /* Creates an instance of HydroelasticContactVisualizer.
  Note that not all fields of `params` are relevant nor used. */
  HydroelasticContactVisualizer(std::shared_ptr<geometry::Meshcat> meshcat,
                         ContactVisualizerParams params);

  ~HydroelasticContactVisualizer();

  /* Update meshcat to show _only_ the given contacts.
  Any previously-visualized contacts will no longer be visible. */
  void Update(const std::vector<HydroelasticContactVisualizerItem>& items);

  /* Calls geometry::Meshcat::Delete(path), with the path set to params.prefix.
  Since this visualizer will only ever add geometry under this prefix, this will
  remove all geometry/transforms added by the visualizer, or by a previous
  instance of this visualizer using the same prefix. */
  void Delete();

 private:
  /* When a contact disappears, we mark it invisible rather than deleting it
  (to improve resposiveness). This struct tracks that state. */
  struct VisibilityStatus {
    /* Whether this path is currently visible in meshcat. */
    bool visible{false};
    /* Whether this contact was active as of the most recent Update(). */
    bool active{false};
  };

  /* Find an entry in path_visibility_status_, or else add one and return it.
  When an entry is added by this function, the arrow geometry is also added to
  meshcat (with visible=false) as a side-effect. */
  VisibilityStatus& FindOrAdd(const std::string& path);

  const std::shared_ptr<geometry::Meshcat> meshcat_;
  const ContactVisualizerParams params_;

  /* Map of from a contact pair's path to its status. When the map has no key
  for a given path, that indicates no geometry for that pair exists yet. */
  std::unordered_map<std::string, VisibilityStatus> path_visibility_status_;
};

}  // namespace internal
}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
