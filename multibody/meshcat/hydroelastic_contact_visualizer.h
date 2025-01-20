#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
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
  HydroelasticContactVisualizerItem(std::string body_A_, std::string body_B_,
                                    const Eigen::Vector3d& centroid_W_,
                                    const Eigen::Vector3d& force_C_W_,
                                    const Eigen::Vector3d& moment_C_W_,
                                    const Eigen::Matrix3Xd p_WV_,
                                    const Eigen::Matrix3Xi faces_,
                                    const Eigen::VectorXd pressure_)
      : body_A(std::move(body_A_)),
        body_B(std::move(body_B_)),
        centroid_W(centroid_W_),
        force_C_W(force_C_W_),
        moment_C_W(moment_C_W_),
        p_WV(std::move(p_WV_)),
        faces(std::move(faces_)),
        pressure(std::move(pressure_)) {}

  std::string body_A;
  std::string body_B;
  Eigen::Vector3d centroid_W;
  Eigen::Vector3d force_C_W;
  Eigen::Vector3d moment_C_W;
  Eigen::Matrix3Xd p_WV;
  Eigen::Matrix3Xi faces;
  Eigen::VectorXd pressure;
};

/* HydroelasticContactVisualizer publishes hydroelastic contact results for
MeshCat. It draws two single-sided arrows, one for force and one for moment, at
the centroid of the contact patch. The length of each vector is scaled by the
magnitude of the contact force/moment.

This is unit tested via contact_visualizer_test overall; there is currently no
hydroelastic-contact-specific unit test.
*/
class HydroelasticContactVisualizer {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HydroelasticContactVisualizer);

  /* Creates an instance of HydroelasticContactVisualizer.
  Note that not all fields of `params` are relevant nor used.

  @pre meshcat != nullptr
  */
  HydroelasticContactVisualizer(std::shared_ptr<geometry::Meshcat> meshcat,
                                ContactVisualizerParams params);

  ~HydroelasticContactVisualizer();

  /* Update meshcat to show _only_ the given contacts.
  Any previously-visualized contacts will no longer be visible. */
  void Update(double time,
              const std::vector<HydroelasticContactVisualizerItem>& items);

  /* Calls geometry::Meshcat::Delete(path), with the path set to params.prefix.
  Since this visualizer will only ever add geometry under this prefix, this will
  remove all geometry/transforms added by the visualizer, or by a previous
  instance of this visualizer using the same prefix. */
  void Delete();

 private:
  /* When a contact disappears, we mark it invisible rather than deleting it
  (to improve responsiveness). This struct tracks that state. */
  struct VisibilityStatus {
    /* Whether this path is currently visible in meshcat. */
    bool visible{false};
    /* Whether this contact was active as of the most recent Update(). */
    bool active{false};
  };

  /* Find an entry in path_visibility_status_, or else add one and return it.
  When an entry is added by this function, the arrow geometry is also added to
  meshcat (with visible=false and active=false) as a side-effect. */
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
