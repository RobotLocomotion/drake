#include "drake/automotive/box_car_vis.h"

#include "drake/multibody/shapes/visual_element.h"
#include "drake/systems/rendering/drake_visualizer_client.h"

using std::vector;

namespace drake {

using systems::rendering::MakeGeometryData;
using systems::rendering::PoseBundle;

namespace automotive {

template <typename T>
BoxCarVis<T>::BoxCarVis(int model_instance_id, const std::string& name)
    : CarVis<T>(model_instance_id, name) {
  DrakeShapes::VisualElement box_vis(
      DrakeShapes::Box(Eigen::Vector3d(1.0, 1.0, 1.0)),
      Eigen::Isometry3d::Identity(),
      Eigen::Vector4d(0.7, 0.7, 0.7, 1));

  lcmt_viewer_link_data link_data;
  link_data.name = CarVis<T>::name();
  link_data.robot_num = CarVis<T>::id();
  link_data.num_geom = 1;
  link_data.geom.resize(1);
  link_data.geom[0] = MakeGeometryData(box_vis);

  vis_elements_.push_back(link_data);
}

template <typename T>
const vector<lcmt_viewer_link_data>& BoxCarVis<T>::GetVisElements() const {
  return vis_elements_;
}

template <typename T>
PoseBundle<T> BoxCarVis<T>::CalcPoses(const Isometry3<T>& X_WM) const {
  PoseBundle<T> result(1);
  result.set_pose(0, X_WM);
  result.set_name(0, this->name());
  result.set_model_instance_id(0, this->id());
  return result;
}

// These instantiations must match the API documentation in
// box_car_vis.h.
template class BoxCarVis<double>;

}  // namespace automotive
}  // namespace drake
