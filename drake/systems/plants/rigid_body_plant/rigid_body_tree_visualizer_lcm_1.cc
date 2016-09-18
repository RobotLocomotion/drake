// NOLINT(readability/line_length)
#include "drake/systems/plants/rigid_body_plant/rigid_body_tree_visualizer_lcm_1.h"

namespace drake {
namespace systems {

namespace {
// Defines the index of the port that the RigidBodyTreeVisualizerLcm1 uses.
const int kPortIndex = 0;
}  // namespace

RigidBodyTreeVisualizerLcm1::RigidBodyTreeVisualizerLcm1(
    const RigidBodyTree& tree, ::lcm::LCM* lcm) :
    lcm_(lcm), draw_message_translator_(tree) {
  int vector_size = tree.number_of_positions() + tree.number_of_velocities();
  DeclareInputPort(kVectorValued, vector_size, kContinuousSampling);
  InitializeLoadMessage(tree);
}

std::string RigidBodyTreeVisualizerLcm1::get_name() const {
  return "rigid_body_tree_visualizer_lcm_1";
}

const drake::lcmt_viewer_load_robot&
RigidBodyTreeVisualizerLcm1::get_load_message() const {
  return load_message_;
}

const std::vector<uint8_t>&
RigidBodyTreeVisualizerLcm1::get_draw_message_bytes() const {
  return message_bytes_;
}

void RigidBodyTreeVisualizerLcm1::DoPublish(const Context<double>& context)
    const {
  // Before any draw commands, we need to send the load_robot message.
  if (context.get_time() == 0.0) {
    PublishLoadRobot();
  }
  DRAKE_DEMAND(sent_load_robot_);

  // Obtains the input vector, which contains the generalized q,v state of the
  // RigidBodyTree.
  const VectorBase<double>* input_vector = context.get_vector_input(kPortIndex);

  // Translates the input vector into an array of bytes representing an LCM
  // message.
  std::vector<uint8_t> lcm_message_bytes;
  draw_message_translator_.TranslateVectorBaseToLcm(
      context.get_time(), *input_vector, &lcm_message_bytes);

  // Publishes onto the specified LCM channel.
  lcm_->publish("DRAKE_VIEWER_DRAW", lcm_message_bytes.data(),
      lcm_message_bytes.size());

  // Saves the message bytes into a local member variable.
  message_bytes_ = lcm_message_bytes;
}

void RigidBodyTreeVisualizerLcm1::PublishLoadRobot() const {
  lcm_->publish("DRAKE_VIEWER_LOAD_ROBOT", &load_message_);
  sent_load_robot_ = true;
}

void RigidBodyTreeVisualizerLcm1::InitializeLoadMessage(
    const RigidBodyTree& tree) {
  load_message_.num_links = tree.bodies.size();
  for (const auto& body : tree.bodies) {
    drake::lcmt_viewer_link_data link;
    link.name = body->get_name();
    link.robot_num = body->get_model_instance_id();
    link.num_geom = body->get_visual_elements().size();
    for (const auto& v : body->get_visual_elements()) {
      drake::lcmt_viewer_geometry_data gdata;
      const DrakeShapes::Geometry& geometry = v.getGeometry();
      switch (v.getShape()) {  // would prefer to do this through virtual
                               // methods, but don't want to introduce any LCM
                               // dependency on the Geometry classes
        case DrakeShapes::BOX: {
          gdata.type = gdata.BOX;
          gdata.num_float_data = 3;
          auto b = dynamic_cast<const DrakeShapes::Box&>(geometry);
          for (int i = 0; i < 3; ++i)
            gdata.float_data.push_back(static_cast<float>(b.size(i)));
          break;
        }
        case DrakeShapes::SPHERE: {
          gdata.type = gdata.SPHERE;
          gdata.num_float_data = 1;
          auto b = dynamic_cast<const DrakeShapes::Sphere&>(geometry);
          gdata.float_data.push_back(static_cast<float>(b.radius));
          break;
        }
        case DrakeShapes::CYLINDER: {
          gdata.type = gdata.CYLINDER;
          gdata.num_float_data = 2;
          auto c = dynamic_cast<const DrakeShapes::Cylinder&>(geometry);
          gdata.float_data.push_back(static_cast<float>(c.radius));
          gdata.float_data.push_back(static_cast<float>(c.length));
          break;
        }
        case DrakeShapes::MESH: {
          gdata.type = gdata.MESH;
          gdata.num_float_data = 3;
          auto m = dynamic_cast<const DrakeShapes::Mesh&>(geometry);
          gdata.float_data.push_back(static_cast<float>(m.scale_[0]));
          gdata.float_data.push_back(static_cast<float>(m.scale_[1]));
          gdata.float_data.push_back(static_cast<float>(m.scale_[2]));

          if (m.uri_.find("package://") == 0) {
            gdata.string_data = m.uri_;
          } else {
            gdata.string_data = m.resolved_filename_;
          }

          break;
        }
        case DrakeShapes::CAPSULE: {
          gdata.type = gdata.CAPSULE;
          gdata.num_float_data = 2;
          auto c = dynamic_cast<const DrakeShapes::Capsule&>(geometry);
          gdata.float_data.push_back(static_cast<float>(c.radius));
          gdata.float_data.push_back(static_cast<float>(c.length));
          break;
        }
        default: {
          // intentionally do nothing
          break;
        }
      }

      Eigen::Isometry3d T = v.getLocalTransform();
      Eigen::Map<Eigen::Vector3f> position(gdata.position);
      position = T.translation().cast<float>();
      Eigen::Map<Eigen::Vector4f> quaternion(gdata.quaternion);
      quaternion = drake::math::rotmat2quat(T.rotation()).cast<float>();

      Eigen::Map<Eigen::Vector4f> color(gdata.color);
      color = v.getMaterial().template cast<float>();

      link.geom.push_back(gdata);
    }
    load_message_.link.push_back(link);
  }
}

}  // namespace systems
}  // namespace drake
