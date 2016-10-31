#include "drake/systems/plants/rigid_body_plant/drake_visualizer.h"

namespace drake {
namespace systems {

namespace {
// Defines the index of the port that the DrakeVisualizer uses.
const int kPortIndex = 0;
}  // namespace

DrakeVisualizer::DrakeVisualizer(
    const RigidBodyTree& tree, drake::lcm::DrakeLcmInterface* lcm) :
    lcm_(lcm), load_message_(CreateLoadMessage(tree)),
    draw_message_translator_(tree) {
  set_name("drake_visualizer");
  const int vector_size =
      tree.get_num_positions() + tree.get_num_velocities();
  DeclareInputPort(kVectorValued, vector_size, kContinuousSampling);
}

const lcmt_viewer_load_robot&
DrakeVisualizer::get_load_message() const {
  return load_message_;
}

const std::vector<uint8_t>&
DrakeVisualizer::get_draw_message_bytes() const {
  return draw_message_bytes_;
}

void DrakeVisualizer::DoPublish(const Context<double>& context)
    const {
  // TODO(liang.fok): Replace the following code once System 2.0's API allows
  // systems to declare that they need a certain action to be performed at
  // simulation time t_0.
  //
  // Before any draw commands, we need to send the load_robot message.
  if (context.get_time() == 0.0) {
    PublishLoadRobot();
  }
  DRAKE_DEMAND(sent_load_robot_);

  // Obtains the input vector, which contains the generalized q,v state of the
  // RigidBodyTree.
  const BasicVector<double>* input_vector = EvalVectorInput(context,
                                                            kPortIndex);

  // Translates the input vector into an array of bytes representing an LCM
  // message.
  draw_message_translator_.Serialize(
      context.get_time(), *input_vector, &draw_message_bytes_);

  // Publishes onto the specified LCM channel.
  lcm_->Publish("DRAKE_VIEWER_DRAW", draw_message_bytes_.data(),
      draw_message_bytes_.size());
}

void DrakeVisualizer::PublishLoadRobot() const {
  const int lcm_message_length = load_message_.getEncodedSize();
  std::vector<uint8_t> lcm_message_bytes{};
  lcm_message_bytes.resize(lcm_message_length);
  load_message_.encode(lcm_message_bytes.data(), 0, lcm_message_length);

  lcm_->Publish("DRAKE_VIEWER_LOAD_ROBOT", lcm_message_bytes.data(),
      lcm_message_length);
  sent_load_robot_ = true;
}

lcmt_viewer_load_robot DrakeVisualizer::CreateLoadMessage(
    const RigidBodyTree& tree) {
  lcmt_viewer_load_robot load_message;
  load_message.num_links = tree.bodies.size();
  for (const auto& body : tree.bodies) {
    lcmt_viewer_link_data link;
    link.name = body->get_name();
    link.robot_num = body->get_model_instance_id();
    link.num_geom = body->get_visual_elements().size();
    for (const auto& visual_element : body->get_visual_elements()) {
      lcmt_viewer_geometry_data geometry_data;
      const DrakeShapes::Geometry& geometry = visual_element.getGeometry();

      // TODO(liang.fok) Do this through virtual methods without introducing any
      // LCM dependency on the Geometry classes.
      //
      // TODO(liang.fok) Add support for the DrakeShapes::MESH_POINTS type.
      switch (visual_element.getShape()) {
        case DrakeShapes::BOX: {
          geometry_data.type = geometry_data.BOX;
          geometry_data.num_float_data = 3;
          auto box = dynamic_cast<const DrakeShapes::Box&>(geometry);
          for (int i = 0; i < 3; ++i)
            geometry_data.float_data.push_back(static_cast<float>(box.size(i)));
          break;
        }
        case DrakeShapes::SPHERE: {
          geometry_data.type = geometry_data.SPHERE;
          geometry_data.num_float_data = 1;
          auto sphere = dynamic_cast<const DrakeShapes::Sphere&>(geometry);
          geometry_data.float_data.push_back(static_cast<float>(sphere.radius));
          break;
        }
        case DrakeShapes::CYLINDER: {
          geometry_data.type = geometry_data.CYLINDER;
          geometry_data.num_float_data = 2;
          auto cylinder = dynamic_cast<const DrakeShapes::Cylinder&>(geometry);
          geometry_data.float_data.push_back(static_cast<float>(
              cylinder.radius));
          geometry_data.float_data.push_back(static_cast<float>(
              cylinder.length));
          break;
        }
        case DrakeShapes::MESH: {
          geometry_data.type = geometry_data.MESH;
          geometry_data.num_float_data = 3;
          auto mesh = dynamic_cast<const DrakeShapes::Mesh&>(geometry);
          geometry_data.float_data.push_back(static_cast<float>(
              mesh.scale_[0]));
          geometry_data.float_data.push_back(static_cast<float>(
              mesh.scale_[1]));
          geometry_data.float_data.push_back(static_cast<float>(
              mesh.scale_[2]));

          if (mesh.uri_.find("package://") == 0) {
            geometry_data.string_data = mesh.uri_;
          } else {
            geometry_data.string_data = mesh.resolved_filename_;
          }

          break;
        }
        case DrakeShapes::CAPSULE: {
          geometry_data.type = geometry_data.CAPSULE;
          geometry_data.num_float_data = 2;
          auto c = dynamic_cast<const DrakeShapes::Capsule&>(geometry);
          geometry_data.float_data.push_back(static_cast<float>(c.radius));
          geometry_data.float_data.push_back(static_cast<float>(c.length));
          break;
        }
        default: {
          // Intentionally do nothing.
          break;
        }
      }

      // Saves the location and orientation of the visualization geometry in the
      // `lcmt_viewer_geometry_data` object. The location and orientation are
      // specified in the body's frame.
      Eigen::Isometry3d transform = visual_element.getLocalTransform();
      Eigen::Map<Eigen::Vector3f> position(geometry_data.position);
      position = transform.translation().cast<float>();
      Eigen::Map<Eigen::Vector4f> quaternion(geometry_data.quaternion);
      quaternion = math::rotmat2quat(transform.rotation()).cast<float>();

      Eigen::Map<Eigen::Vector4f> color(geometry_data.color);
      color = visual_element.getMaterial().template cast<float>();

      link.geom.push_back(geometry_data);
    }
    load_message.link.push_back(link);
  }

  return load_message;
}

}  // namespace systems
}  // namespace drake
