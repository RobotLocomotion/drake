#include "drake/systems/bot_visualizer_system.h"

#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

namespace {
const int kNumInputPorts = 1;
const int kPortIndex = 0;
}  // namespace

BotVisualizerSystem::BotVisualizerSystem(
    const RigidBodyTree& tree,
    ::lcm::LCM* lcm)
    : tree_(tree), lcm_(lcm) {
  initialize_drake_visualizer();
  initialize_draw_message();
}

BotVisualizerSystem::~BotVisualizerSystem() {}

std::string BotVisualizerSystem::get_name() const {
  return "BotVisualizerSystem";
}

std::unique_ptr<ContextBase<double>> BotVisualizerSystem::CreateDefaultContext()
    const {
  std::unique_ptr<Context<double>> context(new Context<double>());
  context->SetNumInputPorts(kNumInputPorts);
  return std::unique_ptr<ContextBase<double>>(context.release());
}

std::unique_ptr<SystemOutput<double>> BotVisualizerSystem::AllocateOutput(
    const ContextBase<double>& context) const {
  std::unique_ptr<SystemOutput<double>> output(new LeafSystemOutput<double>);
  return output;
}

// TODO(liang.fok) Move the contents of this method into another method that's
// dedicated to publishing middleware messages once it is defined by System. For
// more information, see:
//     https://github.com/RobotLocomotion/drake/issues/2836.
void BotVisualizerSystem::EvalOutput(const ContextBase<double>& context,
                                    SystemOutput<double>* output) const {
  // Obtains the input vector.
  const VectorBase<double>* input_vector = context.get_vector_input(kPortIndex);

  draw_msg_.timestamp = static_cast<int64_t>(context.get_time() * 1000.0);

  const Eigen::VectorXd q = input_vector->get_value().head(
      tree_.number_of_positions());
  KinematicsCache<double> cache = tree_.doKinematics(q);

  for (size_t i = 0; i < tree_.bodies.size(); ++i) {
    auto transform = tree_.relativeTransform(cache, 0, i);
    auto quat = drake::math::rotmat2quat(transform.linear());
    std::vector<float>& position = draw_msg_.position[i];
    auto translation = transform.translation();
    for (int j = 0; j < 3; ++j) {
      position[j] = static_cast<float>(translation(j));
    }
    std::vector<float>& quaternion = draw_msg_.quaternion[i];
    for (int j = 0; j < 4; ++j) {
      quaternion[j] = static_cast<float>(quat(j));
    }
  }

  lcm_->publish("DRAKE_VIEWER_DRAW", &draw_msg_);
}

void BotVisualizerSystem::initialize_drake_visualizer() {
  drake::lcmt_viewer_load_robot message;
  message.num_links = tree_.bodies.size();
  for (const auto& body : tree_.bodies) {
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
    message.link.push_back(link);
  }

  lcm_->publish("DRAKE_VIEWER_LOAD_ROBOT", &message);
}

void BotVisualizerSystem::initialize_draw_message() {
  draw_msg_.num_links = tree_.bodies.size();
  std::vector<float> position = {0, 0, 0};
  std::vector<float> quaternion = {0, 0, 0, 1};
  for (const auto& body : tree_.bodies) {
    draw_msg_.link_name.push_back(body->get_name());
    draw_msg_.robot_num.push_back(body->get_model_instance_id());
    draw_msg_.position.push_back(position);
    draw_msg_.quaternion.push_back(quaternion);
  }
}

}  // namespace systems
}  // namespace drake
