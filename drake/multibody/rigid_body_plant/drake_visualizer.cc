#include "drake/multibody/rigid_body_plant/drake_visualizer.h"

#include <chrono>
#include <thread>


namespace drake {
namespace systems {

namespace {
// Defines the index of the port that the DrakeVisualizer uses.
const int kPortIndex = 0;
}  // namespace

DrakeVisualizer::DrakeVisualizer(const RigidBodyTree<double>& tree,
                                 drake::lcm::DrakeLcmInterface* lcm,
                                 bool record_for_playback)
    : lcm_(lcm),
      load_message_(CreateLoadMessage(tree)),
      draw_message_translator_(tree),
      has_playback_(record_for_playback) {
  set_name("drake_visualizer");
  const int vector_size =
      tree.get_num_positions() + tree.get_num_velocities();
  DeclareInputPort(kVectorValued, vector_size);
  if (has_playback_) log_.reset(new SignalLog<double>(vector_size));
}

void DrakeVisualizer::set_publish_period(double period) {
  LeafSystem<double>::DeclarePublishPeriodSec(period);
}

void DrakeVisualizer::ReplayCachedSimulation() {
  if (has_playback_) {
    // Build piecewise polynomial
    auto times = log_->sample_times();
    // NOTE: The SignalLog can record signal for multiple identical time stamps.
    //  This culls the duplicates as required by the PiecewisePolynomial.
    std::vector<int> included_times;
    included_times.reserve(times.rows());
    std::vector<double> breaks;
    included_times.push_back(0);
    breaks.push_back(times(0));
    int last = 0;
    for (int i = 1; i < times.rows(); ++i) {
      double val = times(i);
      if (val != breaks[last]) {
        breaks.push_back(val);
        included_times.push_back(i);
        ++last;
      }
    }

    auto sample_data = log_->data();
    std::vector<MatrixX<double>> knots;
    knots.reserve(sample_data.cols());
    for (int c : included_times) {
      knots.push_back(sample_data.col(c));
    }
    auto func = PiecewisePolynomial<double>::ZeroOrderHold(breaks, knots);

    RunPlaybackLoop(func, breaks[breaks.size() - 1]);
  }
}

void DrakeVisualizer::RunPlaybackLoop(
    const PiecewisePolynomial<double>& trajectory,
    const double kMaxTime) const {
  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::duration<double>;
  using TimePoint = std::chrono::time_point<Clock, Duration>;

  // Target frame length at 60 Hz playback rate.
  const double kFrameLength = 1 / 60.0;
  double sim_time = 0;
  TimePoint prev_time = Clock::now();
  BasicVector<double> data(log_->get_input_size());
  while (sim_time <= kMaxTime) {
    data.set_value(trajectory.value(sim_time));

    // Evaluate at time
    // Translates the input vector into an array of bytes representing an LCM
    // message.
    std::vector<uint8_t> message_bytes;
    draw_message_translator_.Serialize(sim_time, data,
                                       &message_bytes);

    // Publishes onto the specified LCM channel.
    lcm_->Publish("DRAKE_VIEWER_DRAW", message_bytes.data(),
                  message_bytes.size());

    const TimePoint earliest_next_frame = prev_time + Duration(kFrameLength);
    std::this_thread::sleep_until(earliest_next_frame);
    TimePoint curr_time = Clock::now();
    sim_time += (curr_time - prev_time).count();
    prev_time = curr_time;
  }
}

void DrakeVisualizer::DoPublish(const Context<double>& context) const {
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
  if (has_playback_) {
    log_->AddData(context.get_time(), input_vector->get_value());
  }

  // Translates the input vector into an array of bytes representing an LCM
  // message.
  std::vector<uint8_t> message_bytes;
  draw_message_translator_.Serialize(context.get_time(), *input_vector,
                                     &message_bytes);

  // Publishes onto the specified LCM channel.
  lcm_->Publish("DRAKE_VIEWER_DRAW", message_bytes.data(),
                message_bytes.size());
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
    const RigidBodyTree<double>& tree) {
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
