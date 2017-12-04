#include "drake/multibody/rigid_body_plant/drake_visualizer.h"

#include <chrono>
#include <thread>
#include <utility>

#include "drake/common/text_logging.h"
#include "drake/multibody/rigid_body_plant/create_load_robot_message.h"
#include "drake/systems/rendering/drake_visualizer_client.h"

namespace drake {
namespace systems {

namespace {
// Defines the index of the port that the DrakeVisualizer uses.
const int kPortIndex = 0;
}  // namespace

DrakeVisualizer::DrakeVisualizer(const RigidBodyTree<double>& tree,
                                 drake::lcm::DrakeLcmInterface* lcm,
                                 bool enable_playback)
    : lcm_(lcm),
      load_message_(multibody::CreateLoadRobotMessage<double>(tree)),
      draw_message_translator_(tree) {
  set_name("drake_visualizer");
  const int vector_size = tree.get_num_positions() + tree.get_num_velocities();
  DeclareInputPort(kVectorValued, vector_size);
  if (enable_playback) log_.reset(new SignalLog<double>(vector_size));

  PublishEvent<double> init_event(
      Event<double>::TriggerType::kInitialization);
  DeclareInitializationEvent(init_event);
}

void DrakeVisualizer::set_publish_period(double period) {
  LeafSystem<double>::DeclarePeriodicPublish(period);
}

void DrakeVisualizer::ReplayCachedSimulation() const {
  if (log_ != nullptr) {
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

    PlaybackTrajectory(func);
  } else {
    drake::log()->warn(
        "DrakeVisualizer::ReplayCachedSimulation() called on instance that "
        "wasn't initialized to record. Next time, please construct "
        "DrakeVisualizer with recording enabled.");
  }
}

void DrakeVisualizer::PlaybackTrajectory(
    const PiecewisePolynomial<double>& input_trajectory) const {
  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::duration<double>;
  using TimePoint = std::chrono::time_point<Clock, Duration>;

  // Target frame length at 60 Hz playback rate.
  const double kFrameLength = 1 / 60.0;
  double sim_time = input_trajectory.getStartTime();
  TimePoint prev_time = Clock::now();
  BasicVector<double> data(log_->get_input_size());
  while (sim_time < input_trajectory.getEndTime()) {
    data.set_value(input_trajectory.value(sim_time));

    // Translates the input vector into an array of bytes representing an LCM
    // message.
    std::vector<uint8_t> message_bytes;
    draw_message_translator_.Serialize(sim_time, data, &message_bytes);

    // Publishes onto the specified LCM channel.
    lcm_->Publish("DRAKE_VIEWER_DRAW", message_bytes.data(),
                  message_bytes.size());

    const TimePoint earliest_next_frame = prev_time + Duration(kFrameLength);
    std::this_thread::sleep_until(earliest_next_frame);
    TimePoint curr_time = Clock::now();
    sim_time += (curr_time - prev_time).count();
    prev_time = curr_time;
  }

  // Final evaluation is at the final time stamp, guaranteeing the final state
  // is visualized.
  data.set_value(input_trajectory.value(input_trajectory.getEndTime()));
  std::vector<uint8_t> message_bytes;
  draw_message_translator_.Serialize(sim_time, data, &message_bytes);
  lcm_->Publish("DRAKE_VIEWER_DRAW", message_bytes.data(),
                message_bytes.size());
}

void DrakeVisualizer::DoPublish(
    const Context<double>& context,
    const std::vector<const PublishEvent<double>*>& event) const {
  // Initialization should only happen as a singleton event.
  if (event.size() == 1 && event.front()->get_trigger_type() ==
      Event<double>::TriggerType::kInitialization) {
    PublishLoadRobot();
    return;
  }

  // Obtains the input vector, which contains the generalized q,v state of the
  // RigidBodyTree.
  const BasicVector<double>* input_vector =
      EvalVectorInput(context, kPortIndex);
  if (log_ != nullptr) {
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
}

}  // namespace systems
}  // namespace drake
