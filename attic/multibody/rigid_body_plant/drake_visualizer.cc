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
      draw_message_translator_(tree),
      tree_(tree) {
  set_name("drake_visualizer");
  const int input_size = tree.get_num_positions() + tree.get_num_velocities();
  DeclareInputPort(kVectorValued, input_size);
  if (enable_playback)
    log_.reset(new SignalLog<double>(tree.get_num_positions()));

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
    auto func =
        trajectories::PiecewisePolynomial<double>::ZeroOrderHold(breaks, knots);

    PlaybackTrajectory(func);
  } else {
    drake::log()->warn(
        "DrakeVisualizer::ReplayCachedSimulation() called on instance that "
        "wasn't initialized to record. Next time, please construct "
        "DrakeVisualizer with recording enabled.");
  }
}

void DrakeVisualizer::PlaybackTrajectory(
    const trajectories::PiecewisePolynomial<double>& input_trajectory) const {
  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::duration<double>;
  using TimePoint = std::chrono::time_point<Clock, Duration>;

  // Target frame length at 60 Hz playback rate.
  const double kFrameLength = 1 / 60.0;
  double sim_time = input_trajectory.start_time();
  TimePoint prev_time = Clock::now();
  const int num_positions = tree_.get_num_positions();
  BasicVector<double> data(num_positions);
  while (sim_time < input_trajectory.end_time()) {
    data.set_value(input_trajectory.value(sim_time).col(0).head(num_positions));

    // Translates the input vector into an array of bytes representing an LCM
    // message.
    std::vector<uint8_t> message_bytes;
    draw_message_translator_.Serialize(sim_time, data, &message_bytes);

    // Publishes onto the specified LCM channel.
    lcm_->Publish("DRAKE_VIEWER_DRAW", message_bytes.data(),
                  message_bytes.size(), sim_time);

    const TimePoint earliest_next_frame = prev_time + Duration(kFrameLength);
    std::this_thread::sleep_until(earliest_next_frame);
    TimePoint curr_time = Clock::now();
    sim_time += (curr_time - prev_time).count();
    prev_time = curr_time;
  }

  // Final evaluation is at the final time stamp, guaranteeing the final state
  // is visualized.
  data.set_value(input_trajectory.value(input_trajectory.end_time())
                     .col(0)
                     .head(num_positions));
  std::vector<uint8_t> message_bytes;
  draw_message_translator_.Serialize(sim_time, data, &message_bytes);
  lcm_->Publish("DRAKE_VIEWER_DRAW", message_bytes.data(),
                message_bytes.size(), sim_time);
}

void DrakeVisualizer::DoPublish(
    const Context<double>& context,
    const std::vector<const PublishEvent<double>*>& events) const {
  // This event handler is called to handle initialization, periodic, and
  // forced events (at least). The clean way to handle this variety of event
  // types would be to establish a handler for each type, rather than the pseudo
  // switch statement that is being done. But since this code lives in the
  // attic, we're going with the simplest (ugly) strategy that works.

  // Handle any initialization events first.
  int num_initialization_events = 0;
  for (int i = 0; i < static_cast<int>(events.size()); ++i) {
    if (events[i]->get_trigger_type() ==
        Event<double>::TriggerType::kInitialization) {
      // We expect no more than one initialization event.
      DRAKE_DEMAND(++num_initialization_events == 1);
      PublishLoadRobot();
    }
  }

  // If events are all initialization events, return now.
  if (static_cast<int>(events.size()) == num_initialization_events)
    return;

  // There is at least one non-initialization event (it's conceivable
  // multiple periodic events triggered at the same time), so we do the
  // non-initial behavior.

  // Obtains the input vector, which contains the generalized q,v state of the
  // RigidBodyTree.
  const BasicVector<double>* input_vector =
      EvalVectorInput(context, kPortIndex);
  const auto q = input_vector->get_value().head(tree_.get_num_positions());
  if (log_ != nullptr) {
    log_->AddData(context.get_time(), q);
  }

  // Translates the input vector into an array of bytes representing an LCM
  // message.
  std::vector<uint8_t> message_bytes;
  draw_message_translator_.Serialize(context.get_time(), BasicVector<double>{q},
                                     &message_bytes);

  // Publishes onto the specified LCM channel.
  lcm_->Publish("DRAKE_VIEWER_DRAW", message_bytes.data(),
                message_bytes.size(), context.get_time());
}

void DrakeVisualizer::PublishLoadRobot() const {
  drake::lcm::Publish(lcm_, "DRAKE_VIEWER_LOAD_ROBOT", load_message_);
}

}  // namespace systems
}  // namespace drake
