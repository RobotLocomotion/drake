#include "drake/systems/primitives/signal_logger_playback.h"

#include <iomanip>
#include <vector>

#include "drake/common/extract_double.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace systems {

template <typename T>
SignalLoggerPlayback<T>::SignalLoggerPlayback(int input_size,
                                              int batch_allocation_size)
    : SignalLogger<T>(input_size, batch_allocation_size) {
  this->DeclareOutputPort(kVectorValued, input_size);
}

template <typename T>
void SignalLoggerPlayback<T>::InitializePlayback() {
  if (!playback_active_) {
    auto times = this->sample_times();
    // NOTE: The SignalLogger parent class will record signal for multiple
    // identical time stamps.  This culls the duplicates as required by the
    // PiecewisePolynomial.
    std::vector<int> included_times;
    included_times.reserve(times.rows());
    std::vector<double> breaks;
    included_times.push_back(0);
    breaks.push_back(ExtractDoubleOrThrow<T>(times(0)));
    int last = 0;
    for (int i = 1; i < times.rows(); ++i) {
      double val = times(i);
      if (val != breaks[last]) {
        breaks.push_back(ExtractDoubleOrThrow<T>(val));
        included_times.push_back(i);
        ++last;
      }
    }

    auto sample_data = this->data();
    std::vector<MatrixX<T>> knots;
    knots.reserve(sample_data.cols());
    for (int c : included_times) {
      knots.push_back(sample_data.col(c));
    }
    playback_data_ = PiecewisePolynomial<T>::ZeroOrderHold(breaks, knots);
    playback_active_ = true;
  }
}

template <typename T>
void SignalLoggerPlayback<T>::ResetPlayback(Simulator<T> *simulator) {
  auto context = simulator->get_mutable_context();
  simulator->set_target_realtime_rate(1.0);
  simulator->get_mutable_integrator()->set_maximum_step_size(1/60.0);
  context->set_time(0.0);
  simulator->ResetStatistics();
}

template <typename T>
void SignalLoggerPlayback<T>::DoCalcOutput(const Context<T>& context,
                                           SystemOutput<T>* output) const {
  // Evaluates the state output port.
  BasicVector<T>* output_vector = output->GetMutableVectorData(0);
  auto output_data = output_vector->get_mutable_value();

  if (playback_active_) {
    // NOTE: this is safe for *any* time value because the time value gets
    // clamped to the domain of the playback data.
    T time = context.get_time();
    output_data = playback_data_.value(time);
  } else {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    output_data = input->get_value();
  }
}

template <typename T>
void SignalLoggerPlayback<T>::DoPublish(
    const Context<T>& context) const {
  if (!playback_active_) SignalLogger<T>::DoPublish(context);
}

template class SignalLoggerPlayback<double>;
// TODO(SeanCurtis-TRI): Specialize on AutoDiffXd if the need is shown; as is,
// attempting to compile with AutoDiff is a real pain.

}  // namespace systems
}  // namespace drake
