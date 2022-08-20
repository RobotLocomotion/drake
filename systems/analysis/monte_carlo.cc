#include "drake/systems/analysis/monte_carlo.h"

#include <atomic>
#include <condition_variable>
#include <list>
#include <mutex>
#include <thread>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {
namespace analysis {

double RandomSimulation(
    const SimulatorFactory& make_simulator, const ScalarSystemFunction& output,
    const double final_time, RandomGenerator* const generator) {
  auto simulator = make_simulator(generator);

  const System<double>& system = simulator->get_system();
  system.SetRandomContext(&simulator->get_mutable_context(), generator);

  simulator->AdvanceTo(final_time);

  return output(system, simulator->get_context());
}

namespace {

// Serial (single-threaded) implementation of MonteCarloSimulation.
std::vector<RandomSimulationResult> MonteCarloSimulationSerial(
    const SimulatorFactory& make_simulator, const ScalarSystemFunction& output,
    const double final_time, const int num_samples,
    RandomGenerator* const generator) {
  std::vector<RandomSimulationResult> simulation_results;
  simulation_results.reserve(num_samples);

  for (int sample = 0; sample < num_samples; ++sample) {
    RandomSimulationResult simulation_result(*generator);
    simulation_result.output =
        RandomSimulation(make_simulator, output, final_time, generator);
    simulation_results.push_back(std::move(simulation_result));
  }

  return simulation_results;
}

// Encapsulation for a simulator and simulation thread.
class SimulationWorker {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimulationWorker);

  SimulationWorker(std::unique_ptr<Simulator<double>> simulator, int sample_num)
      : simulator_(std::move(simulator)), sample_num_(sample_num) {
    DRAKE_THROW_UNLESS(simulator_ != nullptr);
    DRAKE_THROW_UNLESS(sample_num >= 0);
  }

  ~SimulationWorker() {
    if (simulation_thread_.joinable()) {
      simulation_thread_.join();
    }
  }

  bool simulation_complete() const { return simulation_complete_.load(); }

  int sample_num() const { return sample_num_; }

  // Start simulation thread. Simulation cannot already have been started.
  void DispatchSimulation(
      double final_time, size_t* num_live_workers, std::mutex* cv_mutex,
      std::condition_variable* cv) {
    DRAKE_THROW_UNLESS(num_live_workers != nullptr);
    DRAKE_THROW_UNLESS(cv_mutex != nullptr);
    DRAKE_THROW_UNLESS(cv != nullptr);
    DRAKE_THROW_UNLESS(!simulation_thread_.joinable());
    {
      std::lock_guard<std::mutex> lock(*cv_mutex);
      (*num_live_workers)++;
    }
    simulation_thread_ = std::thread(
        [this, final_time, num_live_workers, cv_mutex, cv]() {
          SimulateTo(final_time, num_live_workers, cv_mutex, cv);
        });
  }

  // Evaluate the scalar system output function on the final state of
  // simulation. Simulation must have completed. If an exception was thrown
  // during simulation, it will be rethrown here.
  double EvaluateScalarSystemFunction(
      const ScalarSystemFunction& output) const {
    DRAKE_THROW_UNLESS(simulation_complete());
    if (simulator_exception_) {
      std::rethrow_exception(simulator_exception_);
    }
    return output(simulator_->get_system(), simulator_->get_context());
  }

 private:
  void SimulateTo(
      double final_time, size_t* num_live_workers, std::mutex* cv_mutex,
      std::condition_variable* cv) {
    try {
      simulator_->AdvanceTo(final_time);
    } catch (...) {
      simulator_exception_ = std::current_exception();
    }
    simulation_complete_.store(true);
    {
      std::lock_guard<std::mutex> lock(*cv_mutex);
      (*num_live_workers)--;
    }
    cv->notify_all();
  }

  std::unique_ptr<Simulator<double>> simulator_;
  std::thread simulation_thread_;
  std::exception_ptr simulator_exception_{nullptr};
  std::atomic<bool> simulation_complete_{false};
  int sample_num_{0};
};

// Parallel (multi-threaded) implementation of MonteCarloSimulation.
std::vector<RandomSimulationResult> MonteCarloSimulationParallel(
    const SimulatorFactory& make_simulator, const ScalarSystemFunction& output,
    const double final_time, const int num_samples,
    RandomGenerator* const generator, const int num_threads) {
  std::vector<RandomSimulationResult> simulation_results;
  simulation_results.reserve(num_samples);

  std::mutex cv_mutex;
  std::condition_variable cv;
  size_t num_live_workers = 0;

  // Storage for active parallel simulation workers.
  std::list<SimulationWorker> active_workers;
  // Keep track of how many simulations have been dispatched already.
  int simulations_dispatched = 0;

  while (active_workers.size() > 0 ||
         simulations_dispatched < num_samples) {
    // Check for completed workers.
    for (auto worker = active_workers.begin();
         worker != active_workers.end();) {
      if (worker->simulation_complete()) {
        const int sample_num = worker->sample_num();
        // This output function evaluation also propagates any exception thrown
        // during simulation execution.
        const double sample_output =
            worker->EvaluateScalarSystemFunction(output);
        simulation_results.at(sample_num).output = sample_output;
        drake::log()->debug(
            "Simulation {} completed with output {}",
            sample_num, sample_output);
        // Erase returns iterator to the next node in the list.
        worker = active_workers.erase(worker);
      } else {
        // Advance to next node in the list.
        ++worker;
      }
    }

    // Dispatch new workers.
    while (static_cast<int>(active_workers.size()) < num_threads
           && simulations_dispatched < num_samples) {
      // Create the simulation result using the current generator state.
      simulation_results.emplace_back(RandomSimulationResult(*generator));

      // Make the simulator.
      auto simulator = make_simulator(generator);
      const auto& system = simulator->get_system();
      system.SetRandomContext(&simulator->get_mutable_context(), generator);

      // Create the worker.
      active_workers.emplace_back(std::move(simulator), simulations_dispatched);

      // Start worker simulation.
      active_workers.back().DispatchSimulation(
          final_time, &num_live_workers, &cv_mutex, &cv);

      drake::log()->debug("Simulation {} dispatched", simulations_dispatched);
      ++simulations_dispatched;
    }

    // Wait until a worker completes.
    std::unique_lock<std::mutex> wait_lock(cv_mutex);
    cv.wait(
        wait_lock,
        [&num_live_workers, &active_workers]() {
          return
              (num_live_workers == 0) ||
              (num_live_workers < active_workers.size());
        });
  }

  return simulation_results;
}

}  // namespace

namespace internal {

int SelectNumberOfThreadsToUse(const int num_parallel_executions) {
  const int hardware_concurrency =
      static_cast<int>(std::thread::hardware_concurrency());

  int num_threads = 0;

  if (num_parallel_executions > 1) {
    num_threads = num_parallel_executions;
    if (num_threads > hardware_concurrency) {
      drake::log()->warn(
          "Provided num_parallel_executions value of {} is greater than the "
          "value of hardware concurrency {} for this computer, this is likely "
          "to result in poor performance",
          num_threads, hardware_concurrency);
    } else {
      drake::log()->debug(
          "Using provided value of {} parallel executions", num_threads);
    }
  } else if (num_parallel_executions == kNoConcurrency) {
    num_threads = 1;
    drake::log()->debug("kNoConcurrency specified, using a single thread");
  } else if (num_parallel_executions == kUseHardwareConcurrency) {
    num_threads = hardware_concurrency;
    drake::log()->debug(
        "kUseHardwareConcurrency specified, using hardware concurrency {}",
        num_threads);
  } else {
    throw std::runtime_error(fmt::format(
        "Specified num_parallel_executions {} is not valid. Valid options are "
        "kNoConcurrency, kUseHardwareConcurrency, or num_parallel_executions "
        ">= 1",
        num_parallel_executions));
  }

  return num_threads;
}

}  // namespace internal

std::vector<RandomSimulationResult> MonteCarloSimulation(
    const SimulatorFactory& make_simulator, const ScalarSystemFunction& output,
    const double final_time, const int num_samples, RandomGenerator* generator,
    const int num_parallel_executions) {
  // Create a generator if the user didn't provide one.
  std::unique_ptr<RandomGenerator> owned_generator;
  if (generator == nullptr) {
    owned_generator = std::make_unique<RandomGenerator>();
    generator = owned_generator.get();
  }

  // Check num_parallel_executions vs the available hardware concurrency.
  // This also serves to sanity-check the num_parallel_executions argument.
  const int num_threads =
      internal::SelectNumberOfThreadsToUse(num_parallel_executions);

  // Since the parallel implementation incurs additional overhead even in the
  // num_threads=1 case, dispatch to the serial implementation in these cases.
  if (num_threads > 1) {
    return MonteCarloSimulationParallel(
        make_simulator, output, final_time, num_samples, generator,
        num_threads);
  } else {
    return MonteCarloSimulationSerial(
        make_simulator, output, final_time, num_samples, generator);
  }
}

}  // namespace analysis
}  // namespace systems
}  // namespace drake
