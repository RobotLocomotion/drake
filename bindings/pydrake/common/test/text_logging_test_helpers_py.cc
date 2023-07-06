#include <atomic>
#include <chrono>
#include <thread>

#include "pybind11/pybind11.h"

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"

namespace py = pybind11;

namespace drake {
namespace pydrake {
namespace {

// Logs one message at every available level.
void do_log_test() {
  drake::log()->trace("Test Trace message");
  drake::log()->debug("Test Debug message");
  drake::log()->info("Test Info message");
  drake::log()->warn("Test Warn message");
  drake::log()->error("Test Error message");
  drake::log()->critical("Test Critical message");
}

// Launches a C++ thread that logs periodically.
class Worker {
 public:
  Worker() = default;

  ~Worker() {
    if (thread_ != nullptr) {
      Stop();
    }
  }

  void Start() {
    DRAKE_THROW_UNLESS(thread_ == nullptr);
    keep_running_.store(true);
    thread_ = std::make_unique<std::thread>([this]() { this->ThreadMain(); });
  }

  void Stop() {
    DRAKE_THROW_UNLESS(thread_ != nullptr);
    pybind11::gil_scoped_release guard;
    keep_running_.store(false);
    thread_->join();
    thread_.reset();
  }

  void ThreadMain() {
    while (keep_running_.load()) {
      drake::log()->info("Thread info message");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

 private:
  std::atomic<bool> keep_running_{true};
  std::unique_ptr<std::thread> thread_;
};

}  // namespace

PYBIND11_MODULE(text_logging_test_helpers, m) {
  m.doc() = "Test text logging";

  m.def("do_log_test", &do_log_test);

  {
    using Class = Worker;
    py::class_<Class>(m, "Worker")
        .def(py::init<>())
        .def("Start", &Class::Start)
        .def("Stop", &Class::Stop);
  }
}

}  // namespace pydrake
}  // namespace drake
