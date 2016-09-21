//
// Created by drum on 9/20/16.
//

#ifndef DRAKE_MY_SPRING_MASS_SYSTEM_H
#define DRAKE_MY_SPRING_MASS_SYSTEM_H

#include "drake/systems/framework/examples/spring_mass_system.h"

namespace drake {
  namespace systems {
    namespace {

      class MySpringMassSystem : public SpringMassSystem {
      public:
        // Pass through to SpringMassSystem, except add sample rate in samples/s.
        MySpringMassSystem(double stiffness, double mass, double sample_rate)
            : SpringMassSystem(stiffness, mass, false /*no input force*/),
              sample_rate_(sample_rate) {}

        int get_publish_count() const { return publish_count_; }

        int get_update_count() const { return update_count_; }

      private:
        // Publish t q u to standard output.
        void DoPublish(const Context<double> &context) const override {
          ++publish_count_;
        }

        void DoUpdate(Context<double> *context,
                      const SampleActions &actions) const override {
          ++update_count_;
        }

        // Force a sample at the next multiple of the sample rate. If the current
        // time is exactly at a sample time, we assume the sample has already been
        // done and return the following sample time. That means we don't get a
        // sample at 0 but will get one at the end.
        void DoCalcNextSampleTime(const Context<double> &context,
                                  SampleActions *actions) const override {
          if (sample_rate_ <= 0.) {
            actions->time = std::numeric_limits<double>::infinity();
            return;
          }

          // For reliable behavior, convert floating point times into integer
          // sample counts. We want the ceiling unless it is the same as the floor.
          const int prev =
              static_cast<int>(std::floor(context.get_time() * sample_rate_));
          const int next =
              static_cast<int>(std::ceil(context.get_time() * sample_rate_));
          const int which = next == prev ? next + 1 : next;

          // Convert the next sample count back to a time to return.
          const double next_sample = which / sample_rate_;
          actions->time = next_sample;
        }

        double sample_rate_{0.};  // Default is "don't sample".

        mutable int publish_count_{0};
        mutable int update_count_{0};
      }; // MySpringMassSystem
    } // (blank)
  } // systems
  } // drake

#endif //DRAKE_MY_SPRING_MASS_SYSTEM_H
