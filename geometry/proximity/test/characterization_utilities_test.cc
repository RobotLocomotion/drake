#include "drake/geometry/proximity/test/characterization_utilities.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using math::RigidTransformd;
using std::vector;

namespace {

/* These unit tests test the mathematical underpinnings of the
 CharacterizeResultTest. Specifically, it tests that the pieces of code which
 contribute to *configuring* a test are valid.

 It also tests the code execution by creating a *fake* callback whose results
 are controlled by the test to make sure that the test is detecting error that
 is *known* to exist.  */

/* The MakeFclShapeTest tests confirm that Drake shape specifications turn into
 the expected fcl geometries. */
// TODO(SeanCurtis-TRI) Consider templating this on T so I can show I get the
//  same thing across double and AutoDiffXd.

GTEST_TEST(MakeFclShapeTest, Sphere) {
  const Sphere sphere(0.7);
  auto fcl_geometry = MakeFclShape(sphere).object();
  const auto& fcl_sphere = dynamic_cast<fcl::Sphered&>(*fcl_geometry);
  EXPECT_EQ(fcl_sphere.radius, sphere.radius());
}

/* Dummy callback, callback data, and traits for the CharacterizeResultTest.  */

/* The callback and its data is designed to facilitate testing the
 RunCharacterization functions. That function invokes the callback multiple
 times (at least 4 times per configuration), with a fixed *expectation* on what
 the returned value will be. For each run, it measures the error and remembers
 the *worst* error. It then confirms the worst error is on scale with the
 expected level of precision.

 So, for a given "true answer" t and expected error e, we need a callback that
 will produce values in the range [t - e, t + e]. To make the test interesting
 we further need:

   - We have to have at least one result with the expected error.
   - We don't want that result first or last (to make sure the algorithm isn't
     accidentally saving the first or last values).
   - It needs to support an aribtrary number of invocations (although we'll
     require at least two).

 So, fortunately, we don't have to guess. In the tests we run, we know exactly
 how many configurations we're running, so we can announce the total number of
 expected invocations. Therefore, we'll initialize a test by declaring the
 number of configurations and the expected error, the result of which, we'll
 cache a sequence of values that satisfy the properties above.

 That means each test must invoke SetValueSequence() prior to invoking
 RunCharacterization.  */
template <typename T>
class DummyCallbackData {
 public:
  explicit DummyCallbackData(std::vector<T>* results) : results_(results) {
    DRAKE_DEMAND(results != nullptr);
  }

  static void SetValueSequence(int invocation_count, double truth,
                               double precision) {
    DRAKE_DEMAND(invocation_count >= 2);
    invocations = 0;
    sequence.clear();
    sequence.reserve(invocation_count);
    /* We'll create a kind of damped sinusoidal signal. The first result will
     be truth, the second will be the truth + precision, and all other answers
     will have decaying error.

     t+p  │      o
          │
          │                  o
       t  ├─o───────────────────────
          │
          │            o
      t-p │

           1    2     3     4  ...
      */
    sequence.push_back(truth);
    /* We need this to be <= truth + precision, so we'll knock down the
     precision ever so slightly to ensure it. */
    sequence.push_back(truth + precision * 0.9999);
    const double decay = precision / invocation_count;
    for (int i = 2; i < invocation_count; ++i) {
      /* Linear decay to *almost* truth. */
      precision -= decay;
      /* Sign-alternating signal. */
      sequence.push_back(truth + (i % 2 == 0 ? -1 : 1) * precision);
    }
  }

  static T NextValue() { return sequence.at(invocations++); }

  static size_t get_invocations() { return invocations; }

  std::vector<T>& results() { return *results_; }
  const std::vector<T>& results() const { return *results_; }

 private:
  static size_t invocations;
  static std::vector<T> sequence;
  std::vector<T>* results_;
};

template <typename T>
size_t DummyCallbackData<T>::invocations = 0;

template <typename T>
std::vector<T> DummyCallbackData<T>::sequence;

template <typename T>
bool DummyCallback(fcl::CollisionObjectd*, fcl::CollisionObjectd*, void* data) {
  auto& dummy_data = *static_cast<DummyCallbackData<T>*>(data);
  dummy_data.results().push_back(DummyCallbackData<T>::NextValue());
  return false;
}

/* Make sure the sequence satisfies the properties given above. We want to make
 sure it supports the declared invocation count and returns values that are all
 within the precision (with the second value having exactly the expected
 error). */
GTEST_TEST(DummyCallbackTest, ConfirmErrorSequence) {
  /* Combination of various counts, truth values, and precisions. */
  for (int count : {4, 8, 12}) {
    for (double truth : {-0.5, 0.75}) {
      for (double precision : {1e-3, 1e-10, 1e-15}) {
        DummyCallbackData<double>::SetValueSequence(count, truth, precision);
        std::vector<double> results;
        DummyCallbackData<double> data(&results);
        for (int i = 0; i < count; ++i) {
          DummyCallback<double>(nullptr, nullptr, &data);
        }
        /* The second value has error equal to the declared precision. */
        ASSERT_NEAR(results[1], truth, precision);
        /* All *other* reported values have strictly less error. */
        for (int i = 0; i < count; ++i) {
          if (i == 1) continue;
          ASSERT_LT(std::abs(results[i] - truth), precision);
        }
      }
    }
  }
}

template <typename T>
class DummyImplementation : public DistanceCallback<T> {
 public:
  bool Invoke(
      fcl::CollisionObjectd* obj_A, fcl::CollisionObjectd* obj_B,
      const CollisionFilterLegacy*,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>*) override {
    DummyCallbackData<T> data(&results_);
    return DummyCallback<T>(obj_A, obj_B, &data);
  }

  void ClearResults() override { results_.clear(); }

  int GetNumResults() const override {
    return static_cast<int>(results_.size());
  }

  T GetFirstSignedDistance() const override { return results_[0]; }

 private:
  vector<T> results_;
};

/* CharacterizeResultTest::RunCharacterization runs a number of configurations
 through a callback. Its primary task is to execute the callback and evaluate
 the results, confirming the observed, worst-case error and comparing that with
 expectations. This must be correct for us to trust it to report errors on
 production callbacks.

 In this case, we use the DummyCallback (documented above) with *known*
 semantics. Given the number of known invocations, a truth value, and a
 precision we want to predict, we want to confirm that the test detects that
 precision.

 We need to know the number of expected invocations. For each unique
 configuration, RunCharacterization invokes the callback multiple times:

   - Creates N poses mapping the configuration to another, arbitrary frame.
     N is the number of poses reported by X_WAs().
   - Once as geometry pair (A, B) and once for (B, A) (in case there's an
     asymmetry in the algorithm that produces bad answers).

  Therefore, for one configuration there will be 2N calls to callback. */
class RunCharacterizationTest
    : public CharacterizeResultTest<double> {
 public:
  RunCharacterizationTest()
      : CharacterizeResultTest<double>(
            std::make_unique<DummyImplementation<double>>()) {}
};

/* Create a single configuration and confirm the expected results.  */
TEST_F(RunCharacterizationTest, CustomConfigurations) {
  const int call_count = 2 * static_cast<int>(X_WAs().size());
  const double truth = 1;
  for (const double target_precision : {1e-2, 1e-7, 1e-14}) {
    DummyCallbackData<double>::SetValueSequence(call_count, truth,
                                                target_precision);
    const Expectation expectation{true, target_precision, ""};
    const Configuration<double> config{{}, truth};
    /* If this test passes, it detected the expected precision. */
    RunCharacterization(expectation, this->sphere(), this->sphere(true),
                        {config});
    /* If this test passes, it did all the required work. */
    EXPECT_EQ(DummyCallbackData<double>::get_invocations(), call_count);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
