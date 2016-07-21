#include "drake/systems/framework/system3.h"

#include <algorithm>
#include <cctype>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/systems/framework/context3.h"
#include "drake/systems/framework/primitives/cascade3.h"

#include "drake/systems/framework/system3.h"
#include "drake/systems/framework/system3_input.h"
#include "drake/systems/framework/system3_output.h"

#include "gtest/gtest.h"

using std::unique_ptr;
using std::make_unique;
using std::string;
using std::vector;

namespace drake {
namespace systems {
namespace {

// This is a System with no inputs and a single output port that delivers
// an std::vector<std::string> value of arbitrary length.

// TODO(sherm1) Should be able to derive this from AbstractSystem3. Currently
// the primitives like Cascade are System3 so won't work with AbstractSystems.
class SourceForStrings: public System3<double> {
public:
 SourceForStrings(const string& name, const vector<string> strings)
     : System3<double>(name), strings_(strings) {

   // An empty string vector is a good enough model -- we can resize later.
   auto model_value = make_unique<Value<vector<string>>>(vector<string>());

   AddOutputPort(std::make_unique<OutputPort3>(std::move(model_value)));
 }

private:
  // Implement System<T> interface.

  std::unique_ptr<AbstractContext3> DoCreateEmptyContext() const override {
    return make_unique<Context3<double>>();
  }

  // Set output to the string list that was provided in the constructor.
  // Since this is cached it will only be called once.
  void DoCalcOutputPort(const AbstractContext3& context, int port_num,
                        AbstractValue* value) const override {
    DRAKE_ASSERT(port_num == 0);

    auto result = value->GetMutableValue<vector<string>>();
    *result = strings_;
  }

  vector<string> strings_;
};

// Takes an std::vector<std::string> on an input port and writes one of the
// same size to its output port but with all the strings upshifted.

// TODO(sherm1) Should be able to derive this from AbstractSystem3.
class Upshift : public System3<double> {
 public:
  Upshift(const std::string& name) : System3<double>(name) {
    // Create an unrestricted input port.
    AddInputPort(std::make_unique<InputPort3>());

    // Create an output port that is assigned a cache entry that can hold
    // a vector<string> of any length. Start out empty.
    auto model_value = make_unique<Value<vector<string>>>(vector<string>());
    AddOutputPort(std::make_unique<OutputPort3>(std::move(model_value)));
  }

 private:
  // Implement System<T> interface.

  std::unique_ptr<AbstractContext3> DoCreateEmptyContext() const override {
    return make_unique<Context3<double>>();
  }

  // Set output to the upshifted string input.
  void DoCalcOutputPort(const AbstractContext3& context, int port_num,
                        AbstractValue* result) const override {
    DRAKE_ASSERT(port_num == 0);

    // Get the value of the input port and downcast it to the expected type.
    const AbstractValue& value = EvalInputPort(context, 0);
    const auto& in_strings = value.GetValue<vector<string>>();

    // Downcast the supplied result argument to the concrete type and
    // initialize it to the input value.
    auto out_strings = result->GetMutableValue<vector<string>>();
    *out_strings = in_strings;

    // Upshift the result in place.
    for (auto& one : *out_strings)
      std::transform(one.cbegin(), one.cend(), one.begin(), ::toupper);
  }

};

// Test that we can make a System diagram using the two string systems and
// use it to do a non-numerical computation.
GTEST_TEST(System3Test, StringValuedPorts) {
  vector<string> strings{"one", "two", "three"};
  vector<string> expected{"ONE", "TWO", "THREE"};

  // Use a Cascade system to conveniently wire up string source to upshifter.
  Cascade3<double> diagram("UPPERizer",
    make_unique<SourceForStrings>("source", strings),
    make_unique<Upshift>("upshift"));

  auto context = diagram.CreateDefaultContext();

  // Diagram has no input ports and one output port and context should match.
  ASSERT_EQ(0, context->get_num_input_ports());
  ASSERT_EQ(1, context->get_num_output_ports());

  // Get the result and downcast it to the expected type.
  const auto& result = diagram.EvalOutputPort(*context, 0);
  const auto& upper = result.GetValue<vector<string>>();

  // Check that we got the upshifted strings.
  EXPECT_EQ(upper, expected);
}

}  // namespace
}  // namespace systems
}  // namespace drake
