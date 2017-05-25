#include "drake/systems/lcm/translator.h"
#include "drake/systems/lcm/new_lcm_publisher_system.h"
#include "drake/systems/lcm/translator_system.h"
#include "drake/lcm/drake_mock_lcm.h"

#include "drake/systems/lcm/translator_system.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace lcm {
namespace {

struct TestData {
  TestData(int size) {
    vector.resize(size, 0);
    names.resize(size);
  }

  std::vector<std::string> names;
  std::vector<double> vector;
};

class TestDataTranslator : public drake::lcm::TranslatorBase<TestData, lcmt_drake_signal> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestDataTranslator)
  explicit TestDataTranslator() {}

  void InitializeMessage(lcmt_drake_signal* msg) const override {
    msg->dim = 0;
    msg->val.resize(msg->dim, 0);
    msg->coord.resize(msg->dim);
    msg->timestamp = 0;
  }

  void Decode(const lcmt_drake_signal& msg,
      double* time, TestData* data) const override {
    DRAKE_DEMAND(msg.dim == static_cast<int>(data->names.size()));

    // Saves the values from the LCM message into vector_base.
    // Assumes that the order of the values are identical in both.
    for (int i = 0; i < msg.dim; ++i) {
      data->vector[i] = msg.val[i];
      data->names[i] = msg.coord[i];
    }

    *time = static_cast<double>(msg.timestamp) / 1e3;
  }

  void Encode(double time, const TestData& data,
      lcmt_drake_signal* msg) const override {

    msg->dim = data.names.size();
    msg->val.resize(msg->dim);
    msg->coord.resize(msg->dim);
    msg->timestamp = static_cast<int64_t>(time * 1e3);

    for (int i = 0; i < msg->dim; ++i) {
      msg->val[i] = data.vector[i];
      msg->coord[i] = data.names[i];
    }
  }
};

template <typename T>
class TestVector : public BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestVector)

  TestVector(int size) : BasicVector<T>(size) {
    names_.resize(size);
  }

  TestVector* DoClone() const override {
    TestVector* ret = new TestVector(this->size());
    ret->names_ = names_;
    return ret;
  }

  const std::vector<std::string>& get_names() const { return names_; }
  std::vector<std::string>& get_mutable_names() { return names_; }

 private:
  std::vector<std::string> names_;
};

class TestVectorTranslator : public drake::lcm::TranslatorBase<TestVector<double>, lcmt_drake_signal> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestVectorTranslator)
  explicit TestVectorTranslator(int size) : vector_size_(size) {}

  void InitializeMessage(lcmt_drake_signal* msg) const override {
    msg->dim = 0;
    msg->val.resize(msg->dim, 0);
    msg->coord.resize(msg->dim);
    msg->timestamp = 0;
  }

  void Decode(const lcmt_drake_signal& msg,
      double* time, TestVector<double>* vector) const override {
    DRAKE_DEMAND(msg.dim == static_cast<int>(vector->size()));

    // Saves the values from the LCM message into vector_base.
    // Assumes that the order of the values are identical in both.
    for (int i = 0; i < msg.dim; ++i) {
      vector->SetAtIndex(i, msg.val[i]);
      vector->get_mutable_names()[i] = msg.coord[i];
    }

    *time = static_cast<double>(msg.timestamp) / 1e3;
  }

  void Encode(double time, const TestVector<double>& vector,
      lcmt_drake_signal* msg) const override {

    msg->dim = vector.size();
    msg->val.resize(msg->dim);
    msg->coord.resize(msg->dim);
    msg->timestamp = static_cast<int64_t>(time * 1e3);

    for (int i = 0; i < msg->dim; ++i) {
      msg->val[i] = vector.GetAtIndex(i);
      msg->coord[i] = vector.get_names()[i];
    }
  }

  int get_vector_size() const { return vector_size_; }

 private:
  int vector_size_{0};
};

GTEST_TEST(TranslatorTest, ToLcmMessageBasicVectorVersion) {
  const int kVecSize = 2;

  ToLcmMessageTranslator<TestVector<double>, lcmt_drake_signal> dut(
      TestVector<double>(kVecSize),
      std::make_unique<TestVectorTranslator>(kVecSize));

  auto context = dut.CreateDefaultContext();
  auto output = dut.AllocateOutput(*context);

  {
    auto vec = std::make_unique<TestVector<double>>(kVecSize);
    for (int i = 0; i < kVecSize; ++i) {
      vec->SetAtIndex(i, i);
      vec->get_mutable_names()[i] = std::to_string(i);
    }

    context->FixInputPort(0, std::move(vec));
  }

  const double time = 233;
  context->set_time(time);

  dut.CalcOutput(*context, output.get());
  const lcmt_drake_signal& msg =
        output->get_data(0)->GetValue<lcmt_drake_signal>();

  EXPECT_EQ(time * 1e3, msg.timestamp);

  EXPECT_EQ(msg.dim, kVecSize);
  for (int i = 0; i < msg.dim; ++i) {
    EXPECT_EQ(msg.coord[i], std::to_string(i));
    EXPECT_EQ(msg.val[i], i);
  }
}

GTEST_TEST(TranslatorTest, ToLcmMessageAbstractValVersion) {
  const int kVecSize = 2;

  ToLcmMessageTranslator<TestData, lcmt_drake_signal> dut(
      TestData(kVecSize),
      std::make_unique<TestDataTranslator>());

  auto context = dut.CreateDefaultContext();
  auto output = dut.AllocateOutput(*context);

  {
    TestData data(kVecSize);
    for (int i = 0; i < kVecSize; ++i) {
      data.vector[i] = i;
      data.names[i] = std::to_string(i);
    }

    context->FixInputPort(0, AbstractValue::Make<TestData>(data));
  }

  const double time = 233;
  context->set_time(time);

  dut.CalcOutput(*context, output.get());
  const lcmt_drake_signal& msg =
        output->get_data(0)->GetValue<lcmt_drake_signal>();

  EXPECT_EQ(time * 1e3, msg.timestamp);

  EXPECT_EQ(msg.dim, kVecSize);
  for (int i = 0; i < msg.dim; ++i) {
    EXPECT_EQ(msg.coord[i], std::to_string(i));
    EXPECT_EQ(msg.val[i], i);
  }
}

GTEST_TEST(TranslatorTest, FromLcmMessageAbstractValVersion) {
  const int kVecSize = 2;

  FromLcmMessageTranslator<TestData, lcmt_drake_signal> dut(
      TestData(kVecSize),
      std::make_unique<TestDataTranslator>());

  auto context = dut.CreateDefaultContext();
  auto output = dut.AllocateOutput(*context);

  lcmt_drake_signal msg;
  msg.dim = kVecSize;
  msg.val.resize(msg.dim);
  msg.coord.resize(msg.dim);
  for (int i = 0; i < kVecSize; ++i) {
    msg.val[i] = i;
    msg.coord[i] = std::to_string(i);
  }
  context->FixInputPort(0, AbstractValue::Make<lcmt_drake_signal>(msg));

  dut.CalcOutput(*context, output.get());
  const TestData& data =
      output->get_data(0)->GetValue<TestData>();

  EXPECT_EQ(data.names.size(), kVecSize);
  EXPECT_EQ(data.vector.size(), kVecSize);
  for (int i = 0; i < kVecSize; ++i) {
    EXPECT_EQ(data.names[i], std::to_string(i));
    EXPECT_EQ(data.vector[i], i);
  }
}

GTEST_TEST(TranslatorTest, FromLcmMessageBasicVectorVersion) {
  const int kVecSize = 2;

  FromLcmMessageTranslator<TestVector<double>, lcmt_drake_signal> dut(
      TestVector<double>(kVecSize),
      std::make_unique<TestVectorTranslator>(kVecSize));

  auto context = dut.CreateDefaultContext();
  auto output = dut.AllocateOutput(*context);

  lcmt_drake_signal msg;
  msg.dim = kVecSize;
  msg.val.resize(msg.dim);
  msg.coord.resize(msg.dim);
  for (int i = 0; i < kVecSize; ++i) {
    msg.val[i] = i;
    msg.coord[i] = std::to_string(i);
  }
  context->FixInputPort(0, AbstractValue::Make<lcmt_drake_signal>(msg));

  dut.CalcOutput(*context, output.get());

  const TestVector<double>* const vector =
      dynamic_cast<const TestVector<double>*>(output->get_vector_data(0));

  EXPECT_EQ(vector->size(), kVecSize);
  for (int i = 0; i < kVecSize; ++i) {
    EXPECT_EQ(vector->GetAtIndex(i), i);
    EXPECT_EQ(vector->get_names()[i], std::to_string(i));
  }
}











/*
GTEST_TEST(NewLcmPublisherTest, BasicVectorVersion) {
  drake::lcm::DrakeMockLcm lcm;
  const int kVecSize = 2;

  NewLcmPublisherSystem<VectorBase<double>, lcmt_drake_signal> dut(
      "test", std::make_unique<MyLcmtDrakeSignalTranslator>(kVecSize), kVecSize, &lcm);

  auto context = dut.CreateDefaultContext();
  auto output = dut.AllocateOutput(*context);

  {
    auto vec = std::make_unique<BasicVector<double>>(kVecSize);
    for (int i = 0; i < kVecSize; ++i)
      vec->SetAtIndex(i, i);

    context->FixInputPort(0, std::move(vec));
  }

  const double time = 233;
  context->set_time(time);

  dut.Publish(*context.get());

  const std::vector<uint8_t>& published_message_bytes =
      lcm.get_last_published_message("test");

  drake::lcmt_drake_signal received_message;
  received_message.decode(&published_message_bytes[0], 0,
      published_message_bytes.size());

  EXPECT_EQ(time * 1e3, received_message.timestamp);

  EXPECT_EQ(received_message.dim, kVecSize);
  for (int i = 0; i < received_message.dim; ++i) {
    EXPECT_EQ(received_message.coord[i], "");
    EXPECT_EQ(received_message.val[i], i);
  }
}

GTEST_TEST(NewLcmPublisherTest, AbstractValueVersion) {
  drake::lcm::DrakeMockLcm lcm;
  const int kVecSize = 2;

  NewLcmPublisherSystem<TestData, lcmt_drake_signal> dut(
      "test", std::make_unique<TestDataTranslator>(), &lcm);

  auto context = dut.CreateDefaultContext();
  auto output = dut.AllocateOutput(*context);

  {
    TestData data(kVecSize);
    for (int i = 0; i < kVecSize; ++i) {
      data.vector[i] = i;
      data.names[i] = std::to_string(i);
    }

    context->FixInputPort(0, AbstractValue::Make<TestData>(data));
  }

  const double time = 233;
  context->set_time(time);

  dut.Publish(*context.get());

  const std::vector<uint8_t>& published_message_bytes =
      lcm.get_last_published_message("test");

  drake::lcmt_drake_signal received_message;
  received_message.decode(&published_message_bytes[0], 0,
      published_message_bytes.size());

  EXPECT_EQ(time * 1e3, received_message.timestamp);

  EXPECT_EQ(received_message.dim, kVecSize);
  for (int i = 0; i < received_message.dim; ++i) {
    EXPECT_EQ(received_message.coord[i], std::to_string(i));
    EXPECT_EQ(received_message.val[i], i);
  }
}
*/

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake
