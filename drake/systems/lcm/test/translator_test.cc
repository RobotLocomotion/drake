#include <gtest/gtest.h>

#include "drake/lcmt_drake_signal.hpp"

#include "drake/systems/lcm/translator_system.h"

namespace drake {
namespace systems {
namespace lcm {
namespace {

// Some arbitrary struct that holds the same amount of information as a
// lcmt_drake_signal message.
struct TestData {
  explicit TestData(int size) {
    vector.resize(size, 0);
    names.resize(size);
  }

  std::vector<std::string> names;
  std::vector<double> vector;
};

// A derived BasicVector that holds the same amount of information as a
// lcmt_drake_signal message.
template <typename T>
class TestVector : public BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestVector)

  explicit TestVector(int size) : BasicVector<T>(size) { names_.resize(size); }

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

// A translator between TestData and lcmt_drake_signal.
class TestDataTranslator
    : public drake::lcm::TranslatorBase<TestData, lcmt_drake_signal> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestDataTranslator)

  explicit TestDataTranslator(int size) : default_data_(size) {
    // Message defaults to same size, with 0 value, and empty strings.
    default_msg_.dim = size;
    default_msg_.val.resize(default_msg_.dim, 0);
    default_msg_.coord.resize(default_msg_.dim);
    default_msg_.timestamp = 0;
  }

  const TestData& get_default_data() const override { return default_data_; }

  const lcmt_drake_signal& get_default_msg() const override {
    return default_msg_;
  }

  // Resizes @p data if its dimension doesn't match @p msg.
  void Decode(const lcmt_drake_signal& msg, double* time,
              TestData* data) const override {
    if (msg.dim != static_cast<int>(data->names.size())) {
      *data = TestData(msg.dim);
    }

    for (int i = 0; i < msg.dim; ++i) {
      data->vector[i] = msg.val[i];
      data->names[i] = msg.coord[i];
    }

    *time = static_cast<double>(msg.timestamp) / 1e3;
  }

  // Resizes @p msg if its dimension doesn't match @p data.
  void Encode(double time, const TestData& data,
              lcmt_drake_signal* msg) const override {
    if (msg->dim != static_cast<int>(data.names.size())) {
      msg->dim = static_cast<int>(data.names.size());
      msg->val.resize(msg->dim);
      msg->coord.resize(msg->dim);
    }

    msg->timestamp = static_cast<int64_t>(time * 1e3);
    for (int i = 0; i < msg->dim; ++i) {
      msg->val[i] = data.vector[i];
      msg->coord[i] = data.names[i];
    }
  }

 private:
  lcmt_drake_signal default_msg_;
  TestData default_data_;
};

// A translator between TestVector<double> and lcmt_drake_signal.
class TestVectorTranslator
    : public drake::lcm::TranslatorBase<TestVector<double>, lcmt_drake_signal> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestVectorTranslator)

  explicit TestVectorTranslator(int size) : default_vector_(size) {
    // Message defaults to same size, with 0 value, and empty strings.
    default_msg_.dim = size;
    default_msg_.val.resize(default_msg_.dim, 0);
    default_msg_.coord.resize(default_msg_.dim);
    default_msg_.timestamp = 0;
  }

  const TestVector<double>& get_default_data() const override {
    return default_vector_;
  }

  const lcmt_drake_signal& get_default_msg() const override {
    return default_msg_;
  }

  // Assumes that @p vector's dimension is the same as @p msg's. Because
  // @p vector cannot be resized / assigned.
  void Decode(const lcmt_drake_signal& msg, double* time,
              TestVector<double>* vector) const override {
    DRAKE_DEMAND(msg.dim == static_cast<int>(vector->size()));

    for (int i = 0; i < msg.dim; ++i) {
      vector->SetAtIndex(i, msg.val[i]);
      vector->get_mutable_names()[i] = msg.coord[i];
    }

    *time = static_cast<double>(msg.timestamp) / 1e3;
  }

  // Resizes @p msg if its dimension doesn't match @p data.
  void Encode(double time, const TestVector<double>& vector,
              lcmt_drake_signal* msg) const override {
    if (msg->dim != vector.size()) {
      msg->dim = vector.size();
      msg->val.resize(msg->dim);
      msg->coord.resize(msg->dim);
    }

    msg->timestamp = static_cast<int64_t>(time * 1e3);
    for (int i = 0; i < msg->dim; ++i) {
      msg->val[i] = vector.GetAtIndex(i);
      msg->coord[i] = vector.get_names()[i];
    }
  }

 private:
  TestVector<double> default_vector_;
  lcmt_drake_signal default_msg_;
};

// Tests a LcmEncoderSystem from TestVector<double> to lcmt_drake_signal using
// a TestVectorTranslator.
GTEST_TEST(TranslatorTest, ToLcmMessageBasicVectorVersion) {
  const int kVecSize = 2;

  LcmEncoderSystem<TestVector<double>, lcmt_drake_signal> dut(
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

// Tests a LcmEncoderSystem from TestData to lcmt_drake_signal using a
// TestDataTranslator.
GTEST_TEST(TranslatorTest, ToLcmMessageAbstractValVersion) {
  const int kVecSize = 2;

  LcmEncoderSystem<TestData, lcmt_drake_signal> dut(
      std::make_unique<TestDataTranslator>(kVecSize));

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

// Tests a LcmDecoderSystem from lcmt_drake_signal to TestData using a
// TestDataTranslator.
GTEST_TEST(TranslatorTest, FromLcmMessageAbstractValVersion) {
  const int kVecSize = 2;

  LcmDecoderSystem<TestData, lcmt_drake_signal> dut(
      std::make_unique<TestDataTranslator>(kVecSize));

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
  const TestData& data = output->get_data(0)->GetValue<TestData>();

  EXPECT_EQ(data.names.size(), kVecSize);
  EXPECT_EQ(data.vector.size(), kVecSize);
  for (int i = 0; i < kVecSize; ++i) {
    EXPECT_EQ(data.names[i], std::to_string(i));
    EXPECT_EQ(data.vector[i], i);
  }
}

// Tests a LcmDecoderSystem from lcmt_drake_signal to TestVector<double> using
// a TestVectorTranslator.
GTEST_TEST(TranslatorTest, FromLcmMessageBasicVectorVersion) {
  const int kVecSize = 2;

  LcmDecoderSystem<TestVector<double>, lcmt_drake_signal> dut(
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

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake
