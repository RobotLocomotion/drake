#include <gtest/gtest.h>

#include "drake/lcmt_drake_signal.hpp"

//#include "drake/lcm/robotlocomotion_translators/pose_t_translator.h"
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
  void Decode(const lcmt_drake_signal& msg, TestData* data) const override {
    if (msg.dim != static_cast<int>(data->names.size())) {
      *data = TestData(msg.dim);
    }

    for (int i = 0; i < msg.dim; ++i) {
      data->vector[i] = msg.val[i];
      data->names[i] = msg.coord[i];
    }
  }

  // Resizes @p msg if its dimension doesn't match @p data.
  void Encode(const TestData& data, lcmt_drake_signal* msg) const override {
    if (msg->dim != static_cast<int>(data.names.size())) {
      msg->dim = static_cast<int>(data.names.size());
      msg->val.resize(msg->dim);
      msg->coord.resize(msg->dim);
    }

    for (int i = 0; i < msg->dim; ++i) {
      msg->val[i] = data.vector[i];
      msg->coord[i] = data.names[i];
    }
  }

  void DecodeTime(const lcmt_drake_signal& msg, double* time) const override {
    *time = static_cast<double>(msg.timestamp) / 1e3;
  }

  void EncodeTime(double time, lcmt_drake_signal* msg) const override {
    msg->timestamp = static_cast<int64_t>(time * 1e3);
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
  void Decode(const lcmt_drake_signal& msg,
              TestVector<double>* vector) const override {
    DRAKE_DEMAND(msg.dim == static_cast<int>(vector->size()));

    for (int i = 0; i < msg.dim; ++i) {
      vector->SetAtIndex(i, msg.val[i]);
      vector->get_mutable_names()[i] = msg.coord[i];
    }
  }

  // Resizes @p msg if its dimension doesn't match @p data.
  void Encode(const TestVector<double>& vector,
              lcmt_drake_signal* msg) const override {
    if (msg->dim != vector.size()) {
      msg->dim = vector.size();
      msg->val.resize(msg->dim);
      msg->coord.resize(msg->dim);
    }

    for (int i = 0; i < msg->dim; ++i) {
      msg->val[i] = vector.GetAtIndex(i);
      msg->coord[i] = vector.get_names()[i];
    }
  }

  void DecodeTime(const lcmt_drake_signal& msg, double* time) const override {
    *time = static_cast<double>(msg.timestamp) / 1e3;
  }

  void EncodeTime(double time, lcmt_drake_signal* msg) const override {
    msg->timestamp = static_cast<int64_t>(time * 1e3);
  }

 private:
  TestVector<double> default_vector_;
  lcmt_drake_signal default_msg_;
};

// Makes a test message of size @p size.
lcmt_drake_signal MakeTestMessage(int size) {
  lcmt_drake_signal msg;
  msg.dim = size;
  msg.val.resize(msg.dim);
  msg.coord.resize(msg.dim);
  for (int i = 0; i < size; ++i) {
    msg.val[i] = i;
    msg.coord[i] = std::to_string(i);
  }
  return msg;
}

// Checks that @p msg is encoded correctly.
void CheckEncodedMessage(const lcmt_drake_signal& msg, int size, double time) {
  EXPECT_EQ(static_cast<int64_t>(time * 1e3), msg.timestamp);
  EXPECT_EQ(msg.dim, size);
  for (int i = 0; i < msg.dim; ++i) {
    EXPECT_EQ(msg.coord[i], std::to_string(i));
    EXPECT_EQ(msg.val[i], i);
  }
}

// Tests a LcmEncoderSystem from TestVector<double> to lcmt_drake_signal using
// a TestVectorTranslator.
GTEST_TEST(TranslatorTest, EncodeBasicVectorVersion) {
  const int kVecSize = 2;
  TestVector<double> vec(kVecSize);
  for (int i = 0; i < kVecSize; ++i) {
    vec.SetAtIndex(i, i);
    vec.get_mutable_names()[i] = std::to_string(i);
  }

  LcmEncoderSystem<TestVector<double>, lcmt_drake_signal> dut(
      std::make_unique<TestVectorTranslator>(kVecSize));
  auto context = dut.CreateDefaultContext();
  auto output = dut.AllocateOutput(*context);
  context->FixInputPort(0, vec.Clone());
  context->set_time(233);

  dut.CalcOutput(*context, output.get());
  const lcmt_drake_signal& msg =
      output->get_data(0)->GetValue<lcmt_drake_signal>();

  CheckEncodedMessage(msg, kVecSize, context->get_time());
}

// Tests a LcmEncoderSystem from TestData to lcmt_drake_signal using a
// TestDataTranslator.
GTEST_TEST(TranslatorTest, EncodeAbstractValVersion) {
  const int kVecSize = 2;
  TestData data(kVecSize);
  for (int i = 0; i < kVecSize; ++i) {
    data.vector[i] = i;
    data.names[i] = std::to_string(i);
  }

  LcmEncoderSystem<TestData, lcmt_drake_signal> dut(
      std::make_unique<TestDataTranslator>(kVecSize));
  auto context = dut.CreateDefaultContext();
  auto output = dut.AllocateOutput(*context);
  context->FixInputPort(0, AbstractValue::Make<TestData>(data));
  context->set_time(233);

  dut.CalcOutput(*context, output.get());
  const lcmt_drake_signal& msg =
      output->get_data(0)->GetValue<lcmt_drake_signal>();

  CheckEncodedMessage(msg, kVecSize, context->get_time());
}

// Tests a LcmDecoderSystem from lcmt_drake_signal to TestData using a
// TestDataTranslator.
GTEST_TEST(TranslatorTest, DecodeAbstractValVersion) {
  const int kVecSize = 2;

  LcmDecoderSystem<TestData, lcmt_drake_signal> dut(
      std::make_unique<TestDataTranslator>(kVecSize));

  auto context = dut.CreateDefaultContext();
  auto output = dut.AllocateOutput(*context);
  lcmt_drake_signal msg = MakeTestMessage(kVecSize);
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
  lcmt_drake_signal msg = MakeTestMessage(kVecSize);
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

///////////////////////////////////////////////////////////////////////////////
// Tests for encoding / decoding composite message.

// Tests Isometry3<double> -> robotlocomotion::pose_t.
GTEST_TEST(TranslatorTest, PoseTranslatorEncodeTest) {
  LcmEncoderSystem<Isometry3<double>, robotlocomotion::pose_t> dut(
      std::make_unique<
          drake::lcm::robotlocomotion_translators::PoseTranslator<double>>());
  auto context = dut.CreateDefaultContext();
  auto output = dut.AllocateOutput(*context);

  Isometry3<double> data = Isometry3<double>::Identity();
  data.translation() << 1, 2, 3;
  data.linear() =
      AngleAxis<double>(0.3, Vector3<double>::UnitX()).toRotationMatrix() *
      AngleAxis<double>(-1.0, Vector3<double>::UnitY()).toRotationMatrix() *
      AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()).toRotationMatrix();
  Quaternion<double> expected_quat(data.linear());

  context->FixInputPort(0, AbstractValue::Make<Isometry3<double>>(data));
  dut.CalcOutput(*context, output.get());
  const robotlocomotion::pose_t& msg =
      output->get_data(0)->GetValue<robotlocomotion::pose_t>();

  EXPECT_EQ(msg.position.x, 1);
  EXPECT_EQ(msg.position.y, 2);
  EXPECT_EQ(msg.position.z, 3);

  Quaternion<double> msg_quat(msg.orientation.w, msg.orientation.x,
                              msg.orientation.y, msg.orientation.z);
  EXPECT_NEAR(std::abs(msg_quat.dot(expected_quat)), 1., 1e-15);
}

// Tests robotlocomotion::pose_t -> Isometry3<double>.
GTEST_TEST(TranslatorTest, PoseTranslatorDecodeTest) {
  LcmDecoderSystem<Isometry3<double>, robotlocomotion::pose_t> dut(
      std::make_unique<
          drake::lcm::robotlocomotion_translators::PoseTranslator<double>>());
  auto context = dut.CreateDefaultContext();
  auto output = dut.AllocateOutput(*context);

  robotlocomotion::pose_t msg;
  msg.position.x = 3;
  msg.position.y = 2;
  msg.position.z = 1;

  Quaternion<double> expected_quat(1, 2, 3, 4);
  expected_quat.normalize();
  msg.orientation.w = expected_quat.w();
  msg.orientation.x = expected_quat.x();
  msg.orientation.y = expected_quat.y();
  msg.orientation.z = expected_quat.z();

  context->FixInputPort(0, AbstractValue::Make<robotlocomotion::pose_t>(msg));
  dut.CalcOutput(*context, output.get());
  const Isometry3<double>& data =
      output->get_data(0)->GetValue<Isometry3<double>>();
  EXPECT_EQ(data.translation()[0], 3);
  EXPECT_EQ(data.translation()[1], 2);
  EXPECT_EQ(data.translation()[2], 1);
  Quaternion<double> quat(data.linear());

  EXPECT_NEAR(std::abs(quat.dot(expected_quat)), 1., 1e-15);
}
*/

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake
