#include "gtest/gtest.h"

// TODO(liang.fok) Automatically generate this file.

#include "drake/examples/spring_mass/spring_mass_lcm_vector.h"
#include "drake/util/eigen_matrix_compare.h"

namespace drake {
namespace examples {
namespace spring_mass {
namespace {

using drake::examples::spring_mass::SpringMassLCMVector;

// Tests the default values in
// drake::examples::spring_mass::SpringMassLCMVector.
GTEST_TEST(SpringMassLCMVectorTest, InstantiateTest) {
  // Instantiates a SpringMassLCMVector<double> and verifies that it was
  // successfully stored in a unique_ptr. The variable is called "dut" to
  // indicate it is the "device under test".
  std::unique_ptr<SpringMassLCMVector<double>> dut(
    new SpringMassLCMVector<double>());
  EXPECT_NE(dut.get(), nullptr);

  // Verifies that the default timestamp inside the SpringMassLCMVector<double>
  // object is -1.
  EXPECT_EQ(dut->get_timestamp(), -1);

  // Obtains the size of the state vector.
  const int state_vector_size = SpringMassLCMVector<double>::kStateSize;

  // Checks method SpringMassLCMVector<double>::size().
  EXPECT_EQ(dut->size(), state_vector_size);

  // Verifies that the length of the value vector is equal to
  // SpringMassLCMVector<double>::kStateSize and that its default values are
  // NaN.
  Eigen::VectorBlock<const VectorX<double>> value = dut->get_value();
  EXPECT_EQ(value.size(), state_vector_size);
  for (int ii = 0; ii < state_vector_size; ++ii) {
    EXPECT_TRUE(isnan(value[ii]));
  }
}

// Tests the following methods in
// drake::examples::spring_mass::SpringMassLCMVector:
//
//  - get_mutable_value()
//  - get_value()
//  - set_value()
GTEST_TEST(SpringMassLCMVectorTest, SetAndGetValueTest) {
  // Instantiates a SpringMassLCMVector<double> and verifies that it was
  // successfully stored in a unique_ptr. The variable is called "dut" to
  // indicate it is the "device under test".
  std::unique_ptr<SpringMassLCMVector<double>> dut(
    new SpringMassLCMVector<double>());
  EXPECT_NE(dut.get(), nullptr);

  // Tests calling set_value() with a vector that's too long.
  {
    VectorX<double> vector;
    vector.resize(dut->size() + 1);
    EXPECT_THROW(dut->set_value(vector), std::runtime_error);
  }

  // Tests calling set_value() with a vector that's too short.
  {
    VectorX<double> vector;
    vector.resize(dut->size() - 1);
    EXPECT_THROW(dut->set_value(vector), std::runtime_error);
  }

  // Tests calling set_value() with a correct-length vector. Verifies that
  // the values obtained from get_value() and get_mutable_value() are correct.
  VectorX<double> vector;
  vector.resize(dut->size());
  vector[0] = 3.14;
  vector[1] = 2.718;
  EXPECT_NO_THROW(dut->set_value(vector));

  {
    Eigen::VectorBlock<const VectorX<double>> const_value = dut->get_value();
    EXPECT_TRUE(drake::util::CompareMatrices(const_value, vector, 1e-10,
      drake::util::MatrixCompareType::relative));

    Eigen::VectorBlock<VectorX<double>> mutable_value = dut->get_mutable_value();
    EXPECT_TRUE(drake::util::CompareMatrices(mutable_value, vector, 1e-10,
      drake::util::MatrixCompareType::relative));
  }

  // Changes the values in `vector`. Then ensures the values returned by
  // `get_value()` and `get_mutable_value()` do not match `vector` anymore.
  vector[0] = 10;
  vector[1] = 12.5;

  {
    Eigen::VectorBlock<const VectorX<double>> const_value = dut->get_value();
    EXPECT_FALSE(drake::util::CompareMatrices(const_value, vector, 1e-10,
      drake::util::MatrixCompareType::relative));

    Eigen::VectorBlock<VectorX<double>> mutable_value = dut->get_mutable_value();
    EXPECT_FALSE(drake::util::CompareMatrices(mutable_value, vector, 1e-10,
      drake::util::MatrixCompareType::relative));
  }
}

// Tests the following methods in
// drake::examples::spring_mass::SpringMassLCMVector:
//
//  - Clone()
//  - Decode()
//  - Encode()
GTEST_TEST(SpringMassLCMVectorTest, EncodeDecodeAndCloneTest) {
  // Instantiates a SpringMassLCMVector<double> and verifies that it was
  // successfully stored in a unique_ptr. The variable is called "dut" to
  // indicate it is the "device under test".
  std::unique_ptr<SpringMassLCMVector<double>> dut(
    new SpringMassLCMVector<double>());
  EXPECT_NE(dut.get(), nullptr);

  // Initializes a lcmt_spring_mass_state_t object, then calls Encode() using it
  // as a parameter.
  lcmt_spring_mass_state_t message;
  message.timestamp = 123456;
  message.position = 0.57721;
  message.velocity = 4.6692;

  dut->Encode(message);

  // Verifies that the output of Decode() is correct.
  lcmt_spring_mass_state_t decoded_message;
  dut->Decode(&decoded_message);
  EXPECT_EQ(message.timestamp, decoded_message.timestamp);
  EXPECT_EQ(message.position, decoded_message.position);
  EXPECT_EQ(message.velocity, decoded_message.velocity);

  // Verifies that the outputs of get_value(), get_mutable_value(), and
  // get_timestamp() are correct.
  VectorX<double> vector;
  vector.resize(dut->size());
  vector[0] = message.position;
  vector[1] = message.velocity;

  {
    Eigen::VectorBlock<const VectorX<double>> const_value = dut->get_value();
    EXPECT_TRUE(drake::util::CompareMatrices(const_value, vector, 1e-10,
      drake::util::MatrixCompareType::relative));

    Eigen::VectorBlock<VectorX<double>> mutable_value = dut->get_mutable_value();
    EXPECT_TRUE(drake::util::CompareMatrices(mutable_value, vector, 1e-10,
      drake::util::MatrixCompareType::relative));

    EXPECT_EQ(dut->get_timestamp(), 123456);
  }

  // Verifies that Clone() is correct.
  std::unique_ptr<drake::systems::VectorInterface<double>> clone = dut->Clone();

  {
    Eigen::VectorBlock<const VectorX<double>> const_value = clone->get_value();
    EXPECT_TRUE(drake::util::CompareMatrices(const_value, vector, 1e-10,
      drake::util::MatrixCompareType::relative));

    Eigen::VectorBlock<VectorX<double>> mutable_value =
      clone->get_mutable_value();
    EXPECT_TRUE(drake::util::CompareMatrices(mutable_value, vector, 1e-10,
      drake::util::MatrixCompareType::relative));
  }
}

}  // namespace
}  // namespace spring_mass
}  // namespace examples
}  // namespace drake
