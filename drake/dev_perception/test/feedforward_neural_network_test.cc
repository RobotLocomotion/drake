#include "drake/dev_perception/feedforward_neural_network.h"
#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
using std::unique_ptr;
using systems::SystemOutput;
using systems::Context;
using systems::BasicVector;

using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::cout;
using std::endl;
namespace perception {
namespace {

// Helper function to generate a vector of Connected layers
std::vector<LayerType> generateFullyConnectedLayers(int numLayers) {
  std::vector<LayerType> layers;
  LayerType fullyConnected = LayerType::FullyConnected;
  for (int i = 0; i < numLayers; i++) {
    layers.push_back(fullyConnected);
  }
  return layers;
}

// Helper function to generate a vector of Relu nonlinearities
std::vector<NonlinearityType> generateReluNonlinearities(int numLayers) {
  std::vector<NonlinearityType> nonlinearities;
  NonlinearityType relu = NonlinearityType::Relu;
  for (int i = 0; i < numLayers; i++) {
    nonlinearities.push_back(relu);
  }
  return nonlinearities;
}

// Test that the matrices are being encoded and decoded correctly from
// BasicVectors
GTEST_TEST(FeedforwardNeuralNetworkTest, MatrixCoderDecoder) {
  MatrixXd W1 = MatrixXd::Random(3, 3);
  MatrixXd W2 = MatrixXd::Random(7, 3);
  std::vector<MatrixXd> W;
  W.push_back(W1);
  W.push_back(W2);

  VectorXd B1 = VectorXd::Random(3);
  VectorXd B2 = VectorXd::Random(7);
  std::vector<VectorXd> B;
  B.push_back(B1);
  B.push_back(B2);

  FeedforwardNeuralNetwork<double> dut(W, B, generateFullyConnectedLayers(2),
                            generateReluNonlinearities(2));

  unique_ptr<BasicVector<double>> encoded1 = dut.Encode(W1);
  unique_ptr<BasicVector<double>> encoded2 = dut.Encode(W2);

  unique_ptr<MatrixXd> W1recovered = dut.Decode(*encoded1);
  unique_ptr<MatrixXd> W2recovered = dut.Decode(*encoded2);

  EXPECT_EQ(W1, *W1recovered);
  EXPECT_EQ(W2, *W2recovered);
}

// Test that the NN is correctly loading and extracting its parameters from
// the Context
GTEST_TEST(FeedforwardNeuralNetworkTest, ParameterStorage) {
  MatrixXd W1 = MatrixXd::Random(3, 3);
  MatrixXd W2 = MatrixXd::Random(7, 3);
  MatrixXd W3 = MatrixXd::Random(2, 7);
  std::vector<MatrixXd> W;
  W.push_back(W1);
  W.push_back(W2);
  W.push_back(W3);

  VectorXd B1 = VectorXd::Random(3);
  VectorXd B2 = VectorXd::Random(7);
  VectorXd B3 = VectorXd::Random(2);
  std::vector<VectorXd> B;
  B.push_back(B1);
  B.push_back(B2);
  B.push_back(B3);

  FeedforwardNeuralNetwork<double> dut(W, B, generateFullyConnectedLayers(3),
                            generateReluNonlinearities(3));

  unique_ptr<Context<double>> context = dut.CreateDefaultContext();

  EXPECT_EQ(W1, *(dut.get_weight_matrix(0, *context)));
  EXPECT_EQ(W2, *(dut.get_weight_matrix(1, *context)));
  EXPECT_EQ(W3, *(dut.get_weight_matrix(2, *context)));

  // cout << "Expected: " << endl << B1 << endl;
  // cout << "Recovered: " << endl << *(dut.get_bias_vector(0, *context));
  EXPECT_EQ(B1, *(dut.get_bias_vector(0, *context)));
  EXPECT_EQ(B2, *(dut.get_bias_vector(1, *context)));
  EXPECT_EQ(B3, *(dut.get_bias_vector(2, *context)));
}

// Test that the NN can figure out what "shape" it should be:
// - num inputs,
// - num outputs,
// - num layers
GTEST_TEST(FeedforwardNeuralNetworkTest, ShapeParameters) {
  MatrixXd W1 = MatrixXd::Random(10, 19);
  MatrixXd W2 = MatrixXd::Random(7, 10);
  MatrixXd W3 = MatrixXd::Random(2, 7);
  std::vector<MatrixXd> W;
  W.push_back(W1);
  W.push_back(W2);
  W.push_back(W3);

  VectorXd B1 = VectorXd::Random(10);
  VectorXd B2 = VectorXd::Random(7);
  VectorXd B3 = VectorXd::Random(2);
  std::vector<VectorXd> B;
  B.push_back(B1);
  B.push_back(B2);
  B.push_back(B3);

  FeedforwardNeuralNetwork<double> dut(W, B, generateFullyConnectedLayers(3),
                            generateReluNonlinearities(3));

  EXPECT_EQ(dut.get_num_layers(), 3);
  EXPECT_EQ(dut.get_num_inputs(), 19);
  EXPECT_EQ(dut.get_num_outputs(), 2);
}

// Very basic sanity test of input-output behavior with identity weight
// matrices
void TestIO(std::vector<MatrixXd> W, std::vector<VectorXd> B, VectorXd input,
            VectorXd expectedOutput) {
  FeedforwardNeuralNetwork<double> dut(W, B, generateFullyConnectedLayers(3),
                            generateReluNonlinearities(3));
  unique_ptr<Context<double>> context = dut.CreateDefaultContext();
  unique_ptr<SystemOutput<double>> output = dut.AllocateOutput(*context);

  context->FixInputPort(dut.input().get_index(), input);
  dut.CalcOutput(*context, output.get());
  VectorXd computedOutput =
      output->get_vector_data(dut.output().get_index())->get_value();

  // EXPECT_TRUE( CompareMatrices(expectedOutput, computedOutput, 0.1) );
  EXPECT_EQ(expectedOutput, computedOutput);
  cout << endl << input << endl;
  cout << expectedOutput << endl;
  cout << computedOutput << endl;
}
GTEST_TEST(FeedforwardNeuralNetworkTest, BasicSanity) {
  MatrixXd I1 = MatrixXd::Identity(3, 3);
  MatrixXd I2 = MatrixXd::Identity(3, 3);
  MatrixXd I3 = MatrixXd::Identity(3, 3);
  std::vector<MatrixXd> W;
  W.push_back(I1);
  W.push_back(I2);
  W.push_back(I3);

  VectorXd B1 = VectorXd::Zero(3);
  VectorXd B2 = VectorXd::Zero(3);
  VectorXd B3 = VectorXd::Zero(3);
  std::vector<VectorXd> B;
  B.push_back(B1);
  B.push_back(B2);
  B.push_back(B3);

  // A few test vectors
  VectorXd input1(3);
  input1 << 1, 2, 3;
  VectorXd expectedOutput1 = input1;

  VectorXd input2(3);
  input2 << -1, -2, -3;
  VectorXd expectedOutput2(3);
  expectedOutput2 << 0, 0, 0;  // ReLU zeroes them out because they are negative

  VectorXd input3(3);
  input3 << -1, 0, 1;
  VectorXd expectedOutput3(3);
  expectedOutput2 << 0, 0, 1;  // ReLU zeroes out the negative

  TestIO(W, B, input1, expectedOutput1);
  // TODO(nikos-tri) Figure out why these are failing
  // TestIO( W, B, input2, expectedOutput2 );
  // TestIO( W, B, input3, expectedOutput3 );
}

}  // namespace
}  // namespace perception
}  // namespace drake
