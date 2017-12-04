#include "drake/perception/dev/feedforward_neural_network.h"

#include <stdlib.h>

#include <iostream>
#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
using std::abs;
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

/********************************
 * Helper function declarations
 * (definitions at end of file)
 ********************************/
// Helper functions to generate a new matrix/vector with specified dimensions,
// initialize the entries. We don't care about what the entries are; this is
// meant to be a way to get some variety of entries without using Matrix::Random
MatrixXd NewMatrix(int rows, int columns);
VectorXd NewVector(int rows);
// Helper function to generate a vector of Connected layers
std::vector<LayerType> GenerateFullyConnectedLayers(int num_layers);
// Helper function to generate a vector of Relu nonlinearities
std::vector<NonlinearityType> GenerateReluNonlinearities(int num_layers);
// Helper function to check IO behavior of a double FeedforwardNeuralNetwork
template <typename T>
void TestIO(std::vector<MatrixX<T>> W, std::vector<VectorX<T>> B,
            VectorX<T> input, VectorX<T> expectedOutput);
// Helper function to compare a MatrixX<double> and a MatrixX<AutoDiffXd>
bool CompareMats(const MatrixX<double>& Mdouble,
                 const MatrixX<AutoDiffXd>& Mautodiff, double tolerance = 0);

/********************************
 * Tests
 ********************************/

// Test that the matrices are being encoded and decoded correctly from
// BasicVectors
GTEST_TEST(FeedforwardNeuralNetworkTest, MatrixCoderDecoder) {
  MatrixXd W1 = NewMatrix(3, 3);
  MatrixXd W2 = NewMatrix(7, 3);
  std::vector<MatrixXd> W;
  W.push_back(W1);
  W.push_back(W2);

  VectorXd B1 = NewVector(3);
  VectorXd B2 = NewVector(7);
  std::vector<VectorXd> B;
  B.push_back(B1);
  B.push_back(B2);

  FeedforwardNeuralNetwork<double> dut(W, B, GenerateFullyConnectedLayers(2),
                                       GenerateReluNonlinearities(2));

  unique_ptr<BasicVector<double>> encoded1 = dut.EncodeWeightsToBasicVector(W1);
  unique_ptr<BasicVector<double>> encoded2 = dut.EncodeWeightsToBasicVector(W2);

  unique_ptr<MatrixXd> W1recovered =
      dut.DecodeWeightsFromBasicVector(3, 3, *encoded1);
  unique_ptr<MatrixXd> W2recovered =
      dut.DecodeWeightsFromBasicVector(7, 3, *encoded2);

  EXPECT_EQ(W1, *W1recovered);
  EXPECT_EQ(W2, *W2recovered);
}

// Test that the NN is correctly loading and extracting its parameters from
// the Context
GTEST_TEST(FeedforwardNeuralNetworkTest, ParameterStorage) {
  MatrixXd W1 = NewMatrix(3, 3);
  MatrixXd W2 = NewMatrix(7, 3);
  MatrixXd W3 = NewMatrix(2, 7);
  std::vector<MatrixXd> W;
  W.push_back(W1);
  W.push_back(W2);
  W.push_back(W3);

  VectorXd B1 = NewVector(3);
  VectorXd B2 = NewVector(7);
  VectorXd B3 = NewVector(2);
  std::vector<VectorXd> B;
  B.push_back(B1);
  B.push_back(B2);
  B.push_back(B3);

  FeedforwardNeuralNetwork<double> dut(W, B, GenerateFullyConnectedLayers(3),
                                       GenerateReluNonlinearities(3));

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
  MatrixXd W1 = NewMatrix(10, 19);
  MatrixXd W2 = NewMatrix(7, 10);
  MatrixXd W3 = NewMatrix(2, 7);
  std::vector<MatrixXd> W;
  W.push_back(W1);
  W.push_back(W2);
  W.push_back(W3);

  VectorXd B1 = NewVector(10);
  VectorXd B2 = NewVector(7);
  VectorXd B3 = NewVector(2);
  std::vector<VectorXd> B;
  B.push_back(B1);
  B.push_back(B2);
  B.push_back(B3);

  FeedforwardNeuralNetwork<double> dut(W, B, GenerateFullyConnectedLayers(3),
                                       GenerateReluNonlinearities(3));

  EXPECT_EQ(dut.get_num_layers(), 3);
  EXPECT_EQ(dut.get_num_inputs(), 19);
  EXPECT_EQ(dut.get_num_outputs(), 2);
}

GTEST_TEST(FeedforwardNeuralNetworkTest, ToAutoDiffTest) {
  MatrixXd W1 = NewMatrix(10, 19);
  MatrixXd W2 = NewMatrix(7, 10);
  MatrixXd W3 = NewMatrix(2, 7);
  std::vector<MatrixXd> W;
  W.push_back(W1);
  W.push_back(W2);
  W.push_back(W3);

  VectorXd B1 = NewVector(10);
  VectorXd B2 = NewVector(7);
  VectorXd B3 = NewVector(2);
  std::vector<VectorXd> B;
  B.push_back(B1);
  B.push_back(B2);
  B.push_back(B3);

  FeedforwardNeuralNetwork<double> dut(W, B, GenerateFullyConnectedLayers(3),
                                       GenerateReluNonlinearities(3));
  unique_ptr<Context<double>> context = dut.CreateDefaultContext();

  unique_ptr<FeedforwardNeuralNetwork<AutoDiffXd>> ad_ffnn = dut.ToAutoDiffXd();
  unique_ptr<Context<AutoDiffXd>> ad_context = ad_ffnn->CreateDefaultContext();
  ad_context->SetTimeStateAndParametersFrom(*context);

  unique_ptr<MatrixX<AutoDiffXd>> W1recovered =
      ad_ffnn->get_weight_matrix(0, *ad_context);
  unique_ptr<MatrixX<AutoDiffXd>> W2recovered =
      ad_ffnn->get_weight_matrix(1, *ad_context);
  unique_ptr<MatrixX<AutoDiffXd>> W3recovered =
      ad_ffnn->get_weight_matrix(2, *ad_context);

  EXPECT_TRUE(CompareMats(W1, *W1recovered));
  EXPECT_TRUE(CompareMats(W2, *W2recovered));
  EXPECT_TRUE(CompareMats(W3, *W3recovered));
}

// Very basic sanity test of input-output behavior with identity weight
// matrices
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
  expectedOutput3 << 0, 0, 1;  // ReLU zeroes out the negative

  TestIO(W, B, input1, expectedOutput1);
  TestIO(W, B, input2, expectedOutput2);
  TestIO(W, B, input3, expectedOutput3);
}

/********************************
 * Helper function definitions
 ********************************/
// Helper functions to generate a new matrix/vector with specified dimensions,
// initialize the entries. We don't care about what the entries are; this is
// meant to be a way to get some variety of entries without using Matrix::Random
MatrixXd NewMatrix(const int rows, const int columns) {
  MatrixXd matrix(rows, columns);

  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < columns; j++) {
      matrix(i, j) = i + j;  // Initialize each entry to whatever number
    }
  }

  return matrix;
}
VectorXd NewVector(const int rows) {
  VectorXd vector(rows);

  for (int i = 0; i < rows; i++) {
    vector(i) = i * i;
  }

  return vector;
}

// Helper function to generate a vector of Connected layers
std::vector<LayerType> GenerateFullyConnectedLayers(int num_layers) {
  std::vector<LayerType> layers;
  LayerType fullyConnected = LayerType::FullyConnected;
  for (int i = 0; i < num_layers; i++) {
    layers.push_back(fullyConnected);
  }
  return layers;
}

// Helper function to generate a vector of Relu nonlinearities
std::vector<NonlinearityType> GenerateReluNonlinearities(int num_layers) {
  std::vector<NonlinearityType> nonlinearities;
  NonlinearityType relu = NonlinearityType::Relu;
  for (int i = 0; i < num_layers; i++) {
    nonlinearities.push_back(relu);
  }
  return nonlinearities;
}

// Helper function to compare a MatrixX<double> and a MatrixX<AutoDiffXd>
bool CompareMats(const MatrixX<double>& Mdouble,
                 const MatrixX<AutoDiffXd>& Mautodiff,
                 double tolerance /* default is 0, see declaration*/) {
  if ((Mdouble.rows() != Mautodiff.rows()) ||
      (Mdouble.cols() != Mautodiff.cols())) {
    return false;
  }

  for (int i = 0; i < Mdouble.rows(); i++) {
    for (int j = 0; j < Mdouble.rows(); j++) {
      double thisAdValue = (Mautodiff(i, j)).value();

      if (abs(thisAdValue - Mdouble(i, j)) > tolerance) {
        return false;
      }
    }
  }
  return true;
}

// Check that a FeedforwardNeuralNetwork<T> gives the expected result
template <typename T>
void TestIO(std::vector<MatrixX<T>> W, std::vector<VectorX<T>> B,
            VectorX<T> input, VectorX<T> expectedOutput) {
  FeedforwardNeuralNetwork<double> dut(W, B, GenerateFullyConnectedLayers(3),
                                       GenerateReluNonlinearities(3));
  unique_ptr<Context<double>> context = dut.CreateDefaultContext();
  unique_ptr<SystemOutput<double>> output = dut.AllocateOutput(*context);

  context->FixInputPort(dut.input().get_index(), input);
  dut.CalcOutput(*context, output.get());
  VectorXd computedOutput =
      output->get_vector_data(dut.output().get_index())->get_value();

  EXPECT_EQ(expectedOutput, computedOutput);
}

}  // namespace
}  // namespace perception
}  // namespace drake
