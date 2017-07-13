#include "drake/systems/framework/basic_vector.h"
#include "drake/dev_perception/neural_network.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <memory>

#include <iostream>


namespace drake {
	using std::unique_ptr;
	using systems::SystemOutput;

	using std::cout;
	using std::endl;
namespace perception {
namespace {

	std::vector<LayerType> generateFullyConnectedLayers( int numLayers ) {
		std::vector<LayerType> layers;
		LayerType fullyConnected = LayerType::FullyConnected;
		for ( int i = 0; i < numLayers; i++ ) {
			layers.push_back( fullyConnected );
		}
		return layers;
	}

	std::vector<NonlinearityType> generateReluNonlinearities( int numLayers ) {
		std::vector<NonlinearityType> nonlinearities;
		NonlinearityType relu = NonlinearityType::Relu;
		for ( int i = 0; i < numLayers; i++ ) {
			nonlinearities.push_back( relu );
		}
		return nonlinearities;
	}

  // Test that the matrices are being encoded and decoded correctly from
  // BasicVectors
	GTEST_TEST( NeuralNetworkTest, MatrixCoderDecoder ) {
		MatrixXd W1 = MatrixXd::Random(3, 3);
		MatrixXd W2 = MatrixXd::Random(7, 3);
		std::vector<MatrixXd> W;
		W.push_back( W1 );
		W.push_back( W2 );

		VectorXd B1 = VectorXd::Random(3);
		VectorXd B2 = VectorXd::Random(7);
		std::vector<VectorXd> B;
		B.push_back( B1 );
		B.push_back( B2 );

		NeuralNetwork<double> dut(W, B, 
															generateFullyConnectedLayers(2),
															generateReluNonlinearities(2) );

		unique_ptr<BasicVector<double>> encoded1 = dut.encode( W1 );
		unique_ptr<BasicVector<double>> encoded2 = dut.encode( W2 );

		unique_ptr<MatrixXd> W1recovered = dut.decode( *encoded1 );
		unique_ptr<MatrixXd> W2recovered = dut.decode( *encoded2 );

		EXPECT_EQ( W1, *W1recovered );
		EXPECT_EQ( W2, *W2recovered );
	}

	// Test that the NN is correctly loading and extracting its parameters from
	// the Context
	GTEST_TEST( NeuralNetworkTest, ParameterStorage ) {
		MatrixXd W1 = MatrixXd::Random(3, 3);
		MatrixXd W2 = MatrixXd::Random(7, 3);
		MatrixXd W3 = MatrixXd::Random(2, 7);
		std::vector<MatrixXd> W;
		W.push_back( W1 );
		W.push_back( W2 );
		W.push_back( W3 );

		VectorXd B1 = VectorXd::Random(3);
		VectorXd B2 = VectorXd::Random(7);
		VectorXd B3 = VectorXd::Random(2);
		std::vector<VectorXd> B;
		B.push_back( B1 );
		B.push_back( B2 );
		B.push_back( B3 );

		NeuralNetwork<double> dut(W, B, 
															generateFullyConnectedLayers(3),
															generateReluNonlinearities(3) );

		unique_ptr<Context<double>> context = dut.CreateDefaultContext();
		unique_ptr<SystemOutput<double>> output = dut.AllocateOutput(*context);

		EXPECT_EQ( W1, *(dut.getWeightMatrix( 0, *context )) );
		EXPECT_EQ( W2, *(dut.getWeightMatrix( 1, *context )) );
		EXPECT_EQ( W3, *(dut.getWeightMatrix( 2, *context )) );

		//cout << "Expected: " << endl << B1 << endl;
		//cout << "Recovered: " << endl << *(dut.getBiasVector(0, *context));
		EXPECT_EQ( B1, *(dut.getBiasVector( 0, *context )) );
		EXPECT_EQ( B2, *(dut.getBiasVector( 1, *context )) );
		EXPECT_EQ( B3, *(dut.getBiasVector( 2, *context )) );
	}

	// Test that the NN can figure out what "shape" it should be:
	// - num inputs,
	// - num outputs,
	// - num layers
	GTEST_TEST( NeuralNetworkTest, ShapeParameters ) {
		MatrixXd W1 = MatrixXd::Random(10, 19);
		MatrixXd W2 = MatrixXd::Random(7, 10);
		MatrixXd W3 = MatrixXd::Random(2, 7);
		std::vector<MatrixXd> W;
		W.push_back( W1 );
		W.push_back( W2 );
		W.push_back( W3 );

		VectorXd B1 = VectorXd::Random(10);
		VectorXd B2 = VectorXd::Random(7);
		VectorXd B3 = VectorXd::Random(2);
		std::vector<VectorXd> B;
		B.push_back( B1 );
		B.push_back( B2 );
		B.push_back( B3 );

		NeuralNetwork<double> dut(W, B, 
															generateFullyConnectedLayers(3),
															generateReluNonlinearities(3) );

		EXPECT_EQ( dut.getNumLayers(), 3 );
		EXPECT_EQ( dut.getNumInputs(), 19 );
		EXPECT_EQ( dut.getNumOutputs(), 2 );

	}

	// TODO(nikos-tri) Make the NN take different kinds of layers
	
} // namespace
} // namespace perception
} // namespace drake
