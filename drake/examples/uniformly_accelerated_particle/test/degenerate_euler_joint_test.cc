#include <memory>

#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/systems/framework/vector_base.h"

#include "drake/examples/uniformly_accelerated_particle/degenerate_euler_joint-inl.h"

namespace drake {
  namespace particles {
    namespace {

      template <typename JointType>
      class DegenerateEulerJointTest : public ::testing::Test {
      protected:
	void SetUp() override {
	  typename JointType::TranslatingMatrix t =
	    JointType::TranslatingMatrix::Zero();
	  t(0, 0) = 1.0;  // translate first generalized coord only
	  this->dut_ = std::make_unique<JointType>(t);
	  this->context_ = this->dut_->CreateDefaultContext();
	  this->output_ = this->dut_->AllocateOutput(*this->context_);
	}
	
	std::unique_ptr<systems::System<typename JointType::ScalarType>> dut_;
	std::unique_ptr<systems::Context<typename JointType::ScalarType>> context_;
	std::unique_ptr<systems::SystemOutput<typename JointType::ScalarType>> output_;
      };

      TYPED_TEST_CASE_P(DegenerateEulerJointTest);
      
      TYPED_TEST_P(DegenerateEulerJointTest, Output) {
	// set input
	const systems::InputPortDescriptor<typename TypeParam::ScalarType>& input_descriptor =
	  this->dut_->get_input_port(0);
	auto input = std::make_unique<systems::BasicVector<typename TypeParam::ScalarType>>(input_descriptor.size());	
	input->SetZero();
	// set first generalized position coordinate
	input->SetAtIndex(0, static_cast<typename TypeParam::ScalarType>(1.0));
	// set first generalized velocity coordinate
	input->SetAtIndex(input->size()/2, static_cast<typename TypeParam::ScalarType>(5.0));
	this->context_->FixInputPort(0, std::move(input));
	// compute outputs
	this->dut_->CalcOutput(*this->context_, this->output_.get());
	const systems::BasicVector<typename TypeParam::ScalarType>* output =
	  this->output_->get_vector_data(0);
	// check results
	EXPECT_EQ(1.0, output->GetAtIndex(0));
	EXPECT_EQ(0.0, output->GetAtIndex(2));
	EXPECT_EQ(5.0, output->GetAtIndex(6));
	EXPECT_EQ(0.0, output->GetAtIndex(8));
      }

      REGISTER_TYPED_TEST_CASE_P(DegenerateEulerJointTest, Output);

      using DegenerateEulerJointd1DOF = DegenerateEulerJoint<double, 1>;
      INSTANTIATE_TYPED_TEST_CASE_P(WithDoubles1DOF,
				    DegenerateEulerJointTest,
				    DegenerateEulerJointd1DOF);
      using DegenerateEulerJointd5DOF = DegenerateEulerJoint<double, 1>;
      INSTANTIATE_TYPED_TEST_CASE_P(WithDoubles5DOF,
				    DegenerateEulerJointTest,
				    DegenerateEulerJointd5DOF);
      
    }  // namespace
  }  // namespace particles
}  // namespace drake
