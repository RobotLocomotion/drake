#include <memory>

#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/systems/framework/vector_base.h"

#include "drake/examples/uniformly_accelerated_particle/particle.h"

namespace drake {
  namespace particles {
    namespace {

      template <typename ParticleType>
      class ParticleTest : public ::testing::Test {
      protected:
	void SetUp() override {
	  this->dut_ = std::make_unique<ParticleType>();
	  this->context_ = this->dut_->CreateDefaultContext();
	  this->output_ = this->dut_->AllocateOutput(*context_);
	  this->derivatives_ = this->dut_->AllocateTimeDerivatives();	  
	}

	std::unique_ptr<systems::System<typename ParticleType::ScalarType>> dut_;
	std::unique_ptr<systems::Context<typename ParticleType::ScalarType>> context_;
	std::unique_ptr<systems::SystemOutput<typename ParticleType::ScalarType>> output_;
	std::unique_ptr<systems::ContinuousState<typename ParticleType::ScalarType>> derivatives_;
      };    

      TYPED_TEST_CASE_P(ParticleTest);
      
      TYPED_TEST_P(ParticleTest, Output) {
	// initialize state
	systems::VectorBase<typename TypeParam::ScalarType>* cstate = 
	  this->context_->get_mutable_continuous_state_vector();
	cstate->SetAtIndex(0, static_cast<typename TypeParam::ScalarType>(10.0)); // x = 10 m
	cstate->SetAtIndex(1, static_cast<typename TypeParam::ScalarType>(1.0));  // v = 1 m/s
	// compute outputs
	this->dut_->CalcOutput(*this->context_, this->output_.get());
	systems::BasicVector<typename TypeParam::ScalarType>* output =
	  this->output_->GetMutableVectorData(0);
	// check results
	EXPECT_EQ(static_cast<typename TypeParam::ScalarType>(10.0), output->GetAtIndex(0));
	EXPECT_EQ(static_cast<typename TypeParam::ScalarType>(1.0), output->GetAtIndex(1));
      }
      
      TYPED_TEST_P(ParticleTest, Derivatives) {
	// set input
	const systems::InputPortDescriptor<typename TypeParam::ScalarType>& input_descriptor =
	  this->dut_->get_input_port(0);
	auto input = std::make_unique<systems::BasicVector<typename TypeParam::ScalarType>>(input_descriptor.size());	
	input->SetZero();
	input->SetAtIndex(0, static_cast<typename TypeParam::ScalarType>(1.0)); // a = 1 m/s^2
	this->context_->FixInputPort(0, std::move(input));
	// set state
	systems::VectorBase<typename TypeParam::ScalarType>* cstate = 
	  this->context_->get_mutable_continuous_state_vector();
	cstate->SetAtIndex(0, static_cast<typename TypeParam::ScalarType>(0.0)); // x = 0 m
	cstate->SetAtIndex(1, static_cast<typename TypeParam::ScalarType>(2.0));  // v = 2 m/s
	// compute derivatives
	this->dut_->CalcTimeDerivatives(*this->context_, this->derivatives_.get());
	const systems::VectorBase<typename TypeParam::ScalarType>& deriv =
	  this->derivatives_->get_vector();
	// check results
	EXPECT_EQ(static_cast<typename TypeParam::ScalarType>(2.0), deriv.GetAtIndex(0));
	EXPECT_EQ(static_cast<typename TypeParam::ScalarType>(1.0), deriv.GetAtIndex(1));
      }

      REGISTER_TYPED_TEST_CASE_P(ParticleTest, Output, Derivatives);
      
      INSTANTIATE_TYPED_TEST_CASE_P(WithDoubles, ParticleTest, Particle<double>);
      
    }  // namespace
  }  // namespace particles
}  // namespace drake
