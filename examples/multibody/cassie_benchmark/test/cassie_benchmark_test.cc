/*
Adapted for Drake from a Cassie benchmark by Michael Posa:
Copyright (c) 2020, Dynamic Autonomy and Intelligent Robotics Lab
BSD 3-clause license: https://github.com/DAIRLab/dairlib/blob/master/LICENSE
See https://github.com/DAIRLab/dairlib/issues/181 for the original benchmark.
*/

#include <chrono>
#include <iostream>
#include <memory>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::multibody::MultibodyPlant;

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace {

typedef std::chrono::steady_clock my_clock;

int do_main() {
  const int num_reps = 100000;
  const int num_autodiff_reps = 1000;

  //
  // Build and test multibody plant
  //
  systems::DiagramBuilder<double> builder;
  MultibodyPlant<double>& multibody_plant =
      *builder.AddSystem<MultibodyPlant>(0);

  multibody::Parser parser(&multibody_plant);
  parser.AddModelFromFile(FindResourceOrThrow(
      "drake/examples/multibody/cassie_benchmark/cassie_v2.urdf"));

  multibody_plant.WeldFrames(multibody_plant.world_frame(),
                             multibody_plant.GetFrameByName("pelvis"));
  multibody_plant.Finalize();

  int nq = multibody_plant.num_positions();
  int nv = multibody_plant.num_velocities();
  int nu = multibody_plant.num_actuators();

  VectorXd x = VectorXd::Zero(nq + nv);
  VectorXd u = VectorXd::Zero(nu);

  auto multibody_context = multibody_plant.CreateDefaultContext();

  auto start = my_clock::now();
  MatrixXd M(nv, nv);
  {
    // drake::test::LimitMalloc guard({.max_num_allocations = 0});
    for (int i = 0; i < num_reps; i++) {
      x(0) = i;
      multibody_plant.SetPositionsAndVelocities(multibody_context.get(), x);
      multibody_plant.CalcMassMatrix(*multibody_context, &M);
    }
  }
  auto stop = my_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_reps)
            << "x mass matrix took " << duration.count()
            << " ms. " << 1000 * duration.count() / num_reps
            << " us per." << std::endl;

  //
  // Build and test multibody plant w/autodiff
  //
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> multibody_plant_autodiff =
      systems::System<double>::ToAutoDiffXd(multibody_plant);

  auto multibody_context_autodiff =
      multibody_plant_autodiff->CreateDefaultContext();

  MatrixX<AutoDiffXd> M_autodiff(nv, nv);
  {
    // drake::test::LimitMalloc guard({.max_num_allocations = 0});
    start = my_clock::now();
    for (int i = 0; i < num_autodiff_reps; i++) {
      x(0) = i;
      multibody_plant_autodiff->SetPositionsAndVelocities(
          multibody_context_autodiff.get(), math::initializeAutoDiff(x));
      multibody_plant_autodiff->CalcMassMatrix(*multibody_context_autodiff,
                                               &M_autodiff);
    }
  }
  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_autodiff_reps)
            << "x autodiff mass matrix took " << duration.count()
            << " ms. " << 1000 * duration.count() / num_autodiff_reps
            << " us per." << std::endl;

  // multibody inverse dynamics
  VectorXd desired_vdot;
  start = my_clock::now();
  multibody::MultibodyForces<double> external_forces(multibody_plant);

  for (int i = 0; i < num_reps; i++) {
    // drake::test::LimitMalloc guard({.max_num_allocations = 3});
    x = VectorXd::Constant(nq + nv, i);
    desired_vdot = VectorXd::Constant(nv, i);
    multibody_plant.SetPositionsAndVelocities(multibody_context.get(), x);
    multibody_plant.CalcInverseDynamics(*multibody_context, desired_vdot,
                                        external_forces);
  }
  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_reps)
            << "x inverse dynamics took " << duration.count()
            << " ms. " << 1000 * duration.count() / num_reps
            << " us per." << std::endl;

  start = my_clock::now();
  multibody::MultibodyForces<AutoDiffXd> external_forces_autodiff(
      *multibody_plant_autodiff);

  for (int i = 0; i < num_autodiff_reps; i++) {
    x = VectorXd::Constant(2 * nq, i);
    desired_vdot = VectorXd::Constant(nv, i);
    multibody_plant_autodiff->SetPositionsAndVelocities(
        multibody_context_autodiff.get(), math::initializeAutoDiff(x));
    multibody_plant_autodiff->CalcInverseDynamics(
        *multibody_context_autodiff, math::initializeAutoDiff(desired_vdot),
        external_forces_autodiff);
  }
  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_autodiff_reps)
            << "x autodiff inverse dynamics took "
            << duration.count() << " ms. "
            << 1000 * duration.count() / num_autodiff_reps
            << " us per." << std::endl;

  // MBP forward dynamics
  start = my_clock::now();
  auto derivatives = multibody_plant.AllocateTimeDerivatives();

  for (int i = 0; i < num_reps; i++) {
    x = VectorXd::Constant(nq + nv, i);
    u = VectorXd::Constant(nu, i);
    multibody_context->FixInputPort(
        multibody_plant.get_actuation_input_port().get_index(), u);
    multibody_plant.SetPositionsAndVelocities(multibody_context.get(), x);
    multibody_plant.CalcTimeDerivatives(*multibody_context, derivatives.get());
  }
  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_reps)
            << "x forward dynamics took " << duration.count()
            << " ms. " << 1000 * duration.count() / num_reps
            << " us per." << std::endl;

  start = my_clock::now();
  auto derivatives_autodiff =
      multibody_plant_autodiff->AllocateTimeDerivatives();
  for (int i = 0; i < num_autodiff_reps; i++) {
    x = VectorXd::Constant(2 * nq, i);
    u = VectorXd::Constant(nu, i);

    multibody_context_autodiff->FixInputPort(
        multibody_plant_autodiff->get_actuation_input_port().get_index(),
        math::initializeAutoDiff(u));
    multibody_plant_autodiff->SetPositionsAndVelocities(
        multibody_context_autodiff.get(), math::initializeAutoDiff(x));
    multibody_plant_autodiff->CalcTimeDerivatives(*multibody_context_autodiff,
                                                  derivatives_autodiff.get());
  }
  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_autodiff_reps)
            << "x autodiff forward dynamics took "
            << duration.count() << " ms. "
            << 1000 * duration.count() / num_autodiff_reps
            << " us per." << std::endl;

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::do_main();
}
