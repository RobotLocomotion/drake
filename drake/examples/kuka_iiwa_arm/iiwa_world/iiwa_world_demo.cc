#include <gflags/gflags.h>

#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_world_sim_builder.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"


DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

namespace {

int DoMain() {
  auto iiwa_world = std::make_unique<IiwaWorldSimBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  iiwa_world->AddObjectUrdf("iiwa", "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf");
  iiwa_world->AddObjectUrdf(
      "table",
      "/examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table.sdf");
  iiwa_world->AddObjectUrdf(
      "cylinder",
      "/examples/kuka_iiwa_arm/models/objects/simple_cylinder.urdf");
  iiwa_world->AddObjectUrdf(
      "cuboid", "/examples/kuka_iiwa_arm/models/objects/simple_cuboid.urdf");

  iiwa_world->AddObjectFixedToWorld(Eigen::Vector3d::Zero() /* xyz */,
                                    Eigen::Vector3d::Zero() /* rpy */, "table");
  iiwa_world->AddGroundToTree();

  iiwa_world->SetPenetrationContactParameters(4500 /* penetration_stiffness */,
                                              1.0 /* penetration_damping */,
                                              1.0 /* contact friction */);

  // The z coordinate of the top of the table in the world frame.
  const double kTableTopZInWorld = 0.736 + 0.057 / 2;

  // The positions of the IIWA robot, two cylinders and the cuboid was fixed
  // in a manner that they are distributed over the surface of the heavy duty
  // table. Only the positions are set, as the default orientations are used
  // in each case.
  const Eigen::Vector3d kRobotBase(-0.25, -0.75, kTableTopZInWorld);
  const Eigen::Vector3d kBoxBase(-0.45, -0.4, kTableTopZInWorld + 0.15);
  const Eigen::Vector3d kCylinder1Base(-0.5, -0.60, kTableTopZInWorld + 0.1);
  const Eigen::Vector3d kCylinder2Base(-0.05, -0.75, kTableTopZInWorld + 0.1);

  iiwa_world->AddObjectFixedToWorld(kRobotBase,
                                    Eigen::Vector3d::Zero() /* rpy */, "iiwa");
  iiwa_world->AddObjectFloatingToWorld(
      kCylinder1Base, Eigen::Vector3d::Zero() /* rpy */, "cylinder");
  iiwa_world->AddObjectFloatingToWorld(
      kCylinder2Base, Eigen::Vector3d::Zero() /* rpy */, "cylinder");
  iiwa_world->AddObjectFloatingToWorld(
      kBoxBase, Eigen::Vector3d::Zero() /* rpy */, "cuboid");

  std::unique_ptr<drake::systems::DiagramBuilder<double>> builder{
      std::make_unique<drake::systems::DiagramBuilder<double>>()};

  std::cout<<"About to call build\n";
  auto robot_world_system =
      builder->template AddSystem<drake::systems::Diagram<double>>(iiwa_world->Build());
  std::cout<<"Back in calling function \n";

  // Instantiates a constant source that outputs a vector of zeros.
  VectorX<double> constant_value(iiwa_world->GetPlantInputSize());
  constant_value.setZero();
  std::cout<<"Constant value :"<<constant_value<<"\n";

  std::cout<<"Building source\n";
  auto const_source_ =
      builder->template
          AddSystem<drake::systems::ConstantVectorSource<double>>(constant_value);
  builder->ExportOutput(robot_world_system->get_output_port(0));


  // Connects the constant source's output port to the RigidBodyPlant's input
  // port. This effectively results in the robot being uncontrolled.
//  builder->Cascade(*const_source_,*robot_world_system);
  builder->Connect(const_source_->get_output_port(),
                   robot_world_system->get_input_port(0));

  // Exposing output and input port.
 // builder->ExportInput(plant_->get_input_port(0));

  std::cout<<"building new diagram\n";
  auto diagram = builder->Build();

  std::cout<<"Finished rebuilding a new diagram\n";

//
//  if(diagram->SortOrderIsCorrect())
//    std::cout<<"sort order is correct\n";
//  else
//    std::cout<<"sort order is not correct\n";

  std::cout<<"Building simulator\n";
  auto simulator = std::make_unique<systems::Simulator<double>>(*diagram);



  std::cout<<"Setting 0 zonfiguration\n";
  iiwa_world->SetZeroConfiguration(simulator.get(), diagram.get());

  std::cout<<"Initializing simulation\n";
  simulator->Initialize();
  std::cout<<"Starting StepTo\n";
  simulator->StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}
