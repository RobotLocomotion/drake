#include <cmath>
#include <cstdlib>
#include <memory>
#include <sstream>

#include "drake/Path.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/n_ary_state.h"
#include "drake/systems/n_ary_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodyTree.h"

#include "drake/examples/Cars/curve2.h"
#include "drake/examples/Cars/gen/euler_floating_joint_state.h"
#include "drake/examples/Cars/trajectory_car.h"

using Drake::AffineSystem;
using Drake::BotVisualizer;
using Drake::NullVector;
using Drake::cascade;

namespace drake {
namespace {

/// A figure-eight.  One loop has a radius of @p radius - @p inset,
/// the other loop has a radius of @p radius + @p inset.
Curve2<double> MakeCurve(double radius, double inset) {
  // TODO(jwnimmer-tri) This function will be rewritten once we have
  // proper splines.  Don't try too hard to understand it.  Run the
  // demo to see it first, and only then try to understand the code.

  typedef Curve2<double>::Point2 Point2d;
  std::vector<Point2d> waypoints;

  // Start (0, +i).
  // Straight right to (+r, +i).
  // Loop around (+i, +r).
  // Straight back to (+i, 0).
  waypoints.push_back({0.0, inset});
  for (int theta_deg = -90; theta_deg <= 180; ++theta_deg) {
    const Point2d center{radius, radius};
    const double theta = theta_deg * M_PI / 180.0;
    const Point2d direction{std::cos(theta), std::sin(theta)};
    waypoints.push_back(center + (direction * (radius - inset)));
  }
  waypoints.push_back({inset, 0.0});

  // Start (+i, 0).
  // Straight down to (+i, -r).
  // Loop around (-r, +i).
  // Straight back to start (implicitly via segment to waypoints[0]).
  for (int theta_deg = 0; theta_deg >= -270; --theta_deg) {
    const Point2d center{-radius, -radius};
    const double theta = theta_deg * M_PI / 180.0;
    const Point2d direction{std::cos(theta), std::sin(theta)};
    waypoints.push_back(center + (direction * (radius + inset)));
  }

  // Many copies.
  std::vector<Point2d> looped_waypoints;
  for (int copies = 0; copies < 100; ++copies) {
    std::copy(waypoints.begin(), waypoints.end(),
              std::back_inserter(looped_waypoints));
  }
  looped_waypoints.push_back(waypoints.front());

  return Curve2<double>(looped_waypoints);
}

int do_main(int argc, const char* argv[]) {
  int num_cars = 100;
  if (argc == 2) {
    num_cars = atoi(argv[1]);
    if (num_cars < 1) {
      std::cerr << "The number of cars must be >= 1.\n";
      std::exit(1);
    }
  }

  // Make a linear system to map NPC car state to the state vector of a
  // floating joint, allowing motion and steering in the x-y plane only.
  const int insize = SimpleCarState<double>().size();
  const int outsize = EulerFloatingJointState<double>().size();
  Eigen::MatrixXd D;
  D.setZero(outsize, insize);
  D(EulerFloatingJointStateIndices::kX, SimpleCarStateIndices::kX) = 1;
  D(EulerFloatingJointStateIndices::kY, SimpleCarStateIndices::kY) = 1;
  D(EulerFloatingJointStateIndices::kYaw, SimpleCarStateIndices::kHeading) = 1;
  EulerFloatingJointState<double> y0;
  auto car_vis_adapter = std::make_shared<
      AffineSystem<
        NullVector,
        SimpleCarState,
        EulerFloatingJointState>>(
            Eigen::MatrixXd::Zero(0, 0),
            Eigen::MatrixXd::Zero(0, insize),
            Eigen::VectorXd::Zero(0),
            Eigen::MatrixXd::Zero(outsize, 0), D,
            toEigen(y0));

  const std::string kSedanUrdf = Drake::getDrakePath() +
      "/examples/Cars/models/sedan.urdf";
  const std::string kBreadtruckUrdf = Drake::getDrakePath() +
      "/examples/Cars/models/breadtruck.urdf";

  // RigidBodyTree for visualization.
  auto world_tree = std::make_shared<RigidBodyTree>();

  // NarySystem for car 'physics'.
  //  U: ()
  //  X: ()
  //  Y: [(xy-position, heading, velocity), ...] per SimpleCarState
  auto cars_system = std::make_shared<NArySystem<TrajectoryCar>>();
  // NarySystem for car visualization.
  // BotVisualizer:
  //  U: [(xy-position, heading, velocity), ...] per SimpleCarState
  //  X: ()
  //  Y: [(x, y, z, roll, pitch, yaw), ...] per DrakeJoint::ROLLPITCHYAW per car
  auto cars_vis_adapter = std::make_shared<
    NArySystem<decltype(car_vis_adapter)::element_type>>();
  // NB:  One could compose the other way as well (i.e., individually cascade
  // each TrajectoryCar with a car_vis_adapter, and then stack each of those
  // pairs into a single NArySystem).

  // The possible curves to trace (lanes).
  const std::vector<Curve2<double>> curves{
    MakeCurve(40.0, 0.0),  // BR
    MakeCurve(40.0, 4.0),  // BR
    MakeCurve(40.0, 8.0),
  };

  // Add all of the desired cars.
  for (int i = 0; i < num_cars; ++i) {
    world_tree->addRobotFromURDF((i % 5) ? kSedanUrdf : kBreadtruckUrdf,
                                 DrakeJoint::ROLLPITCHYAW,
                                 nullptr /*weld_to_frame*/);
    // TODO(maddog) Hmm... it appears that drake_visualizer wants unique names,
    //              on *links*, otherwise only one of the same-named links will
    //              get updated joint parameters.
    std::ostringstream name;
    name << "car-" << i;
    world_tree->bodies.back()->name_ = name.str();

    // Magic car placement to make a good visual demo.
    const auto& curve = curves[i % curves.size()];
    const double start_time = (i / curves.size()) * 0.8;
    const double kSpeed = 8.0;
    auto system = std::make_shared<TrajectoryCar>(curve, kSpeed, start_time);
    cars_system->AddSystem(system);
    cars_vis_adapter->AddSystem(car_vis_adapter);
  }

  // Add LCM support, for visualization.
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
  auto visualizer =
      std::make_shared<Drake::BotVisualizer<
        decltype(cars_vis_adapter)::element_type::OutputVector>>(
            lcm, world_tree);

  // Compose the system together from the parts.
  auto the_system = Drake::cascade(Drake::cascade(
      cars_system, cars_vis_adapter), visualizer);

  decltype(the_system)::element_type::StateVector<double> initial_state;
  Drake::SimulationOptions options;
  options.realtime_factor = 0.0;
  const double t0 = 0.0;
  const double tf = std::numeric_limits<double>::infinity();
  runLCM(the_system, lcm, t0, tf, initial_state, options);

  return 0;
}

}  // namespace
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::do_main(argc, argv);
}
