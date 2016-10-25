//
// Created by Pang Tao on 21/10/16.
//
#include <iostream>
#include "drake/examples/Valkyrie_PT/Valkyrie_plant.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/common/drake_path.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/plants/RigidBodySystem.h"

// Includes for the planner.
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"

using drake::BotVisualizer;
using drake::ValkyriePlant;
using namespace std;

int main() {

    std::cout << "Have you had lunch?" << std::endl;
    std::shared_ptr<ValkyriePlant> val_sys = std::make_shared<ValkyriePlant>();
    auto const& tree = val_sys->get_rigid_body_tree();

    std::cout << "Number of positions: "<< tree->get_num_positions() << std::endl;
    for(int i=0;i<tree->get_num_positions();i++) {
        cout << i << ":" << tree->get_position_name(i) << endl;
    }

    std::cout << "Number of velocities: "<< tree->get_num_velocities() << std::endl;
    for(int i=0;i<tree->get_num_velocities();i++) {
        cout << i << ":" << tree->get_velocity_name(i) << " " << tree->get_position_name(i) << endl;
    }

    std::cout << "Number of links: " << tree->get_num_bodies() << endl;
    for(int i=0;i<tree->get_num_bodies();i++) {
        cout << i << ":" << tree->getBodyOrFrameName(i) << endl;
    }



    //show it in drake visualizer!

    std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
    if (!lcm->good()) return 1;

    auto visualizer =
    std::make_shared<BotVisualizer<ValkyriePlant::StateVector>>(lcm, tree);
    auto sys_with_vis = cascade(val_sys, visualizer);

    drake::SimulationOptions options;
    options.initial_step_size = 5e-5;
    options.realtime_factor = 0.0;

    double final_time = std::numeric_limits<double>::infinity();
    runLCM(sys_with_vis, lcm, 0, final_time, val_sys->get_initial_state(), options);



    return 0;
}