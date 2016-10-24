//
// Created by Pang Tao on 21/10/16.
//
#include <iostream>
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/common/drake_path.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/plants/RigidBodySystem.h"


// Includes for the planner.
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyIK.h"



using drake::BotVisualizer;

using Eigen::VectorXd;


namespace drake {
    class ValkyriePlant {
    public:
        template<typename T>
        using InputVector = drake::RigidBodySystem::InputVector<T>;
        template<typename T>
        using StateVector = drake::RigidBodySystem::StateVector<T>;
        template<typename T>
        using OutputVector = drake::RigidBodySystem::OutputVector<T>;

        ValkyriePlant() {
            sys_.reset(new drake::RigidBodySystem());
            sys_->AddModelInstanceFromFile(
                    GetDrakePath() + "/examples/Valkyrie_PT/val_description/urdf/valkyrie_sim_drake.urdf",
                    systems::plants::joints::kQuaternion);
            x0_ = VectorXd::Zero(sys_->getNumStates());
            SetInitialConfiguration();
        }

        const std::shared_ptr<RigidBodyTree>& get_rigid_body_tree() const;

        const VectorXd& get_initial_state() const;

        bool isTimeVarying() const;

        size_t getNumInputs() const;

        StateVector<double> output(const double& t,
                                   const StateVector<double>& x,
                                   const InputVector<double>& u) const;

        StateVector<double> dynamics(const double& t,
                                     const StateVector<double>& x,
                                     const InputVector<double>& u) const;

    private:
        std::unique_ptr<drake::RigidBodySystem> sys_;
        VectorXd x0_;
        void SetInitialConfiguration() {
            RigidBodyTree* tree = sys_->getRigidBodyTree().get();
            x0_.head(tree->get_num_positions()) = tree->getZeroConfiguration();
        }
    };

    //-------------------------implementation---------------------------------
    const VectorXd& ValkyriePlant::get_initial_state() const { return x0_;}

    const std::shared_ptr<RigidBodyTree>& ValkyriePlant::get_rigid_body_tree() const {
        return sys_->getRigidBodyTree();
    }

    bool ValkyriePlant::isTimeVarying() const {
        return sys_->isTimeVarying();
    }

    size_t ValkyriePlant::getNumInputs() const {
        return sys_->getNumInputs();
    }

    ValkyriePlant::StateVector<double>
    ValkyriePlant::output(const double& t,
                       const StateVector<double>& x,
                       const InputVector<double>& u) const {
        return sys_->output(t, x, u);
    }

    ValkyriePlant::StateVector<double>
    ValkyriePlant::dynamics(const double& t,
                         const StateVector<double>& x,
                         const InputVector<double>& u) const {
        return sys_->dynamics(t, x, u);
    }
}






int main() {
    using drake::ValkyriePlant;

    std::cout << "Have you had lunch?" << std::endl;
    std::shared_ptr<ValkyriePlant> val_sys = std::make_shared<ValkyriePlant>();
    auto const& tree = val_sys->get_rigid_body_tree();

    std::cout << "Number of positions: "<< tree->get_num_positions() << std::endl;


    //show it in drake visualizer!

    std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();
    if (!lcm->good()) return 1;

    auto visualizer =
    std::make_shared<BotVisualizer<ValkyriePlant::StateVector>>(lcm, tree);
    auto sys_with_vis = cascade(val_sys, visualizer);

    //drake::SimulationOptions options;
    //options.initial_step_size = 5e-5;
    //options.realtime_factor = 0.0;

    double final_time = std::numeric_limits<double>::infinity();
    runLCM(sys_with_vis, lcm, 0, final_time);



    return 0;
}