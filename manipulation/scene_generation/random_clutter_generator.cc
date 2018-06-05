#include "drake/manipulation/scene_generation/random_clutter_generator.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <thread>

#include "drake/common/eigen_types.h"
#include "drake/manipulation/util/simple_tree_visualizer.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/rigid_body_constraint.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace manipulation {
namespace scene_generation {
using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::VectorXd;
using Eigen::Vector3d;
using std::map;
using std::string;
using util::WorldSimTreeBuilder;
using std::default_random_engine;
using std::uniform_real_distribution;
using systems::RigidBodyPlant;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::Simulator;
using systems::RungeKutta2Integrator;
using systems::ContinuousState;
using util::ModelInstanceInfo;

namespace {
  VectorXd return_vector = VectorXd::Zero(7);
  return_vector.head(3) = pose.translation();
  Quaterniond return_quat = Quaterniond(pose.linear());
  return_vector.tail(4) = (VectorXd(4) << return_quat.w(), return_quat.x(),
                           return_quat.y(), return_quat.z())
                              .finished();
  return return_vector;
}

Isometry3<double> VectorToIsometry3d(const VectorX<double>& q) {
  Isometry3<double> return_pose = Isometry3<double>::Identity();
  return_pose.translation() = q.head(3);
  Quaterniond return_quat = Quaterniond(q(3), q(4), q(5), q(6));
  return_pose.linear() = return_quat.toRotationMatrix();
  return return_pose;
}

template <typename T>
void AppendTo(T* t, const T& t_to_append) {
  int append_size = t_to_append.size();
  T t_copy = *t;
  int old_size = t->size();
  t->resize(old_size + append_size);
  t->head(old_size) = t_copy;
  t->tail(append_size) = t_to_append;
}

} // namespace

void PopulateWithFloatingObjectRepetitions(
  util::WorldSimTreeBuilder<double>* tree_builder, 
  std::map<std::string, int> model_map) {
  for (map<string, int>::iterator it = model_map_.begin();
       it != model_map_.end(); ++it) {
    stringstream model_name;
    model_name << "model_" << ctr;
    tree_builder->StoreModel(model_name.str(), it->first);

    for (int i = 0; i< it->second; ++i) {
    // All floating objects are added to origin. 
    Isometry3<double> model_pose =
        Isometry3<double>::Identity();
    int tree_instances = tree_builder->AddFloatingModelInstance(
        it, model_pose.translation(), model_pose.linear().eulerAngles(0, 1, 2));
  }
}


const double kRandomInitializationDimensionRatio = 0.7;


RandomClutterGenerator::RandomClutterGenerator(
  const RigidBodyTreed& scene_tree, 
      Eigen::Isometry3<double> bounding_box_pose,
      Vector3<double> bounding_box_size, 
      std::vector<int> clutter_model_instances,
      double min_inter_object_distance) : 
scene_tree_(scene_tree), 
bounding_box_pose_(bounding_box), 
bounding_box_size_(bounding_box_size_), 
clutter_model_instances_(clutter_model_instances) {
  // Checks that the number of requested instances > 0 & < the total number
  // of instances in the tree.
  DRAKE_DEMAND(clutter_model_instances.size() > 0);
  DRAKE_DEMAND(clutter_model_instances.size() < tree.get_num_model_instances);
  for(auto &it = clutter_model_instances) {
    // check that the tree contains the model instance in question. i.e atleast
    // one body exists for each model instance listed in clutter_model_instance.
    DRAKE_DEMAND(scene_tree_.FindModelInstanceBodies(it).size()>0);  
  }
}
    
Isometry3<double> RandomClutterGenerator::GenerateRandomBoundedPose() {
  Quaterniond random_quat = Quaterniond::UnitRandom();
  Isometry3d return_pose = Isometry3d::Identity();
  return_pose.linear() = random_quat.toRotationMatrix();

  uniform_real_distribution<double> x_distribution(
   -bounding_box_size_(0) * (kRandomInitializationDimensionRatio / 2),
    bounding_box_size_(0) * (kRandomInitializationDimensionRatio / 2));

    uniform_real_distribution<double> y_distribution(
        -bounding_box_size_(1) * (kRandomInitializationDimensionRatio / 2),
        bounding_box_size_(1) * (kRandomInitializationDimensionRatio / 2));

    uniform_real_distribution<double> z_distribution(
        -bounding_box_size_(2) * (kRandomInitializationDimensionRatio/2), 
        bounding_box_size_(2) * (kRandomInitializationDimensionRatio/2));

    return_pose.translation() =
        (VectorXd(3) << x_distribution(generator_), y_distribution(generator_),
         z_distribution(generator_))
            .finished();
  return_pose.makeAffine();
  // return pose is computed in the Bounding box frame B. Transform and return 
  // pose in World frame (W).
  return X_WB_*return_pose;
}

VectorX<double> RandomClutterGenerator::GetRandomBoundedConfiguration(
  const VectorX<double> q_inital) {
  VectorX<double> q_random = q_inital;

  int num_model_instances = scene_tree_.get_num_model_instances();
  VectorX<double> x = VectorX<double>::Zero(model_tree.get_num_positions());

  // iterate through tree. Set random poses for each floating element.
  for (auto it : clutter_model_instances_) {
    auto base_body_index = scene_tree_.FindBaseBodies(it);

    // TODO(naveenoid) : resolve multi-link model initialization.
    std::vector<const RigidBody<double>*> model_instance_bodies =
        scene_tree_.FindModelInstanceBodies(it);
    Isometry3d segment_pose = GenerateRandomBoundedPose();
    q_random.segment<7>(model_instance_bodies[0]->get_position_start_index()) =
        Isometry3dToVector(segment_pose);
  }
  return q_random;
}

VectorX<double> RandomClutterGenerator::GetNominalConfiguration(
    const RigidBodyTreed& model_tree) {
  VectorX<double> q_zero =
      VectorX<double>::Zero(model_tree.get_num_positions());

  int num_model_instances = model_tree.get_num_model_instances();
  VectorX<double> x = VectorX<double>::Zero(model_tree.get_num_positions());

  // iterate through tree. Set random poses for each floating element.
  for (int i = 0; i < num_model_instances; ++i) {
    auto base_body_index = model_tree.FindBaseBodies(i);
    // TODO(naveenoid) : figure out if the model instance has more than 1 link

    std::vector<const RigidBody<double>*> model_instance_bodies =
        model_tree.FindModelInstanceBodies(i);

    Isometry3d segment_pose = Isometry3d::Identity();
    q_zero.segment<7>(model_instance_bodies[0]->get_position_start_index()) =
        Isometry3dToVector(segment_pose);
  }
  return q_zero;
}

VectorX<double> RandomClutterGenerator::Generate(
  const VectorX<double> q_initial) {
  
  VectorX<double> q_ini = 
   GetRandomBoundedConfiguration(q_initial);
  
  if (visualize_steps_) {
    SimpleTreeVisualizer visualizer(*scene_tree_.get(), lcm_);
  }
  VectorX<double> q_ik_result = q_initial;

  if (with_ik_) {
    int ik_result_code = 100;
    while(ik_result_code!= 1) {
      VectorX<double> q_nominal = GetNominalConfiguration(*models_tree.get());
      // setup constraint array.
      std::vector<RigidBodyConstraint*> constraint_array;

      // set MinDistanceConstraint
      std::vector<int> active_bodies_idx;
      std::set<std::string> active_group_names;
      auto min_distance_constraint = MinDistanceConstraint(
              models_tree.get(), inter_object_distance_ /* min_distance */,
              active_bodies_idx, active_group_names);
      constraint_array.push_back(&min_distance_constraint);

      Vector3d lb, ub;
      if (is_bounding_box_enabled_) {
        lb = (VectorXd(3) << (-bounding_length_ *
                              kRandomInitializationDimensionRatio * 0.5),
                (-bounding_width_ * kRandomInitializationDimensionRatio * 0.5), 0.0)
                .finished();
        ub = -lb;
        ub(2) = 1.0 * bounding_height_;

        drake::log()->info("Bounding volume lb : {}", lb.transpose());
        drake::log()->info("Bounding volume ub : {}", ub.transpose());
      } else {
        double inf = std::numeric_limits<double>::infinity();
        lb = (VectorX<double>(3) << -inf, -inf, 0).finished();
        ub = (VectorX<double>(3) << inf, inf, inf).finished();
      }

      Eigen::VectorXi iAfun, jAvar;
      VectorX<double> A, orientation_lb, orientation_ub;
      Quaterniond unit_quat = Quaterniond::UnitRandom();

      int floating_body_index = 0;
      int num_floating_bodies = 0;

      for (int i = 0; i < models_tree->get_num_bodies(); ++i) {
        // set WorldPositionConstraint (bounds every object to the bounding box)
        auto world_position_constraint =
                std::make_unique<WorldPositionConstraint>(
                        models_tree.get(), i, Eigen::Vector3d::Zero(), lb, ub);
        constraint_array.push_back(world_position_constraint.get());

        if (i > 1) {
          if (models_tree->get_body(i).getJoint().is_floating()) {
            int start_index = models_tree->get_body(i).get_position_start_index();
            AppendTo<Eigen::VectorXi>(
                    &jAvar,
                    (Eigen::VectorXi(4) << start_index + 3, start_index + 4,
                            start_index + 5, start_index + 6)
                            .finished());
            AppendTo<Eigen::VectorXi>(
                    &iAfun,
                    (Eigen::VectorXi(4) << floating_body_index++,
                            floating_body_index++, floating_body_index++,
                            floating_body_index++)
                            .finished());

            ++num_floating_bodies;

            AppendTo<Eigen::VectorXd>(
                    &orientation_lb,
                    (VectorX<double>(4) << unit_quat.w(), unit_quat.x(),
                            unit_quat.y(), unit_quat.z())
                            .finished());
          }
        }
      }
      orientation_ub = orientation_lb;

      A = VectorX<double>::Ones(iAfun.size());

      // Adding a single linear constraint for setting all orientations to their
      // (initial) random orientations.
      auto linear_orientation_constraint =
              std::make_unique<SingleTimeLinearPostureConstraint>(
                      models_tree.get(), iAfun, jAvar, A, orientation_lb, orientation_ub);

      constraint_array.push_back(linear_orientation_constraint.get());

      drake::log()->info("Constraint array size {}", constraint_array.size());

      IKoptions ikoptions(models_tree.get());
      ikoptions.setDebug(true);

      // setup IK problem and run.
      IKResults ik_results = inverseKinSimple(
              models_tree.get(), q_initial, q_nominal, constraint_array, ikoptions);

      int indx = 0;
      for (auto it : ik_results.info) {
        drake::log()->info(" IK Result code {} : {}", indx++, it);
        ik_result_code = it;
        if(ik_result_code!= 1) {
          drake::log()->info("IK failure, recomputing IK");
        }
      }

      if (!ik_results.q_sol.empty()) {
        q_ik_result = ik_results.q_sol.back();
      }
    }
  }
  drake::log()->info("About to simulate dropping");

  // Simulate Fall and return the state.
  ModelPosePair model_pose_pair = DropObjectsToGround(
      std::make_unique<RigidBodyPlant<double>>(std::move(models_tree)),
      q_ik_result);
//  TransformPoses(&model_pose_pair);

  return model_pose_pair;
}

// void RandomClutterGenerator::Generate(
//     util::WorldSimTreeBuilder<double>* world_sim_tree_builder) {
//   ModelPosePair model_pose_pair = Generate();

//   DRAKE_DEMAND(world_sim_tree_builder != nullptr);

//   int ctr = 0;
// //  for (auto& it : model_map_) {
// //    stringstream model_name;
// //    model_name << "model_" << ctr++;
// //    drake::log()->info("{} added as [{}]", model_name.str(), it.first);
// //    world_sim_tree_builder->StoreModel(model_name.str(), it.first);
// //  }
// //  std::vector<string> repetion_included_model_names;
// //  for (map<string, int>::iterator it = model_map_.begin();
// //       it != model_map_.end(); ++it) {
// //    stringstream model_name;
// //    model_name << "model_" << ctr;
// //    world_sim_tree_builder->StoreModel(model_name.str(), it->first);
// //
// //    drake::log()->info("[{}] added as [{}]", model_name.str(), it->first);
// //    ctr++;
// //  }

//   ctr = 0;
//   for (auto& it : model_pose_pair) {
//     stringstream model_name;
//     model_name << "model_" << ctr++;

//     world_sim_tree_builder->StoreModel(model_name.str(), it.first);

//     Isometry3<double> pose = X_WC_*it.second;
//     drake::log()->info("Adding pose of [{}] as [{}] with {}",
//                        model_name.str(), it.first, pose.translation().transpose());
//     ctr++;
//     world_sim_tree_builder->AddFloatingModelInstance(
//         model_name.str(), pose.translation(),
//         pose.linear().eulerAngles(0, 1, 2));
//   }
// }

// RandomClutterGenerator::ModelPosePair
// RandomClutterGenerator::DropObjectsToGround(
//     std::unique_ptr<RigidBodyPlant<double>> plant_ptr,
//     const VectorX<double>& q_ik) {
//   DiagramBuilder<double> builder;

//   // Transferring ownership of tree to the RigidBodyPlant.
//   auto plant = builder.template AddSystem(std::move(plant_ptr));
//   plant->set_name("RBP");
//   // Decreasing penetration stiffness from default of 10000 to 1000 and
//   // increading damping from 2 to 10. This hopefully prevents sim from
//   // exploding...
//   //  plant->set_normal_contact_parameters(
//   //      1000, 10);

//   if (visualize_steps_) {
//     auto drake_visualizer =
//         builder.template AddSystem<systems::DrakeVisualizer>(
//             plant->get_rigid_body_tree(), lcm_);
//     drake_visualizer->set_name("DV");

//     builder.Connect(plant->get_output_port(0),
//                     drake_visualizer->get_input_port(0));
//   }

//   if (plant->get_num_actuators() > 0) {
//     auto zero_input = builder.template AddSystem<systems::ConstantVectorSource>(
//         Eigen::VectorXd::Zero(plant->get_num_actuators()));
//     builder.Connect(zero_input->get_output_port(), plant->get_input_port(0));
//   }

//   auto sys = builder.Build();
//   Simulator<double> simulator(*sys);

//   // setting initial condition
//   // auto diagram_context = sys->CreateDefaultContext();
//   VectorX<double> x_initial = VectorX<double>::Zero(plant->get_num_states());

//   x_initial.head(plant->get_num_positions()) = q_ik;
//   simulator.get_mutable_context()
//       .get_mutable_continuous_state_vector()
//       .SetFromVector(x_initial);

//   simulator.Initialize();

//   if (visualize_steps_) {
//     // simulator.set_target_realtime_rate(1.0);
//   }
//   simulator.reset_integrator<RungeKutta2Integrator<double>>(
//       *sys, 0.001, &simulator.get_mutable_context());
//   simulator.get_mutable_integrator()->set_maximum_step_size(0.001);
//   simulator.get_mutable_integrator()->set_fixed_step_mode(true);

//   drake::log()->info("Starting Simulation");
//   simulator.StepTo(3.5);

//   drake::log()->info("Copying the return vector");
//   VectorX<double> q_return =
//       simulator.get_context().get_continuous_state_vector().CopyToVector();

//   return EncodeConfiguration(plant->get_rigid_body_tree(), q_return);
// }

// RandomClutterGenerator::ModelPosePair
// RandomClutterGenerator::EncodeConfiguration(const RigidBodyTreed& model_tree,
//                                             const VectorX<double>& q,
//                                             const Isometry3<double>& X_WC) {
//   ModelPosePair model_pose_pair;

//   int num_model_instances = model_tree.get_num_model_instances();
//   VectorX<double> x = VectorX<double>::Zero(model_tree.get_num_positions());

//   // iterate through tree. Set random poses for each floating element.
//   for (int i = 0; i < num_model_instances; ++i) {
//     auto base_body_index = model_tree.FindBaseBodies(i);

//     // TODO(naveenoid) : figure out if the model instance has more than 1 link
//     std::vector<const RigidBody<double>*> model_instance_bodies =
//         model_tree.FindModelInstanceBodies(i);
//     Isometry3<double> model_pose = VectorToIsometry3d(
//         q.segment<7>(model_instance_bodies[0]->get_position_start_index()));
//     model_pose_pair.push_back(std::make_pair(
//         model_instances_info_[i].absolute_model_path, X_WC * model_pose));
//   }
//   return model_pose_pair;
// }

// void RandomClutterGenerator::TransformPoses(ModelPosePair* model_pose_pair) {
//   for (auto& it : *model_pose_pair) {
//     drake::log()->info("Translating pose of {} from {} to {}",
//                        it.first, it.second.translation().transpose(),
//                        (X_WC_*it.second).translation().transpose());
//     it.second = X_WC_ * it.second;
//   }
// }

}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake