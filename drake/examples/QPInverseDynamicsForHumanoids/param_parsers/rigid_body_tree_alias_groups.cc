#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/rigid_body_tree_alias_groups.h"

#include <fcntl.h>

#include <set>

#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {
namespace param_parsers {

template <typename T>
constexpr char RigidBodyTreeAliasGroups<T>::kBodyGroupsKeyword[];
template <typename T>
constexpr char RigidBodyTreeAliasGroups<T>::kJointGroupsKeyword[];

namespace {
// Inserts @p vec into @p mapping if @p key does not exist, or append @p vec
// to the existing vector in @p map. This function also guarantees the newly
// inserted elements do no introduce duplicates.
template <typename Type>
void InsertOrMergeVectorWithoutDuplicates(
    const std::string& key, const std::vector<Type>& vec,
    std::unordered_map<std::string, std::vector<Type>>* mapping) {
  DRAKE_DEMAND(mapping);
  std::set<Type> inserted;

  typename std::unordered_map<std::string, std::vector<Type>>::iterator it =
      mapping->find(key);
  if (it != mapping->end()) {
    inserted = std::set<Type>(it->second.begin(), it->second.end());
  }

  std::vector<Type> unique_vec;
  unique_vec.reserve(vec.size());
  for (auto const& element : vec) {
    if (inserted.find(element) == inserted.end()) {
      unique_vec.push_back(element);
      inserted.emplace(element);
    }
  }

  if (it == mapping->end()) {
    mapping->emplace(key, unique_vec);
  } else {
    it->second.insert(it->second.end(), unique_vec.begin(), unique_vec.end());
  }
}

}  // namespace

template <typename T>
void RigidBodyTreeAliasGroups<T>::AddBodyGroup(
    const std::string& group_name, const std::vector<std::string>& body_names) {
  std::vector<const RigidBody<T>*> bodies;
  bodies.reserve(body_names.size());
  for (const std::string& name : body_names) {
    bodies.push_back(tree_.FindBody(name));
  }

  InsertOrMergeVectorWithoutDuplicates(group_name, bodies, &body_groups_);
}

template <typename T>
void RigidBodyTreeAliasGroups<T>::AddJointGroup(
    const std::string& group_name,
    const std::vector<std::string>& joint_names) {
  std::vector<const RigidBody<T>*> bodies;
  std::vector<const DrakeJoint*> joints;
  bodies.reserve(joint_names.size());
  joints.reserve(joint_names.size());
  int q_size = 0;
  int v_size = 0;
  for (const std::string& name : joint_names) {
    bodies.push_back(tree_.FindChildBodyOfJoint(name));
    joints.push_back(&(bodies.back()->getJoint()));

    q_size += joints.back()->get_num_positions();
    v_size += joints.back()->get_num_velocities();
  }

  InsertOrMergeVectorWithoutDuplicates(group_name, joints, &joint_groups_);

  std::vector<int> q_indices, v_indices;
  q_indices.reserve(q_size);
  v_indices.reserve(v_size);

  for (const RigidBody<T>* body : bodies) {
    int q_start = body->get_position_start_index();
    int q_end = q_start + body->getJoint().get_num_positions();
    for (int i = q_start; i < q_end; ++i) q_indices.push_back(i);

    int v_start = body->get_velocity_start_index();
    int v_end = v_start + body->getJoint().get_num_velocities();
    for (int i = v_start; i < v_end; ++i) v_indices.push_back(i);
  }

  InsertOrMergeVectorWithoutDuplicates(group_name, q_indices,
                                       &position_groups_);
  InsertOrMergeVectorWithoutDuplicates(group_name, v_indices,
                                       &velocity_groups_);
}

template <typename T>
void RigidBodyTreeAliasGroups<T>::LoadFromFile(
    const std::string& file_path) {
  AliasGroups alias_groups;
  int fid = open(file_path.data(), O_RDONLY);
  if (fid < 0) {
    throw std::runtime_error("Cannot open file " + file_path);
  }
  google::protobuf::io::FileInputStream istream(fid);
  google::protobuf::TextFormat::Parse(&istream, &alias_groups);
  istream.Close();

  for (const auto& group : alias_groups.body_group()) {
    AddBodyGroup(group.name(), std::vector<std::string>(group.member().begin(),
                                                        group.member().end()));
  }
  for (const auto& group : alias_groups.joint_group()) {
    AddJointGroup(group.name(), std::vector<std::string>(group.member().begin(),
                                                         group.member().end()));
  }
}

template class RigidBodyTreeAliasGroups<double>;

}  // namespace param_parsers
}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
