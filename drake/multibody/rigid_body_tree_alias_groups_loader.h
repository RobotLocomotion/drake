#pragma once

#include <memory>
#include <string>
#include <vector>

#include "google/protobuf/text_format.h"

#include "drake/common/proto/protobuf.h"
#include "drake/multibody/alias_groups.pb.h"
#include "drake/multibody/rigid_body_tree_alias_groups.h"

template <typename T>
std::unique_ptr<RigidBodyTreeAliasGroups<T>>
RigidBodyTreeAliasGroupsLoadFromFile(const RigidBodyTree<T>* tree,
                                     const std::string& file_path) {
  auto rbtag = std::make_unique<RigidBodyTreeAliasGroups<T>>(tree);

  drake::rigid_body_tree::AliasGroups alias_groups;
  auto istream = drake::MakeFileInputStreamOrThrow(file_path);
  google::protobuf::TextFormat::Parse(istream.get(), &alias_groups);

  for (const auto& group : alias_groups.body_group()) {
    rbtag->AddBodyGroup(group.name(),
                        std::vector<std::string>(group.member().begin(),
                                                 group.member().end()));
  }
  for (const auto& group : alias_groups.joint_group()) {
    rbtag->AddJointGroup(group.name(),
                         std::vector<std::string>(group.member().begin(),
                                                  group.member().end()));
  }

  return rbtag;
}
