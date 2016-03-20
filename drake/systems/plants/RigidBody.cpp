
#include "RigidBody.h"
#include <stdexcept>

#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/testUtil.h"

using namespace std;
using namespace Eigen;

RigidBody::RigidBody()
    : parent(nullptr),
      collision_filter_group(DrakeCollision::DEFAULT_GROUP),
      collision_filter_ignores(DrakeCollision::NONE_MASK) {
  robotnum = 0;
  position_num_start = 0;
  velocity_num_start = 0;
  body_index = 0;
  mass = 0.0;
  com = Vector3d::Zero();
  I << Matrix<double, TWIST_SIZE, TWIST_SIZE>::Zero();
}

void RigidBody::setJoint(std::unique_ptr<DrakeJoint> new_joint) {
  this->joint = move(new_joint);
}

const DrakeJoint& RigidBody::getJoint() const {
  if (joint) {
    return (*joint);
  } else {
    throw runtime_error("Joint is not initialized");
  }
}

bool RigidBody::hasParent() const { return parent != nullptr; }

void RigidBody::addVisualElement(const DrakeShapes::VisualElement& element) {
  visual_elements.push_back(element);
}

const DrakeShapes::VectorOfVisualElements& RigidBody::getVisualElements()
    const {
  return visual_elements;
}

void RigidBody::setCollisionFilter(const DrakeCollision::bitmask& group,
                                   const DrakeCollision::bitmask& ignores) {
  setCollisionFilterGroup(group);
  setCollisionFilterIgnores(ignores);
}

bool RigidBody::appendCollisionElementIdsFromThisBody(
    const string& group_name, vector<DrakeCollision::ElementId>& ids) const {
  auto group_ids_iter = collision_element_groups.find(group_name);
  if (group_ids_iter != collision_element_groups.end()) {
    ids.reserve(ids.size() + distance(group_ids_iter->second.begin(),
                                      group_ids_iter->second.end()));
    ids.insert(ids.end(), group_ids_iter->second.begin(),
               group_ids_iter->second.end());
    return true;
  } else {
    return false;
  }
}

bool RigidBody::appendCollisionElementIdsFromThisBody(
    vector<DrakeCollision::ElementId>& ids) const {
  ids.reserve(ids.size() + collision_element_ids.size());
  ids.insert(ids.end(), collision_element_ids.begin(),
             collision_element_ids.end());
  return true;
}

void RigidBody::applyTransformToJointFrame(const Eigen::Isometry3d& transform_body_to_joint) {
  // std::cout << "RigidBody::applyTransformToJointFrame: Method called!\n"
  //           << " - link name: " << linkname << "\n"
  //           << " - transform_body_to_joint=\n"
  //           << transform_body_to_joint.matrix() << "\n"
  //           << " - I prior to transform=\n"
  //           << I << std::endl;

  I = transformSpatialInertia(transform_body_to_joint, I);

  // std::cout << "  - I after transform:\n"
  //           << I << std::endl;

  for (auto& v : visual_elements) {
    v.setLocalTransform(transform_body_to_joint * v.getLocalTransform());
  }
}

RigidBody::CollisionElement::CollisionElement(const CollisionElement& other)
    : DrakeCollision::Element(other), body(other.getBody()) {}

RigidBody::CollisionElement::CollisionElement(
    const Isometry3d& T_element_to_link, std::shared_ptr<RigidBody> body)
    : DrakeCollision::Element(T_element_to_link), body(body) {}

RigidBody::CollisionElement::CollisionElement(
    const DrakeShapes::Geometry& geometry, const Isometry3d& T_element_to_link,
    std::shared_ptr<RigidBody> body)
    : DrakeCollision::Element(geometry, T_element_to_link), body(body) {}

RigidBody::CollisionElement* RigidBody::CollisionElement::clone() const {
  return new CollisionElement(*this);
}

const std::shared_ptr<RigidBody>& RigidBody::CollisionElement::getBody() const {
  return this->body;
}

bool RigidBody::CollisionElement::collidesWith(
    const DrakeCollision::Element* other) const {
  // DEBUG
  // cout << "RigidBody::CollisionElement::collidesWith: START" << endl;
  // END_DEBUG
  auto other_rb = dynamic_cast<const RigidBody::CollisionElement*>(other);
  bool collides = true;
  if (other_rb != nullptr) {
    collides = this->body->collidesWith(*other_rb->body);
    // DEBUG
    // cout << "RigidBody::CollisionElement::collidesWith:" << endl;
    // cout << "  " << this->body->linkname << " & " <<
    // other_rb->body->linkname;
    // cout << ": collides = " << collides << endl;
    // END_DEBUG
  }
  return collides;
}

ostream& operator<<(ostream& out, const RigidBody& b) {
  std::string joint_name =
      b.hasParent() ? b.getJoint().getName() : "no parent joint";
  out << "RigidBody(" << b.linkname << "," << joint_name << ")";
  return out;
}

#define PRINT_STMT(x) std::cout << "RigidBody: EQUALS: " << x << std::endl;

bool operator==(const RigidBody & rb1, const RigidBody & rb2) {
  bool result = true;

  if (rb1.linkname.compare(rb2.linkname) != 0) {
    PRINT_STMT("Link names do not match "
      << rb1.linkname << " vs. " << rb2.linkname)
    result = false;
  }

  if (result && rb1.hasParent() && !rb2.hasParent()) {
    PRINT_STMT("LHS rigid body has parent whereas RHS rigid body does not. Linkname is " << rb1.linkname << ".")
    result = false;
  }

  if (result && !rb1.hasParent() && rb2.hasParent()) {
    PRINT_STMT("LHS rigid body does not have parent whereas RHS rigid body does. Linkname is " << rb1.linkname << ".")
    result = false;
  }

  if (result && rb1.hasParent() && rb2.hasParent()) {
      if (rb1.parent->linkname.compare(rb2.parent->linkname) != 0) {
        PRINT_STMT("Link " << rb1.linkname << " have different parent links ("
          << rb1.parent->linkname << " vs. " << rb2.parent->linkname << ".")
        result = false;
      }
  }

  if (result && rb1.body_index != rb2.body_index) {
    PRINT_STMT("Body indices do not match: "
      << rb1.body_index << " vs. " << rb2.body_index)
    result = false;
  }

  if (result && rb1.position_num_start != rb2.position_num_start) {
    PRINT_STMT("Variable position_num_start does not match: "
      << rb1.position_num_start << " vs. " << rb2.position_num_start)
    result = false;
  }

  if (result && rb1.velocity_num_start != rb2.velocity_num_start) {
    PRINT_STMT("Variable velocity_num_start does not match: "
      << rb1.velocity_num_start << " vs. " << rb2.velocity_num_start)
    result = false;
  }

  // Do not compare visual_elements for now.
  // DrakeShapes::VectorOfVisualElements visual_elements;

  // Do not compare collision elements for now.
  // if (result) {
  //   for (auto& ce1 : rb1.collision_element_ids) {
  //     // Do not enforce a specific ordering of bodies
  //     bool found = false;
  //     for (auto& ce2 : rb2.collision_element_ids) {
  //       if (ce1 == ce2)
  //         found = true;
  //     }
  //     if (!found) {
  //       PRINT_STMT("Could not find collision element matching " << ce1)
  //       result = false;
  //     }
  //   }
  // }

  // Do not compare collision element groups for now
  // std::map<std::string, std::vector<DrakeCollision::ElementId> >
  //     collision_element_groups;

  // Compare joints
  if (result) {
    bool hasJoint1 = false, hasJoint2 = false;
    try {
      rb1.getJoint();
      hasJoint1 = true;
      rb2.getJoint();
      hasJoint2 = true;
    } catch(std::runtime_error re) {
      // either rb1 or rb2 does not have a joint
    }

    if (hasJoint1 && !hasJoint2) {
      PRINT_STMT("LHS link \"" << rb1.linkname << "\" has a joint but RHS link does not!")
      result = false;
    }

    if (!hasJoint1 && hasJoint2) {
      PRINT_STMT("LHS link \"" << rb1.linkname << "\" does not have a joint but RHS link does!")
      result = false;
    }

    if (result && hasJoint1 && hasJoint2) {
      if (rb1.getJoint() != rb2.getJoint()) {
        result = false;
      }
    }
  }

  // Compare contact points
  if (result) {
    try {
      valuecheckMatrix(rb1.contact_pts, rb2.contact_pts, std::numeric_limits<double>::epsilon());
    } catch(std::runtime_error re) {
      PRINT_STMT("Contact points mismatch!" << std::endl
                  << "  - rigid body 1:\n" << rb1.contact_pts << std::endl
                  << "  - rigid body 2:\n" << rb2.contact_pts << std::endl
                  << "  - details:\n" << re.what())
      result = false;
    }
  }

  // Compare mass
  if (result && rb1.mass != rb2.mass) {
    std::cout << "The rigid body masses do not match: "
      << rb1.mass << " vs. " << rb2.mass << std::endl;
    result = false;
  }

  // Compare center of mass
  if (result) {
    try {
      valuecheckMatrix(rb1.com, rb2.com, std::numeric_limits<double>::epsilon());
    } catch(std::runtime_error re) {
      PRINT_STMT("Center of mass mismatch!" << std::endl
                  << "  - rigid body 1:\n" << rb1.com << std::endl
                  << "  - rigid body 2:\n" << rb2.com << std::endl
                  << "  - details:\n" << re.what())
      result = false;
    }
  }

  // Compare inertia matrices
  if (result) {
    try {
      valuecheckMatrix(rb1.I, rb2.I, std::numeric_limits<double>::epsilon());
    } catch(std::runtime_error re) {
      PRINT_STMT("Inertia matrix mismatch!" << std::endl
                  << "  - rigid body 1:\n" << rb1.I << std::endl
                  << "  - rigid body 2:\n" << rb2.I << std::endl
                  << "  - details:\n" << re.what())
      result = false;
    }
  }

  return result;
}

#undef PRINT_STMT

bool operator!=(const RigidBody & rb1, const RigidBody & rb2) {
  return !operator==(rb1, rb2);
}