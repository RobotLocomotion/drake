
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
  std::string parentJointName =
      b.hasParent() ? b.getJoint().getName() : "no parent joint";

  std::stringstream ceMsg;
  ceMsg << "[";
  for (size_t ii = 0; ii< b.collision_element_ids.size(); ii++) {
    ceMsg << b.collision_element_ids[ii];
    if (ii < b.collision_element_ids.size() - 1)
      ceMsg << ", ";
  }
  ceMsg << "]";

  out << "RigidBody\n"
      << "  - link name: " << b.linkname << "\n"
      << "  - parent joint: " << parentJointName << "\n"
      << "  - Collision elements IDs: " << ceMsg.str();

  return out;
}

bool RigidBody::Compare(const RigidBody & rb, std::string * explanation) const {
  bool result = true;

  if (linkname.compare(rb.linkname) != 0) {
    if (explanation != nullptr) {
      std::stringstream ss;
      ss << "Link names do not match "
         << linkname << " vs. " << rb.linkname;
      *explanation = ss.str();
    }
    result = false;
  }

  if (result && hasParent() && !rb.hasParent()) {
    if (explanation != nullptr) {
      std::stringstream ss;
      ss << "LHS rigid body has parent whereas RHS rigid body does not. Linkname is "
         << linkname << ".";
      *explanation = ss.str();
    }
    result = false;
  }

  if (result && !hasParent() && rb.hasParent()) {
    if (explanation != nullptr) {
      std::stringstream ss;
      ss << "LHS rigid body does not have parent whereas RHS rigid body does. Linkname is "
         << linkname << ".";
      *explanation = ss.str();
    }
    result = false;
  }

  if (result && hasParent() && rb.hasParent()) {
      if (parent->linkname.compare(rb.parent->linkname) != 0) {
        if (explanation != nullptr) {
          std::stringstream ss;
          ss << "Link " << linkname << " have different parent links ("
            << parent->linkname << " vs. " << rb.parent->linkname << ".";
          *explanation = ss.str();
        }
        result = false;
      }
  }

  if (result && body_index != rb.body_index) {
    if (explanation != nullptr) {
      std::stringstream ss;
      ss << "Body indices do not match: "
        << body_index << " vs. " << rb.body_index;
      *explanation = ss.str();
    }
    result = false;
  }

  if (result && position_num_start != rb.position_num_start) {
    if (explanation != nullptr) {
      std::stringstream ss;
      ss << "Variable position_num_start does not match: "
        << position_num_start << " vs. " << rb.position_num_start;
      *explanation = ss.str();
    }
    result = false;
  }

  if (result && velocity_num_start != rb.velocity_num_start) {
    if (explanation != nullptr) {
      std::stringstream ss;
      ss << "Variable velocity_num_start does not match: "
         << velocity_num_start << " vs. " << rb.velocity_num_start;
      *explanation = ss.str();
    }
    result = false;
  }

  // Compare joints
  if (result) {
    bool hasJoint1 = false, hasJoint2 = false;
    try {
      getJoint();
      hasJoint1 = true;
      rb.getJoint();
      hasJoint2 = true;
    } catch(std::runtime_error re) {
      // either rb1 or rb does not have a joint
    }

    if (hasJoint1 && !hasJoint2) {
      if (explanation != nullptr) {
        std::stringstream ss;
        ss << "LHS link \"" << linkname << "\" has a joint but RHS link does not!";
        *explanation = ss.str();
      }
      result = false;
    }

    if (!hasJoint1 && hasJoint2) {
      if (explanation != nullptr) {
        std::stringstream ss;
        ss << "LHS link \"" << linkname << "\" does not have a joint but RHS link does!";
        *explanation = ss.str();
      }
      result = false;
    }

    if (result && hasJoint1 && hasJoint2) {
      if (!getJoint().Compare(rb.getJoint(), explanation)) {
        result = false;
      }
    }
  }

  // Compare contact points
  if (result) {
    try {
      valuecheckMatrix(contact_pts, rb.contact_pts, std::numeric_limits<double>::epsilon());
    } catch(std::runtime_error re) {
      if (explanation != nullptr) {
        std::stringstream ss;
        ss << "Contact points mismatch!" << std::endl
           << "  - rigid body 1:\n" << contact_pts << std::endl
           << "  - rigid body 2:\n" << rb.contact_pts << std::endl
           << "  - details:\n" << re.what();
        *explanation = ss.str();
      }
      result = false;
    }
  }

  // Compare mass
  if (result && mass != rb.mass) {
    if (explanation != nullptr) {
      std::stringstream ss;
      ss << "The rigid body masses do not match: "
         << mass << " vs. " << rb.mass;
      *explanation = ss.str();
    }
    result = false;
  }

  // Compare center of mass
  if (result) {
    try {
      valuecheckMatrix(com, rb.com, std::numeric_limits<double>::epsilon());
    } catch(std::runtime_error re) {
      if (explanation != nullptr) {
        std::stringstream ss;
        ss << "Center of mass mismatch!" << std::endl
           << "  - rigid body 1:\n" << com << std::endl
           << "  - rigid body 2:\n" << rb.com << std::endl
           << "  - details:\n" << re.what();
        *explanation = ss.str();
      }
      result = false;
    }
  }

  // Compare inertia matrices
  if (result) {
    try {
      valuecheckMatrix(I, rb.I, std::numeric_limits<double>::epsilon());
    } catch(std::runtime_error re) {
      if (explanation != nullptr) {
        std::stringstream ss;
        ss << "Inertia matrix mismatch!" << std::endl
           << "  - rigid body 1:\n" << I << std::endl
           << "  - rigid body 2:\n" << rb.I << std::endl
           << "  - details:\n" << re.what();
        *explanation = ss.str();
      }
      result = false;
    }
  }

  return result;
}

bool operator==(const RigidBody & rb1, const RigidBody & rb2) {
  return rb1.Compare(rb2);
}

bool operator!=(const RigidBody & rb1, const RigidBody & rb2) {
  return !operator==(rb1, rb2);
}