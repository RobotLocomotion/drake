#include "drake/manipulation/dev/remote_tree_viewer_wrapper.h"

#include <unistd.h>

#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

#include "nlohmann/json.hpp"

#include "drake/lcmt_viewer2_comms.hpp"

using nlohmann::json;

namespace drake {
namespace manipulation {
namespace dev {
static double getUnixTime(void) {
  struct timespec tv;

  if (clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

  return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

RemoteTreeViewerWrapper::RemoteTreeViewerWrapper() {}

void RemoteTreeViewerWrapper::PublishPointCloud(
    const Eigen::Matrix3Xd& pts, const std::vector<std::string>& path,
    const std::vector<std::vector<double>>& color) {
  int64_t now = getUnixTime() * 1000 * 1000;
  // Format a JSON string for this pointcloud
  json j = {
      {"timestamp", now},
      {
          "setgeometry",
          {{{"path", path},
            {"geometry",
             {{"type", "pointcloud"},
              {"points", std::vector<std::vector<double>>()},
              {"channels", {{"rgb", std::vector<std::vector<double>>()}}}}}}},
      },
      {"settransform", json({})},
      {"delete", json({})}};

  // Push in the points and colors.
  for (int i = 0; i < pts.cols(); i++) {
    j["setgeometry"][0]["geometry"]["points"].push_back(
        {pts(0, i), pts(1, i), pts(2, i)});
    if (color.size() == 1)
      j["setgeometry"][0]["geometry"]["channels"]["rgb"].push_back(color[0]);
    else if (static_cast<int>(color.size()) == pts.cols())
      j["setgeometry"][0]["geometry"]["channels"]["rgb"].push_back(color[i]);
    else if (color.size() != 0)
      printf("Color does not have right number of entries: %ld vs %ld\n",
             color.size(), pts.cols());
  }

  auto msg = lcmt_viewer2_comms();
  msg.utime = now;
  msg.format = "treeviewer_json";
  msg.format_version_major = 1;
  msg.format_version_minor = 0;
  msg.data.clear();
  for (auto& c : j.dump()) msg.data.push_back(c);
  msg.num_bytes = j.dump().size();
  // Use channel 0 for remote viewer communications.
  lcm_.get_lcm_instance()->publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}

void RemoteTreeViewerWrapper::PublishLine(
    const Eigen::Matrix3Xd& pts, const std::vector<std::string>& path) {
  int64_t now = getUnixTime() * 1000 * 1000;
  // Format a JSON string for this pointcloud
  json j = {{"timestamp", now},
            {
                "setgeometry",
                {{{"path", path},
                  {"geometry",
                   {
                       {"type", "line"},
                       {"points", std::vector<std::vector<double>>()},
                   }}}},
            },
            {"settransform", json({})},
            {"delete", json({})}};

  // Push in the points and colors.
  for (int i = 0; i < pts.cols(); i++) {
    j["setgeometry"][0]["geometry"]["points"].push_back(
        {pts(0, i), pts(1, i), pts(2, i)});
  }

  auto msg = lcmt_viewer2_comms();
  msg.utime = now;
  msg.format = "treeviewer_json";
  msg.format_version_major = 1;
  msg.format_version_minor = 0;
  msg.data.clear();
  for (auto& c : j.dump()) msg.data.push_back(c);
  msg.num_bytes = j.dump().size();
  // Use channel 0 for remote viewer communications.
  lcm_.get_lcm_instance()->publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}

void RemoteTreeViewerWrapper::PublishArrow(
    const Eigen::Ref<const Eigen::Vector3d>& start,
    const Eigen::Ref<const Eigen::Vector3d>& end,
    const std::vector<std::string>& path, double radius, double head_radius,
    double head_length, const Eigen::Ref<const Eigen::Vector4d>& color) {
  int64_t now = getUnixTime() * 1000 * 1000;
  // Format a JSON string for this pointcloud
  json j = {{"timestamp", now},
            {
                "setgeometry",
                {{{"path", path},
                  {"geometry",
                   {
                       {"type", "line"},
                       {"points", std::array<std::array<double, 3>, 2>()},
                       {"color", std::array<double, 4>()},
                   }}}},
            },
            {"settransform", json({})},
            {"delete", json({})}};

  // Push in the points and colors.
  j["setgeometry"][0]["geometry"]["points"][0] = {start(0), start(1), start(2)};
  j["setgeometry"][0]["geometry"]["points"][1] = {end(0), end(1), end(2)};
  j["setgeometry"][0]["geometry"]["radius"] = radius;
  j["setgeometry"][0]["geometry"]["end_head"] = true;
  j["setgeometry"][0]["geometry"]["head_radius"] = head_radius;
  j["setgeometry"][0]["geometry"]["head_length"] = head_length;
  j["setgeometry"][0]["geometry"]["color"] = {color(0), color(1), color(2),
                                              color(3)};

  auto msg = lcmt_viewer2_comms();
  msg.utime = now;
  msg.format = "treeviewer_json";
  msg.format_version_major = 1;
  msg.format_version_minor = 0;
  msg.data.clear();
  for (auto& c : j.dump()) msg.data.push_back(c);
  msg.num_bytes = j.dump().size();
  // Use channel 0 for remote viewer communications.
  lcm_.get_lcm_instance()->publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}
void RemoteTreeViewerWrapper::PublishRawMesh(
    const Eigen::Matrix3Xd& verts, const std::vector<Eigen::Vector3i>& tris,
    const std::vector<std::string>& path) {
  int64_t now = getUnixTime() * 1000 * 1000;
  json j = {{"timestamp", now},
            {
                "setgeometry",
                {{{"path", path},
                  {"geometry",
                   {{"type", "mesh_data"},
                    {"vertices", std::vector<std::vector<double>>()},
                    {"faces", std::vector<std::vector<int>>()}}}}},
            },
            {"settransform", json({})},
            {"delete", json({})}};

  // Push in the points.
  for (int i = 0; i < verts.cols(); i++) {
    j["setgeometry"][0]["geometry"]["vertices"].push_back(
        {verts(0, i), verts(1, i), verts(2, i)});
  }
  // Push in the triangle indices.
  for (int i = 0; i < static_cast<int>(tris.size()); i++) {
    j["setgeometry"][0]["geometry"]["faces"].push_back(
        {tris[i][0], tris[i][1], tris[i][2]});
  }

  auto msg = lcmt_viewer2_comms();
  msg.utime = now;
  msg.format = "treeviewer_json";
  msg.format_version_major = 1;
  msg.format_version_minor = 0;
  msg.data.clear();
  for (auto& c : j.dump()) msg.data.push_back(c);
  msg.num_bytes = j.dump().size();
  // Use channel 0 for remote viewer communications.
  lcm_.get_lcm_instance()->publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}

void RemoteTreeViewerWrapper::PublishRigidBodyTree(
    const RigidBodyTree<double>& tree, const Eigen::VectorXd& q,
    const Eigen::Vector4d& color, const std::vector<std::string>& path,
    bool visual) {
  auto kinematics_cache = tree.doKinematics(q);
  for (const auto& body : tree.get_bodies()) {
    if (visual) {
      for (const auto& element : body->get_visual_elements()) {
        if (element.hasGeometry()) {
          std::vector<std::string> full_name = path;
          full_name.push_back(body->get_name());
          PublishGeometry(element.getGeometry(),
                          tree.relativeTransform(kinematics_cache, 0,
                                                 body->get_body_index()) *
                              element.getLocalTransform(),
                          color, full_name);
        }
      }
    } else {
      for (const auto& collision_elem_id : body->get_collision_element_ids()) {
        auto element = tree.FindCollisionElement(collision_elem_id);
        if (element->hasGeometry()) {
          std::vector<std::string> full_name = path;
          full_name.push_back(body->get_name());
          PublishGeometry(element->getGeometry(),
                          tree.relativeTransform(kinematics_cache, 0,
                                                 body->get_body_index()) *
                              element->getLocalTransform(),
                          color, full_name);
        }
      }
    }
  }
}

void RemoteTreeViewerWrapper::PublishRigidBody(
    const RigidBody<double>& body, const Eigen::Affine3d& tf,
    const Eigen::Vector4d& color, const std::vector<std::string>& path) {}

void RemoteTreeViewerWrapper::PublishGeometry(
    const DrakeShapes::Geometry& geometry, const Eigen::Affine3d& tf,
    const Eigen::Vector4d& color, const std::vector<std::string>& path) {
  int64_t now = getUnixTime() * 1000 * 1000;

  // Short-circuit to points if the passed geometry is a set of mesh points
  if (geometry.getShape() == DrakeShapes::MESH_POINTS) {
    Eigen::Matrix3Xd pts;
    geometry.getPoints(pts);
    PublishPointCloud(pts, path);
  }

  // Extract std::vector-formatted translation and quaternion from the tf
  std::vector<double> translation = {tf.translation()[0], tf.translation()[1],
                                     tf.translation()[2]};
  Eigen::Quaterniond q(tf.rotation());
  std::vector<double> rotation = {q.w(), q.x(), q.y(), q.z()};

  // Format a JSON string for this bit of geometry (in a setgeometry call)
  // and its transformation (in a settransform call)
  json j = {{"timestamp", now},
            {
                "setgeometry",
                {{{"path", path},
                  {"geometry",
                   {{"color", {color[0], color[1], color[2], color[3]}}}}}},
            },
            {"settransform",
             {{{"path", path},
               {"transform",
                {{"translation", translation}, {"quaternion", rotation}}}}}},
            {"delete", json({})}};

  // Fill in the setgeometry call based on what kind of geometry this is
  switch (geometry.getShape()) {
    case DrakeShapes::BOX: {
      const DrakeShapes::Box* box =
          static_cast<const DrakeShapes::Box*>(&geometry);
      j["setgeometry"][0]["geometry"]["type"] = std::string("box");
      j["setgeometry"][0]["geometry"]["lengths"] =
          std::vector<double>({box->size[0], box->size[1], box->size[2]});
    } break;
    case DrakeShapes::SPHERE: {
      const DrakeShapes::Sphere* sphere =
          static_cast<const DrakeShapes::Sphere*>(&geometry);
      j["setgeometry"][0]["geometry"]["type"] = std::string("sphere");
      j["setgeometry"][0]["geometry"]["radius"] = sphere->radius;
    } break;
    case DrakeShapes::CYLINDER: {
      const DrakeShapes::Cylinder* cylinder =
          static_cast<const DrakeShapes::Cylinder*>(&geometry);
      j["setgeometry"][0]["geometry"]["type"] = std::string("cylinder");
      j["setgeometry"][0]["geometry"]["radius"] = cylinder->radius;
      j["setgeometry"][0]["geometry"]["length"] = cylinder->length;
    } break;
    case DrakeShapes::MESH: {
      const DrakeShapes::Mesh* mesh =
          static_cast<const DrakeShapes::Mesh*>(&geometry);
      j["setgeometry"][0]["geometry"]["type"] = std::string("mesh_file");
      j["setgeometry"][0]["geometry"]["filename"] = mesh->resolved_filename_;
      j["setgeometry"][0]["geometry"]["scale"] = std::vector<double>(
          {mesh->scale_[0], mesh->scale_[1], mesh->scale_[2]});
    } break;
    case DrakeShapes::CAPSULE: {
      const DrakeShapes::Capsule* capsule =
          static_cast<const DrakeShapes::Capsule*>(&geometry);
      j["setgeometry"][0]["geometry"]["type"] = std::string("capsule");
      j["setgeometry"][0]["geometry"]["radius"] = capsule->radius;
      j["setgeometry"][0]["geometry"]["length"] = capsule->length;
    } break;
    default:
      std::cout << "Unsupported geometry type " << geometry.getShape()
                << ", sorry!" << std::endl;
      return;
  }

  auto msg = lcmt_viewer2_comms();
  msg.utime = now;
  msg.format = "treeviewer_json";
  msg.format_version_major = 1;
  msg.format_version_minor = 0;
  msg.data.clear();
  for (auto& c : j.dump()) msg.data.push_back(c);
  msg.num_bytes = j.dump().size();
  // Use channel 0 for remote viewer communications.
  lcm_.get_lcm_instance()->publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}
}  // namespace dev
}  // namespace manipulation
}  // namespace drake
