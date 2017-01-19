#include "drake/multibody/shapes/geometry.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <stdexcept>

#include <tiny_obj_loader.h>

#include "spruce.hh"

using std::ifstream;
using std::istringstream;
using std::ostream;
using std::string;

using Eigen::Matrix3Xd;
using Eigen::Matrix3Xi;
using Eigen::RowVectorXd;
using Eigen::Vector3d;
using Eigen::Vector3i;

namespace DrakeShapes {
const int Geometry::NUM_BBOX_POINTS = 8;
const int Sphere::NUM_POINTS = 1;
const int Capsule::NUM_POINTS = 2;

string ShapeToString(Shape ss) {
  switch (ss) {
    case UNKNOWN:
      return "UNKNOWN";
    case BOX:
      return "BOX";
    case SPHERE:
      return "SPHERE";
    case CYLINDER:
      return "CYLINDER";
    case MESH:
      return "MESH";
    case MESH_POINTS:
      return "MESH_POINTS";
    case CAPSULE:
      return "CAPSULE";
  }
  return "UNDEFINED";
}

Geometry::Geometry() : shape(UNKNOWN) {}

Geometry::Geometry(const Geometry& other) { shape = other.getShape(); }

Geometry::Geometry(Shape shape_in) : shape(shape_in) {}

Shape Geometry::getShape() const { return shape; }

Geometry* Geometry::clone() const { return new Geometry(*this); }

void Geometry::getPoints(Matrix3Xd& points) const { points = Matrix3Xd(); }

void Geometry::getBoundingBoxPoints(Matrix3Xd& points) const {
  points = Matrix3Xd();
}

void Geometry::getBoundingBoxPoints(double x_half_width, double y_half_width,
                                    double z_half_width,
                                    Eigen::Matrix3Xd& points) const {
  // Return axis-aligned bounding-box vertices
  points.resize(3, NUM_BBOX_POINTS);

  RowVectorXd cx(NUM_BBOX_POINTS), cy(NUM_BBOX_POINTS), cz(NUM_BBOX_POINTS);
  cx << -1, 1, 1, -1, -1, 1, 1, -1;
  cy << 1, 1, 1, 1, -1, -1, -1, -1;
  cz << 1, 1, -1, -1, -1, -1, 1, 1;
  cx = cx * x_half_width;
  cy = cy * y_half_width;
  cz = cz * z_half_width;

  points << cx, cy, cz;
}

ostream& operator<<(ostream& out, const Geometry& gg) {
  out << ShapeToString(gg.getShape()) << ", " << gg.NUM_BBOX_POINTS;
  return out;
}

Sphere::Sphere(double radius_in) : Geometry(SPHERE), radius(radius_in) {}

Sphere* Sphere::clone() const { return new Sphere(*this); }

void Sphere::getPoints(Matrix3Xd& points) const {
  points = Matrix3Xd::Zero(3, NUM_POINTS);
}

void Sphere::getBoundingBoxPoints(Matrix3Xd& points) const {
  Geometry::getBoundingBoxPoints(radius, radius, radius, points);
}

void Sphere::getTerrainContactPoints(Matrix3Xd& points) const {
  if (radius < 1e-6)
    getPoints(points);
  else
    points = Matrix3Xd();
}

ostream& operator<<(ostream& out, const Sphere& ss) {
  out << static_cast<const Geometry&>(ss) << ", " << ss.radius << ", "
      << ss.NUM_POINTS;
  return out;
}

Box::Box(const Eigen::Vector3d& size_in) : Geometry(BOX), size(size_in) {}

Box* Box::clone() const { return new Box(*this); }

void Box::getPoints(Matrix3Xd& points) const {
  Geometry::getBoundingBoxPoints(size(0) / 2.0, size(1) / 2.0, size(2) / 2.0,
                                 points);
}

void Box::getBoundingBoxPoints(Matrix3Xd& points) const { getPoints(points); }

void Box::getTerrainContactPoints(Matrix3Xd& points) const {
  getPoints(points);
}

ostream& operator<<(ostream& out, const Box& bb) {
  out << static_cast<const Geometry&>(bb) << ", " << bb.size.transpose();
  return out;
}

Cylinder::Cylinder(double radius_in, double length_in)
    : Geometry(CYLINDER), radius(radius_in), length(length_in) {}

Cylinder* Cylinder::clone() const { return new Cylinder(*this); }

void Cylinder::getPoints(Matrix3Xd& points) const {
  static bool warnOnce = true;
  if (warnOnce) {
    std::cerr
        << "Warning: DrakeShapes::Cylinder::getPoints(): "
           "This method returns the vertices of the cylinder''s bounding-box."
        << std::endl;
    warnOnce = false;
  }

  getBoundingBoxPoints(points);
}

void Cylinder::getBoundingBoxPoints(Matrix3Xd& points) const {
  Geometry::getBoundingBoxPoints(radius, radius, length / 2.0, points);
}

ostream& operator<<(ostream& out, const Cylinder& cc) {
  out << static_cast<const Geometry&>(cc) << ", " << cc.radius << ", "
      << cc.length;
  return out;
}

Capsule::Capsule(double radius_in, double length_in)
    : Geometry(CAPSULE), radius(radius_in), length(length_in) {}

Capsule* Capsule::clone() const { return new Capsule(*this); }

void Capsule::getPoints(Matrix3Xd& points) const {
  // Return segment end-points
  points.resize(Eigen::NoChange, NUM_POINTS);
  RowVectorXd cx = RowVectorXd::Zero(NUM_POINTS);
  RowVectorXd cy = RowVectorXd::Zero(NUM_POINTS);
  RowVectorXd cz = RowVectorXd::Zero(NUM_POINTS);
  cz << length / 2.0, -length / 2.0;

  points << cx, cy, cz;
}

void Capsule::getBoundingBoxPoints(Matrix3Xd& points) const {
  Geometry::getBoundingBoxPoints(radius, radius, (length / 2.0 + radius),
                                 points);
}

ostream& operator<<(ostream& out, const Capsule& cc) {
  out << static_cast<const Geometry&>(cc) << ", " << cc.radius << ", "
      << cc.length;
  return out;
}

Mesh::Mesh(const string& uri, const string& resolved_filename)
    : Geometry(MESH),
      scale_(1.0, 1.0, 1.0),
      uri_(uri),
      resolved_filename_(resolved_filename) {
  if (resolved_filename_.empty()) {
    throw std::runtime_error("Error: The resolved filename provided is empty.");
  }
  // Checks whether:
  // - If file is an obj, if it exists.
  // - If not an obj, if an obj file can be resolved by changing the extension.
  // This throws an exception if an obj file cannot be resolved.
  FindFileWithObjExtension();
}

bool Mesh::extractMeshVertices(Matrix3Xd& vertex_coordinates) const {
  string obj_file_name = FindFileWithObjExtension();
  ifstream file(obj_file_name);
  if (!file) {
    throw std::runtime_error("Error opening file \"" + obj_file_name + "\".");
  }

  string line;
  // Count the number of vertices and resize vertex_coordinates.
  int num_vertices = 0;
  while (getline(file, line)) {
    istringstream iss(line);
    string type;
    if (iss >> type && type == "v") {
      ++num_vertices;
    }
  }
  vertex_coordinates.resize(3, num_vertices);

  file.clear();
  file.seekg(0, file.beg);

  double d;
  int j = 0;
  while (getline(file, line)) {
    istringstream iss(line);
    string type;
    if (iss >> type && type == "v") {
      int i = 0;
      while (iss >> d) {
        vertex_coordinates(i++, j) = d;
      }
      ++j;
    }
  }
  return true;
}

string Mesh::FindFileWithObjExtension() const {
  spruce::path spath(resolved_filename_);
  string ext = spath.extension();
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

  if (ext.compare(".obj") == 0) {
    // Checks if the file with the obj extension exists.
    if (!spath.exists()) {
      throw std::runtime_error(
          "Unable to open file \"" + spath.getStr() + "\".");
    }
  } else {
    // Tries changing the extension to obj.
    spath.setExtension(".obj");
    if (!spath.exists()) {
      throw std::runtime_error(
          "Unable to resolve an obj file from the filename \""
              + spath.getStr() + "\" provided.");
    }
  }

  return spath.getStr();
}

void Mesh::LoadObjFile(PointsVector* vertices,
                       TrianglesVector* triangles) const {
  string obj_file_name = FindFileWithObjExtension();
  ifstream file(obj_file_name);

  if (!file) {
    throw std::runtime_error("Error opening file \"" + obj_file_name + "\".");
  }

  string path;
  size_t idx = obj_file_name.rfind('/');
  if (idx != string::npos) {
    path = obj_file_name.substr(0, idx + 1);
  }

  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string err;

  bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
      obj_file_name.c_str(), path.c_str(), true);

  // Use the boolean return value and the error string to determine
  // if we should proceeed.
  if (!ret || !err.empty()) {
    throw std::runtime_error("Error parsing file \""
        + obj_file_name + "\" : " + err);
  }

  // Store the vertices.
  for (size_t index = 0; index < attrib.vertices.size() - 3; index += 3) {
    vertices->push_back(Vector3d(attrib.vertices[index],
                                 attrib.vertices[index + 1],
                                 attrib.vertices[index + 2]));
  }

  // Iterate over the shapes.
  for (auto const& shape : shapes) {
    unsigned int index_offset = 0;

    // For each face in the shape.
    for (unsigned int face = 0;
         face < shape.mesh.num_face_vertices.size(); ++face) {
      const size_t vert_count = shape.mesh.num_face_vertices[face];

      // Make sure the face has three vertices.
      if (vert_count != 3) {
        throw std::runtime_error(
            "In file \"" + obj_file_name + "\" "
            "Only triangular faces supported. However "
            + std::to_string(vert_count) + " indices are provided.");
      }

      Vector3i face_indices;

      // For each vertex in the face.
      for (size_t vert = 0; vert < vert_count; ++vert) {
        // Store the vertex index.
        face_indices[vert] =
          shape.mesh.indices[index_offset + vert].vertex_index;
      }

      triangles->push_back(face_indices);

      index_offset += vert_count;
    }
  }
}

Mesh* Mesh::clone() const { return new Mesh(*this); }

void Mesh::getPoints(Eigen::Matrix3Xd& point_matrix) const {
  extractMeshVertices(point_matrix);
}

void Mesh::getBoundingBoxPoints(Matrix3Xd& bbox_points) const {
  Matrix3Xd mesh_vertices;
  extractMeshVertices(mesh_vertices);

  Vector3d min_pos = mesh_vertices.rowwise().minCoeff();
  Vector3d max_pos = mesh_vertices.rowwise().maxCoeff();

  bbox_points.resize(Eigen::NoChange, NUM_BBOX_POINTS);
  bbox_points << min_pos(0), min_pos(0), min_pos(0), min_pos(0), max_pos(0),
      max_pos(0), max_pos(0), max_pos(0), min_pos(1), min_pos(1), max_pos(1),
      max_pos(1), min_pos(1), min_pos(1), max_pos(1), max_pos(1), min_pos(2),
      max_pos(2), min_pos(2), max_pos(2), min_pos(2), max_pos(2), min_pos(2),
      max_pos(2);
}

ostream& operator<<(ostream& out, const Mesh& mm) {
  out << static_cast<const Geometry&>(mm) << ", " << mm.scale_ << ", " <<
      mm.uri_ << ", " << mm.resolved_filename_;
  return out;
}

MeshPoints::MeshPoints(const Eigen::Matrix3Xd& points_in)
    : Geometry(MESH_POINTS), points(points_in) {}

MeshPoints* MeshPoints::clone() const { return new MeshPoints(*this); }

void MeshPoints::getPoints(Eigen::Matrix3Xd& point_matrix) const {
  point_matrix = points;
}

void MeshPoints::getBoundingBoxPoints(Matrix3Xd& bbox_points) const {
  Vector3d min_pos = points.rowwise().minCoeff();
  Vector3d max_pos = points.rowwise().maxCoeff();

  bbox_points.resize(Eigen::NoChange, NUM_BBOX_POINTS);
  bbox_points << min_pos(0), min_pos(0), min_pos(0), min_pos(0), max_pos(0),
      max_pos(0), max_pos(0), max_pos(0), min_pos(1), min_pos(1), max_pos(1),
      max_pos(1), min_pos(1), min_pos(1), max_pos(1), max_pos(1), min_pos(2),
      max_pos(2), min_pos(2), max_pos(2), min_pos(2), max_pos(2), min_pos(2),
      max_pos(2);
}

ostream& operator<<(ostream& out, const MeshPoints& mp) {
  out << static_cast<const Geometry&>(mp) << ",\n" << mp.points;
  return out;
}

}  // namespace DrakeShapes
