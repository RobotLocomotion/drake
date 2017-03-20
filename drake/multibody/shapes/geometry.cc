#include "drake/multibody/shapes/geometry.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <stdexcept>

#include "spruce.hh"

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

using std::string;
using std::ostream;
using std::istringstream;
using std::ifstream;

using Eigen::Vector3i;
using Eigen::Vector3d;
using Eigen::RowVectorXd;
using Eigen::Matrix3Xd;
using Eigen::Matrix3Xi;

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

  // Order:                +y
  //       3------2          |
  //      /|     /|          |
  //     / 4----/-5          ------  +x
  //    0------1 /          /
  //    |/     |/          /
  //    7------6        +z

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

void Box::getFaces(TrianglesVector* faces) const {
  faces->resize(12);
  // Here, the vertex indices index into the getBoundingBox vertex order.
  // (See the documentation in that function for a picture.) For each face,
  // vertices are supplied in counterclockwise order when viewed from
  // the "outside" of the face.
  // +y face:
  faces->at(0) = Vector3i(0, 1, 2);
  faces->at(1) = Vector3i(0, 2, 3);
  // +x face:
  faces->at(2) = Vector3i(5, 2, 1);
  faces->at(3) = Vector3i(6, 5, 1);
  // +z face:
  faces->at(4) = Vector3i(7, 6, 1);
  faces->at(5) = Vector3i(7, 1, 0);
  // -y face:
  faces->at(6) = Vector3i(4, 5, 6);
  faces->at(7) = Vector3i(4, 6, 7);
  // -x face:
  faces->at(8) = Vector3i(3, 4, 7);
  faces->at(9) = Vector3i(0, 3, 7);
  // -z face:
  faces->at(10) = Vector3i(5, 4, 2);
  faces->at(11) = Vector3i(4, 3, 2);
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

const double Mesh::kCosThreshold = std::cos(30.0 / 180.0 * M_PI);

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
        vertex_coordinates(i, j) = d * scale_(i);
        i++;
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

void Mesh::LoadObjFile(PointsVector* vertices, TrianglesVector* triangles,
                       TriangulatePolicy triangulate) const {
  string obj_file_name = FindFileWithObjExtension();
  ifstream file(obj_file_name);
  if (!file) {
    throw std::runtime_error("Error opening file \"" + obj_file_name + "\".");
  }

  std::string line;
  int line_number = 0;
  int maximum_index = 0;
  int added_triangles = 0;
  while (!file.eof()) {
    ++line_number;
    std::getline(file, line);
    std::stringstream ss(line);
    std::string key;
    ss >> key;

    if (key == "v") {
      // Reads a 3D vertex.
      double x, y, z;
      ss >> x; ss >> y; ss >> z;
      if (ss.fail()) {
        throw std::runtime_error(
            "In file \"" + obj_file_name + "\" "
            "(line " + std::to_string(line_number) + "). "
            "Vertex in the wrong format.");
      }
      vertices->push_back(Vector3d(x * scale_[0],
                                   y * scale_[1],
                                   z * scale_[2]));
    } else if (key == "f") {
      // Reads the connectivity for a single triangle.
      std::vector<int> indices;
      int index;
      while (ss >> index) {
        // Checks that index >= 1.
        if (index < 1) {
          throw std::runtime_error(
              "In file \"" + obj_file_name + "\" "
              "(line " + std::to_string(line_number) + "). "
              "Invalid vertex index is " + std::to_string(index) + " < 1.");
        }
        maximum_index = std::max(maximum_index, index);

        // Ignores line until the next whitespace.
        // This effectively ignores texture coordinates and normals.
        // The first entry always corresponds to an index in the face.
        ss.ignore(line.size(), ' ');
        if (ss.fail()) {
          throw std::runtime_error(
              "In file \"" + obj_file_name + "\" "
              "(line " + std::to_string(line_number) + "). "
              "Triangle face in the wrong format.");
        }
        indices.push_back(index);
      }
      if (indices.size() != 3) {
        if (triangulate == TriangulatePolicy::kTry && indices.size() > 3) {
          // This is a very naive triangulation.  It simply creates a fan
          // around the 0th vertex through the loop of vertices in the polygon.
          // The fan won't necessarily produce triangles with the best aspect
          // ratio.
          //
          // The requirement that two adjacent, generated triangles have normals
          // that deviate by no more than 30° will implicitly catch the
          // degenerate case where applying this algorithm to a concave face
          // would cause changes in triangle winding and, therefore, normal
          // directions.
          //
          // For a polygon with n vertices indexed in the range [0, n - 1] we
          // create triangles built on those indices in the pattern:
          //    (0, 1, 2), (0, 2, 3), (0, 3, 4), ..., (0, n-2, n-1).
          // The triangle consisting of (0, 1, 2) is handled below, so this
          // starts with (0, 2, 3).
          Vector3d lastNormal;
          bool valid = GetNormal(*vertices, indices[0] - 1, indices[1] - 1,
                                 indices[2] - 1, &lastNormal, kMinArea);
          const int index_size = static_cast<int>(indices.size());
          for (int i = 2; valid && i < index_size - 1; ++i) {
            // OBJ file indices are 1-based; subtracting 1 makes them 0-based.
            Vector3d testNormal;
            valid = GetNormal(*vertices, indices[0] - 1, indices[i] - 1,
                              indices[i + 1] - 1, &testNormal, kMinArea);
            if (valid) {
              double dot_product = lastNormal.dot(testNormal);
              if (dot_product < 0) {
                throw std::runtime_error("Trying to triangulate the face in '" +
                    obj_file_name + "' on line " +
                    std::to_string(line_number) +
                    " led to bad triangles. The triangle based " +
                    "on vertices " + std::to_string(indices[0]) +
                    ", " + std::to_string(indices[i]) + ", and " +
                    std::to_string(indices[i + 1]) +
                    " (1-indexed) is wound in the opposite " +
                    "direction from the previous triangle. " +
                    "Consider triangulating by hand.");
              } else if (dot_product < kCosThreshold) {
                throw std::runtime_error("Trying to triangulate the face in '" +
                    obj_file_name + "' on line " +
                    std::to_string(line_number) +
                    ".  The face is not sufficiently planar. " +
                    "Consider triangulating by hand.");
              }
            }
            lastNormal = testNormal;
            triangles->push_back(
                Vector3i(indices[0] - 1, indices[i] - 1, indices[i + 1] - 1));
          }
          if (!valid) {
            // Problems in computing the normal are logged in GetNormal.
            throw std::runtime_error("Unable to triangulate face in file '" +
                obj_file_name + " on line " + std::to_string(line_number) +
                ". See log for details.");
          }
          // A polygon of n vertices produces a fan of n - 2 triangles.
          // One of those is a given, so we're "adding" n - 3 triangles.
          added_triangles += index_size - 3;
        } else {
          throw std::runtime_error("In file \"" + obj_file_name +
                                   "\" (line " +
                                   std::to_string(line_number) +
                                   "). Only triangular faces are supported. " +
                                   "However " +
                                   std::to_string(indices.size()) +
                                   " indices are provided.");
        }
      }
      triangles->push_back(Vector3i(indices[0]-1, indices[1]-1, indices[2]-1));
    }
  }

  // Verifies that the maximum index referenced when defining faces does not
  // exceed the number of vertices.
  if (maximum_index > static_cast<int>(vertices->size())) {
    throw std::runtime_error(
        "In file \"" + obj_file_name + "\". "
        "The maximum index referenced in defining faces (" +
            std::to_string(maximum_index) + ") "
        "exceeds the number of vertices (" +
            std::to_string(vertices->size()) + ". ");
  }

  if (added_triangles > 0) {
    drake::log()->info("Encountered non-triangular faces in '" + obj_file_name +
                       "'. Triangulation was applied, adding " +
                       std::to_string(added_triangles) +
                       " new triangles to the mesh.");
  }
}

Mesh* Mesh::clone() const { return new Mesh(*this); }

void Mesh::getPoints(Eigen::Matrix3Xd& point_matrix) const {
  extractMeshVertices(point_matrix);
}

void Mesh::getFaces(TrianglesVector* faces) const {
  PointsVector points;
  LoadObjFile(&points, faces, Mesh::TriangulatePolicy::kTry);
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

bool Mesh::GetNormal(const PointsVector& vertices, int i0, int i1, int i2,
                     Eigen::Vector3d* normal, double minArea) {
  DRAKE_ASSERT(normal != nullptr);
  DRAKE_ASSERT(i0 >= 0 && i1 >= 0 && i2 >= 0);
  const int kNumVertex = static_cast<int>(vertices.size());
  if (i0 >= kNumVertex || i1 >= kNumVertex || i2 >= kNumVertex) {
    drake::log()->warn(
        "Unable to compute normal. At least one OBJ index reference: " +
        std::to_string(i0 + 1) + ", " + std::to_string(i1 + 1) + ", " +
        std::to_string(i2 + 1) + " is beyond the parsed number of vertices: " +
        std::to_string(kNumVertex) + ". Make sure the OBJ file defines the " +
        "vertices before referencing them.");
    return false;
  }
  const Vector3d& v0 = vertices[i0];
  const Vector3d& v1 = vertices[i1];
  const Vector3d& v2 = vertices[i2];
  *normal = (v1 - v0).cross(v2 - v0);

  // The magnitude of this normal is *twice* the area of the triangle formed by
  // v0, v1, and v2.  The normal is only considered valid if the area is at
  // at least that specified by the caller.
  double length = normal->norm();
  if (length < 2 * minArea) {
    drake::log()->warn(
        "Unable to compute normal. Triangle with OBJ indices: " +
            std::to_string(i0 + 1) + ", " + std::to_string(i1 + 1) + ", " +
            std::to_string(i2 + 1) + " is too small.");
    return false;
  }
  *normal /= length;
  return true;
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
