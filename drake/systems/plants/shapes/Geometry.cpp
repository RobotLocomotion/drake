#include <fstream>
#include <stdexcept>

#include "Geometry.h"
#include "spruce.hh"

using std::string;
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

std::string ShapeToString(Shape ss) {
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

Geometry::Geometry(const Geometry &other) { shape = other.getShape(); }

Geometry::Geometry(Shape shape) : shape(shape) {}

Shape Geometry::getShape() const { return shape; }

Geometry *Geometry::clone() const { return new Geometry(*this); }

void Geometry::getPoints(Matrix3Xd &points) const { points = Matrix3Xd(); }

void Geometry::getBoundingBoxPoints(Matrix3Xd &points) const {
  points = Matrix3Xd();
}

void Geometry::getBoundingBoxPoints(double x_half_width, double y_half_width,
                                    double z_half_width,
                                    Eigen::Matrix3Xd &points) const {
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

std::ostream &operator<<(std::ostream &out, const Geometry &gg) {
  out << ShapeToString(gg.getShape()) << ", " << gg.NUM_BBOX_POINTS;
  return out;
}

Sphere::Sphere(double radius) : Geometry(SPHERE), radius(radius) {}

Sphere *Sphere::clone() const { return new Sphere(*this); }

void Sphere::getPoints(Matrix3Xd &points) const {
  points = Matrix3Xd::Zero(3, NUM_POINTS);
}

void Sphere::getBoundingBoxPoints(Matrix3Xd &points) const {
  Geometry::getBoundingBoxPoints(radius, radius, radius, points);
}

void Sphere::getTerrainContactPoints(Matrix3Xd &points) const {
  if (radius < 1e-6)
    getPoints(points);
  else
    points = Matrix3Xd();
}

std::ostream &operator<<(std::ostream &out, const Sphere &ss) {
  out << static_cast<const Geometry &>(ss) << ", " << ss.radius << ", "
      << ss.NUM_POINTS;
  return out;
}

Box::Box(const Eigen::Vector3d &size) : Geometry(BOX), size(size) {}

Box *Box::clone() const { return new Box(*this); }

void Box::getPoints(Matrix3Xd &points) const {
  Geometry::getBoundingBoxPoints(size(0) / 2.0, size(1) / 2.0, size(2) / 2.0,
                                 points);
}

void Box::getBoundingBoxPoints(Matrix3Xd &points) const { getPoints(points); }

void Box::getTerrainContactPoints(Matrix3Xd &points) const {
  getPoints(points);
}

std::ostream &operator<<(std::ostream &out, const Box &bb) {
  out << static_cast<const Geometry &>(bb) << ", " << bb.size.transpose();
  return out;
}

Cylinder::Cylinder(double radius, double length)
    : Geometry(CYLINDER), radius(radius), length(length) {}

Cylinder *Cylinder::clone() const { return new Cylinder(*this); }

void Cylinder::getPoints(Matrix3Xd &points) const {
  static bool warnOnce = true;
  if (warnOnce) {
    std::cerr << "Warning: DrakeShapes::Cylinder::getPoints(): """
        "This method returns the vertices of the cylinder''s bounding-box."
         << std::endl;
    warnOnce = false;
  }

  getBoundingBoxPoints(points);
}

void Cylinder::getBoundingBoxPoints(Matrix3Xd &points) const {
  Geometry::getBoundingBoxPoints(radius, radius, length / 2.0, points);
}

std::ostream &operator<<(std::ostream &out, const Cylinder &cc) {
  out << static_cast<const Geometry &>(cc) << ", " << cc.radius << ", "
      << cc.length;
  return out;
}

Capsule::Capsule(double radius, double length)
    : Geometry(CAPSULE), radius(radius), length(length) {}

Capsule *Capsule::clone() const { return new Capsule(*this); }

void Capsule::getPoints(Matrix3Xd &points) const {
  // Return segment end-points
  points.resize(Eigen::NoChange, NUM_POINTS);
  RowVectorXd cx = RowVectorXd::Zero(NUM_POINTS);
  RowVectorXd cy = RowVectorXd::Zero(NUM_POINTS);
  RowVectorXd cz = RowVectorXd::Zero(NUM_POINTS);
  cz << length / 2.0, -length / 2.0;

  points << cx, cy, cz;
}

void Capsule::getBoundingBoxPoints(Matrix3Xd &points) const {
  Geometry::getBoundingBoxPoints(radius, radius, (length / 2.0 + radius),
                                 points);
}

std::ostream &operator<<(std::ostream &out, const Capsule &cc) {
  out << static_cast<const Geometry &>(cc) << ", " << cc.radius << ", "
      << cc.length;
  return out;
}

Mesh::Mesh(const string &filename)
    : Geometry(MESH), scale(1.0, 1.0, 1.0), filename(filename) {}

Mesh::Mesh(const string &filename, const string &resolved_filename)
    : Geometry(MESH),
      scale(1.0, 1.0, 1.0),
      filename(filename),
      resolved_filename(resolved_filename) {}

bool Mesh::extractMeshVertices(Matrix3Xd &vertex_coordinates) const {
  if (resolved_filename.empty()) {
    return false;
  }
  spruce::path spath(resolved_filename);
  string ext = spath.extension();
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

  ifstream file;
  if (ext.compare(".obj") == 0) {
    file.open(spath.getStr().c_str(), ifstream::in);
  } else {
    spath.setExtension(".obj");

    if (spath.exists()) {
      // try changing the extension to obj and loading.
      file.open(spath.getStr().c_str(), ifstream::in);
    }
  }

  if (!file.is_open()) {
    std::cerr << "Warning: Mesh " << spath.getStr()
         << " ignored because it does not have extension .obj (nor can I find "
            "a juxtaposed file with a .obj extension)"
         << std::endl;
    return false;
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

bool Mesh::ReadMeshConnectivities(Matrix3Xi& connectivities) const {
  if (resolved_filename.empty()) return false;

  spruce::path spath(resolved_filename);
  string ext = spath.extension();
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

  FILE* file;
  if (ext.compare(".obj") == 0) {
    file = fopen(spath.getStr().c_str(),"r");
  } else {
    spath.setExtension(".obj");
    if (spath.exists()) {
      // try changing the extension to obj and loading.
      file = fopen(spath.getStr().c_str(),"r");
    }
  }

  if (!file) {
    throw std::logic_error(
        "Could not open mesh file \""+spath.getStr() + "\".");
  }

  // Count the number of triangles and resize connectivities.
  int num_triangles = 0;
  char *line = NULL;
  size_t len = 0;
  ssize_t read;
  char key[128];
  while ((read = getline(&line, &len, file)) != -1) {
    sscanf(line,"%s",key);
    if (strcmp(key, "f") == 0) ++num_triangles;
  }

  // Allocate memory.
  connectivities.resize(3, num_triangles);

  // Read triangles.
  rewind(file);
  int itri = 0;
  int ignored_entry;
  int tri[3];
  while(true) {
    // Get first word in the line.
    if(fscanf(file, "%s", key) == EOF) break;
    if (strcmp(key, "f") == 0) {
      int matches = fscanf(file,
                           "%d//%d %d//%d %d//%d\n",
                           tri + 0,
                           &ignored_entry,
                           tri + 1,
                           &ignored_entry,
                           tri + 2,
                           &ignored_entry);
      if(matches != 6)
        throw std::logic_error(
            "File \""+filename+"\" cannot be parsed. Format not supported.");
      connectivities.col(itri++) = Vector3i(tri[0]-1,tri[1]-1,tri[2]-1);
      if(itri>num_triangles)
        throw(std::logic_error("Number of triangles exceeded previous count."));
    }
  } // while
  fclose(file);

  return true;
}

Mesh *Mesh::clone() const { return new Mesh(*this); }

void Mesh::getPoints(Eigen::Matrix3Xd &point_matrix) const {
  extractMeshVertices(point_matrix);
}

void Mesh::getBoundingBoxPoints(Matrix3Xd &bbox_points) const {
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

std::ostream &operator<<(std::ostream &out, const Mesh &mm) {
  out << static_cast<const Geometry &>(mm) << ", " << mm.scale << ", "
      << mm.filename << ", " << mm.resolved_filename << ", " << mm.root_dir;
  return out;
}

MeshPoints::MeshPoints(const Eigen::Matrix3Xd &points)
    : Geometry(MESH_POINTS), points(points) {}

MeshPoints *MeshPoints::clone() const { return new MeshPoints(*this); }

void MeshPoints::getPoints(Eigen::Matrix3Xd &point_matrix) const {
  point_matrix = points;
}

void MeshPoints::getBoundingBoxPoints(Matrix3Xd &bbox_points) const {
  Vector3d min_pos = points.rowwise().minCoeff();
  Vector3d max_pos = points.rowwise().maxCoeff();

  bbox_points.resize(Eigen::NoChange, NUM_BBOX_POINTS);
  bbox_points << min_pos(0), min_pos(0), min_pos(0), min_pos(0), max_pos(0),
      max_pos(0), max_pos(0), max_pos(0), min_pos(1), min_pos(1), max_pos(1),
      max_pos(1), min_pos(1), min_pos(1), max_pos(1), max_pos(1), min_pos(2),
      max_pos(2), min_pos(2), max_pos(2), min_pos(2), max_pos(2), min_pos(2),
      max_pos(2);
}

std::ostream &operator<<(std::ostream &out, const MeshPoints &mp) {
  out << static_cast<const Geometry &>(mp) << ",\n" << mp.points;
  return out;
}

}  // namespace DrakeShapes
