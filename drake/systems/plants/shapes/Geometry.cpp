#include <fstream>

#include "Geometry.h"
#include "spruce.hh"

using namespace std;
using namespace Eigen;

namespace DrakeShapes
{
  Geometry::Geometry() : shape(UNKNOWN) {}

  Geometry::Geometry(const Geometry& other)
  {
    shape = other.getShape();
  }

  Geometry::Geometry(Shape shape) : shape(shape) {};

  const Shape Geometry::getShape() const
  {
    return shape;
  }

  Geometry* Geometry::clone() const
  {
    return new Geometry(*this);
  }

  void Geometry::getPoints(Matrix3Xd &points) const
  {
    points = Matrix3Xd();
  }

  void Geometry::getBoundingBoxPoints(Matrix3Xd &points) const
  {
    points = Matrix3Xd();
  }
  
  void Geometry::getBoundingBoxPoints(double x_half_width, double y_half_width, double z_half_width, Eigen::Matrix3Xd &points) const
  {
    // Return axis-aligned bounding-box vertices
      points.resize(3, NUM_BBOX_POINTS);

      RowVectorXd cx(NUM_BBOX_POINTS), cy(NUM_BBOX_POINTS), cz(NUM_BBOX_POINTS);
      cx << -1, 1,  1, -1, -1,  1,  1, -1;
      cy <<  1, 1,  1,  1, -1, -1, -1, -1;
      cz <<  1, 1, -1, -1, -1, -1,  1,  1;
      cx = cx*x_half_width;
      cy = cy*y_half_width;
      cz = cz*z_half_width;

      points << cx, cy, cz;
  }


  Sphere::Sphere(double radius)
    : Geometry(SPHERE), radius(radius) {}

  Sphere* Sphere::clone() const
  {
    return new Sphere(*this);
  }

  void Sphere::getPoints(Matrix3Xd &points) const
  {
    points = Matrix3Xd::Zero(3, NUM_POINTS);
  }

  void Sphere::getBoundingBoxPoints(Matrix3Xd &points) const
  {
    Geometry::getBoundingBoxPoints(radius, radius, radius, points);      
  }

  void Sphere::getTerrainContactPoints(Matrix3Xd &points) const
  {
    if (radius < 1e-6)
      getPoints(points);
    else
      points = Matrix3Xd();
  }

  Box::Box(const Eigen::Vector3d& size)
    : Geometry(BOX), size(size) {}

  Box* Box::clone() const
  {
    return new Box(*this);
  }

  void Box::getPoints(Matrix3Xd &points) const
  {
    Geometry::getBoundingBoxPoints(size(0)/2.0, size(1)/2.0, size(2)/2.0, points);      
  }

  void Box::getBoundingBoxPoints(Matrix3Xd &points) const
  {
    getPoints(points);
  }

  void Box::getTerrainContactPoints(Matrix3Xd &points) const
  {
    getPoints(points);
  }

  Cylinder::Cylinder(double radius, double length)
    : Geometry( CYLINDER), radius(radius), length(length) {}

  Cylinder* Cylinder::clone() const
  {
    return new Cylinder(*this);
  }

  void Cylinder::getPoints(Matrix3Xd &points) const
  {
    static bool warnOnce = true;
    if ( warnOnce ) {
      cerr << "Warning: DrakeShapes::Cylinder::getPoints(): This method returns the vertices of the cylinder''s bounding-box." << endl;
      warnOnce = false;
    }

    getBoundingBoxPoints(points);
  }

  void Cylinder::getBoundingBoxPoints(Matrix3Xd &points) const
  {
    Geometry::getBoundingBoxPoints(radius, radius, length/2.0, points);      
  }

  Capsule::Capsule(double radius, double length)
    : Geometry(CAPSULE), radius(radius), length(length) {}

  Capsule* Capsule::clone() const
  {
    return new Capsule(*this);
  }

  void Capsule::getPoints(Matrix3Xd &points) const
  {
      // Return segment end-points
      points.resize(Eigen::NoChange, NUM_POINTS);
      RowVectorXd cx = RowVectorXd::Zero(NUM_POINTS);
      RowVectorXd cy = RowVectorXd::Zero(NUM_POINTS);
      RowVectorXd cz = RowVectorXd::Zero(NUM_POINTS);
      cz << length/2.0, -length/2.0;

      points << cx, cy, cz;
  }

  void Capsule::getBoundingBoxPoints(Matrix3Xd &points) const
  {
      Geometry::getBoundingBoxPoints(radius, radius, (length/2.0 + radius), points);    
  }

  Mesh::Mesh(const string& filename)
    : Geometry(MESH), scale(1.0), filename(filename)
  {}

  Mesh::Mesh(const string& filename, const string& resolved_filename)
    : Geometry(MESH), scale(1.0), filename(filename), resolved_filename(resolved_filename)
  {}

  bool Mesh::extractMeshVertices(Matrix3Xd& vertex_coordinates) const 
  {
    //DEBUG
    //cout << "Mesh::extractMeshVertices: resolved_filename = " << resolved_filename << endl;
    //END_DEBUG
    if (resolved_filename.empty()) {
      return false;
    }
    spruce::path spath(resolved_filename);
    string ext = spath.extension();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);   

    ifstream file;
    //DEBUG
    //cout << "Mesh::extractMeshVertices: do we have obj?" << endl;
    //END_DEBUG
    if (ext.compare(".obj")==0) {
      //cout << "Loading mesh from " << fname << " (scale = " << scale << ")" << endl;
      file.open(spath.getStr().c_str(),ifstream::in);

    } else {
      //DEBUG
      //cout << "Mesh::extractMeshVertices: check for obj file with same name" << endl;
      //END_DEBUG
      spath.setExtension(".obj");

      if ( spath.exists() ) {
        // try changing the extension to obj and loading
        //      cout << "Loading mesh from " << mypath.replace_extension(".obj").native() << endl;
        file.open(spath.getStr().c_str(),ifstream::in);
      }
    }      

    if (!file.is_open()) {
      cerr << "Warning: Mesh " << spath.getStr() << " ignored because it does not have extension .obj (nor can I find a juxtaposed file with a .obj extension)" << endl;
      return false;
    }

    //DEBUG
    //cout << "Mesh::extractMeshVertices: Count num_vertices" << endl;
    //END_DEBUG
    string line;
    // Count the number of vertices and resize vertex_coordinates
    int num_vertices = 0;
    while (getline(file,line)) {
      istringstream iss(line);
      string type;
      if (iss >> type && type == "v") {
        ++num_vertices;
      }
    }
    //DEBUG
    //cout << "Mesh::extractMeshVertices: num_vertices = " << num_vertices << endl;
    //END_DEBUG
    vertex_coordinates.resize(3, num_vertices);

    file.clear();
    file.seekg(0, file.beg);

    //DEBUG
    //cout << "Mesh::extractMeshVertices: Read vertices" << endl;
    //END_DEBUG
    double d;
    int j = 0;
    while (getline(file,line)) {
      istringstream iss(line);
      string type;
      if (iss >> type && type == "v") {
        //DEBUG
        //cout << "Mesh::extractMeshVertices: Vertex" << j << endl;
        //END_DEBUG
        int i = 0;
        while (iss >> d) {
          vertex_coordinates(i++, j) = d;
        }
        ++j;
      }
    }
    return true;
  }


  Mesh* Mesh::clone() const
  {
    return new Mesh(*this);
  }

  void Mesh::getPoints(Eigen::Matrix3Xd &point_matrix) const
  {
      extractMeshVertices(point_matrix);      
  }

  void Mesh::getBoundingBoxPoints(Matrix3Xd &bbox_points) const
  {
      Matrix3Xd mesh_vertices;
      extractMeshVertices(mesh_vertices);

      Vector3d min_pos = mesh_vertices.rowwise().minCoeff();
      Vector3d max_pos = mesh_vertices.rowwise().maxCoeff();
      
      bbox_points.resize(Eigen::NoChange, NUM_BBOX_POINTS);
      bbox_points << min_pos(0), min_pos(0), min_pos(0), min_pos(0), max_pos(0), max_pos(0), max_pos(0), max_pos(0),
                     min_pos(1), min_pos(1), max_pos(1), max_pos(1), min_pos(1), min_pos(1), max_pos(1), max_pos(1),
                     min_pos(2), max_pos(2), min_pos(2), max_pos(2), min_pos(2), max_pos(2), min_pos(2), max_pos(2);
                     
  }

  MeshPoints::MeshPoints(const Eigen::Matrix3Xd& points) 
    : Geometry(MESH_POINTS), points(points) {}

  MeshPoints* MeshPoints::clone() const
  {
    return new MeshPoints(*this);
  }

  void MeshPoints::getPoints(Eigen::Matrix3Xd &point_matrix) const
  {
     point_matrix = points;
  }

  void MeshPoints::getBoundingBoxPoints(Matrix3Xd &bbox_points) const
  {
    Vector3d min_pos = points.rowwise().minCoeff();
    Vector3d max_pos = points.rowwise().maxCoeff();
     
    bbox_points.resize(Eigen::NoChange, NUM_BBOX_POINTS);
    bbox_points << min_pos(0), min_pos(0), min_pos(0), min_pos(0), max_pos(0), max_pos(0), max_pos(0), max_pos(0),
                   min_pos(1), min_pos(1), max_pos(1), max_pos(1), min_pos(1), min_pos(1), max_pos(1), max_pos(1),
                   min_pos(2), max_pos(2), min_pos(2), max_pos(2), min_pos(2), max_pos(2), min_pos(2), max_pos(2);
  }

}
