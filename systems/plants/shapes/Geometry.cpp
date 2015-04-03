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

  void Geometry::getPoints(Matrix4Xd &points) {
    points = Matrix4Xd();
  }

  void Geometry::getBoundingBoxPoints(Matrix4Xd &points) {
    points = Matrix4Xd();
  }


  Sphere::Sphere(double radius)
    : Geometry(SPHERE), radius(radius) {}

  Sphere* Sphere::clone() const
  {
    return new Sphere(*this);
  }

  void Sphere::getPoints(Matrix4Xd &points) {
    points = Matrix4Xd::Zero(4,1);
    points(3,0) = 1;
  }

  void Sphere::getBoundingBoxPoints(Matrix4Xd &points) {
      // Return axis-aligned bounding-box vertices
      points.resize(4,8);

      RowVectorXd cx(8), cy(8), cz(8);
      cx << -1, 1,  1, -1, -1,  1,  1, -1;
      cy <<  1, 1,  1,  1, -1, -1, -1, -1;
      cz <<  1, 1, -1, -1, -1, -1,  1,  1;
      cx = cx*radius;
      cy = cy*radius;
      cz = cz*radius;

      points << cx, cy, cz, RowVectorXd::Ones(8);
  }

  Box::Box(const Eigen::Vector3d& size)
    : Geometry(BOX), size(size) {}

  Box* Box::clone() const
  {
    return new Box(*this);
  }

  void Box::getPoints(Matrix4Xd &points) {
    points.resize(4,8);
    
    RowVectorXd cx(8), cy(8), cz(8);
    cx << -1, 1,  1, -1, -1,  1,  1, -1;
    cy <<  1, 1,  1,  1, -1, -1, -1, -1;
    cz <<  1, 1, -1, -1, -1, -1,  1,  1;
    cx = cx*size(0)/2;
    cy = cy*size(1)/2;
    cz = cz*size(2)/2;
        
    points << cx, cy, cz, RowVectorXd::Ones(8);
  }

  void Box::getBoundingBoxPoints(Matrix4Xd &points) {
    getPoints(points);
  }

  Cylinder::Cylinder(double radius, double length)
    : Geometry( CYLINDER), radius(radius), length(length) {}

  Cylinder* Cylinder::clone() const
  {
    return new Cylinder(*this);
  }

  void Cylinder::getPoints(Matrix4Xd &points) {
    static bool warnOnce = true;
    if ( warnOnce ) {
      cerr << "Warning: DrakeShapes::Cylinder::getPoints(): This method returns the vertices of the cylinder''s bounding-box." << endl;
      warnOnce = false;
    }

    getBoundingBoxPoints(points);
  }

  void Cylinder::getBoundingBoxPoints(Matrix4Xd &points) {
    // Return axis-aligned bounding-box vertices
    points.resize(4,8);

    RowVectorXd cx(8), cy(8), cz(8);
    cx << -1, 1,  1, -1, -1,  1,  1, -1;
    cy <<  1, 1,  1,  1, -1, -1, -1, -1;
    cz <<  1, 1, -1, -1, -1, -1,  1,  1;
    cx = cx*radius;
    cy = cy*radius;
    cz = cz*length;

    points << cx, cy, cz, RowVectorXd::Ones(8);
  }

  Capsule::Capsule(double radius, double length)
    : Geometry(CAPSULE), radius(radius), length(length) {}

  Capsule* Capsule::clone() const
  {
    return new Capsule(*this);
  }

  void Capsule::getPoints(Matrix4Xd &points) {
      // Return segment end-points
      static bool warnOnce = true;
      if ( warnOnce ) {
        cerr << "Warning: DrakeShapes::Capsule::getPoints(): This method returns the end points of the line segment that defines the capsule";
        warnOnce = false;
      }

      points.resize(4,2);
      RowVectorXd cx = RowVectorXd::Zero(2);
      RowVectorXd cy = RowVectorXd::Zero(2);
      RowVectorXd cz = RowVectorXd::Zero(2);
      cz << length/2, -length/2;

      points << cx, cy, cz, RowVectorXd::Ones(8);
  }

  void Capsule::getBoundingBoxPoints(Matrix4Xd &points) {
      points.resize(4,8);

      RowVectorXd cx(8), cy(8), cz(8);
      cx << -1, 1,  1, -1, -1,  1,  1, -1;
      cy <<  1, 1,  1,  1, -1, -1, -1, -1;
      cz <<  1, 1, -1, -1, -1, -1,  1,  1;
      cx = cx*radius;
      cy = cy*radius;
      cz = cz*(length/2 + radius);


      points << cx, cy, cz, RowVectorXd::Ones(8);
  }

  Mesh::Mesh(const string& filename)
    : Geometry(MESH), filename(filename)
  {}

  Mesh::Mesh(const string& filename, const string& resolved_filename)
    : Geometry(MESH), filename(filename), resolved_filename(resolved_filename)
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

  void Mesh::getPoints(Eigen::Matrix4Xd &point_matrix)
  {
      Matrix3Xd mesh_vertices;
      extractMeshVertices(mesh_vertices);

      point_matrix.resize(4, mesh_vertices.cols());
      point_matrix.block(0,0, 3, mesh_vertices.cols()) = mesh_vertices;
  }

  void Mesh::getBoundingBoxPoints(Matrix4Xd &bbox_points)
  {
      Matrix3Xd mesh_vertices;
      extractMeshVertices(mesh_vertices);

      double min_x = mesh_vertices.block(0,0, 1, mesh_vertices.cols()).minCoeff();
      double max_x = mesh_vertices.block(0,0, 1, mesh_vertices.cols()).maxCoeff();
      double min_y = mesh_vertices.block(1,0, 1, mesh_vertices.cols()).minCoeff();
      double max_y = mesh_vertices.block(1,0, 1, mesh_vertices.cols()).maxCoeff();
      double min_z = mesh_vertices.block(2,0, 1, mesh_vertices.cols()).minCoeff();
      double max_z = mesh_vertices.block(2,0, 1, mesh_vertices.cols()).maxCoeff();

      bbox_points.resize(4,8);
      bbox_points << min_x, max_x, max_x, min_x, min_x, max_x, max_x, min_x,
                     min_y, min_y, max_y, max_y, min_y, min_y, max_y, max_y,
                     min_z, min_z, min_z, min_z, max_z, max_z, max_z, max_z,
                     1,     1,     1,     1,     1,     1,     1,     1;
  }

  MeshPoints::MeshPoints(const Eigen::Matrix3Xd& points) 
    : Geometry(MESH_POINTS), points(points) {}

  MeshPoints* MeshPoints::clone() const
  {
    return new MeshPoints(*this);
  }

  void MeshPoints::getPoints(Eigen::Matrix4Xd &point_matrix)
  {
     point_matrix.resize(4, points.cols());
     point_matrix.block(0,0, 3, points.cols()) = points;
  }

  void MeshPoints::getBoundingBoxPoints(Matrix4Xd &bbox_points)
  {
    double min_x = points.block(0,0, 1, points.cols()).minCoeff();
    double max_x = points.block(0,0, 1, points.cols()).maxCoeff();
    double min_y = points.block(1,0, 1, points.cols()).minCoeff();
    double max_y = points.block(1,0, 1, points.cols()).maxCoeff();
    double min_z = points.block(2,0, 1, points.cols()).minCoeff();
    double max_z = points.block(2,0, 1, points.cols()).maxCoeff();

    bbox_points.resize(4,8);
    bbox_points << min_x, max_x, max_x, min_x, min_x, max_x, max_x, min_x,
                   min_y, min_y, max_y, max_y, min_y, min_y, max_y, max_y,
                   min_z, min_z, min_z, min_z, max_z, max_z, max_z, max_z,
                   1,     1,     1,     1,     1,     1,     1,     1;
  }

}
