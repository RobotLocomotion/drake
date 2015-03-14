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

  Sphere::Sphere(double radius)
    : Geometry(SPHERE), radius(radius) {}

  Sphere* Sphere::clone() const
  {
    return new Sphere(*this);
  }

  Box::Box(const Eigen::Vector3d& size)
    : Geometry(BOX), size(size) {}

  Box* Box::clone() const
  {
    return new Box(*this);
  }

  Cylinder::Cylinder(double radius, double length)
    : Geometry( CYLINDER), radius(radius), length(length) {}

  Cylinder* Cylinder::clone() const
  {
    return new Cylinder(*this);
  }

  Capsule::Capsule(double radius, double length)
    : Geometry(CAPSULE), radius(radius), length(length) {}

  Capsule* Capsule::clone() const
  {
    return new Capsule(*this);
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

  MeshPoints::MeshPoints(const Eigen::Matrix3Xd& points) 
    : Geometry(MESH_POINTS), points(points) {}

  MeshPoints* MeshPoints::clone() const
  {
    return new MeshPoints(*this);
  }

}
