#include "HeightMapTerrain.h"
#include <iostream>
#include <fstream>
#include <cstring>

using namespace std;
using namespace Eigen;

namespace DrakeShapes {
// TODO: it would probably cleaner to have a struct per data type, so
//  you could lookup byte sizes, conversion functions, etc.

bool is_power_of_2(int x) {
  return (x>0 && ((x&(x-1))==0) );
}

HeightMapTerrain::HeightMapTerrain(const std::string& name, const Eigen::Vector2i &ncells, const Eigen::Vector2d &size) :
Geometry(HEIGHT_MAP_TERRAIN), name(name), ncells(ncells), size(size), m_gridHeightScale(1.0), m_upAxis(2),m_type(FLOAT)
{
  //check if ncells is of the form 2^N
  assert(is_power_of_2(ncells(0)) && "ncells must be a power of 2.");
  assert(is_power_of_2(ncells(1)) && "ncells must be a power of 2.");

  nnodes = ncells.array()+1;
  nTotNodes = ((long) nnodes(0)) * nnodes(1);
  bytesPerElement = getByteSize(m_type);
  nBytes = nTotNodes * bytesPerElement;

  m_rawHeightfieldData = new byte_t[nBytes];
  if(m_rawHeightfieldData==NULL) assert(!"Out of memory");

  delta_ell = size.array()/ncells.array().cast<double>();

  fname = name+".obj";
}

HeightMapTerrain::HeightMapTerrain(const HeightMapTerrain& other):
Geometry(HEIGHT_MAP_TERRAIN){
  nBytes = other.nBytes;
  nTotNodes = other.nTotNodes;
  bytesPerElement = other.bytesPerElement;
  size = other.size;
  delta_ell = other.delta_ell;
  ncells = other.ncells;
  nnodes = other.nnodes;
  m_gridHeightScale = other.m_gridHeightScale;
  m_minHeight = other.m_minHeight;
  m_maxHeight = other.m_maxHeight;
  m_upAxis = other.m_upAxis;
  m_type = other.m_type;
  name = other.name;
  fname = other.fname;

  m_rawHeightfieldData = new byte_t[nBytes];
  if(m_rawHeightfieldData==NULL) assert(!"Out of memory");
  std::memcpy(m_rawHeightfieldData, other.m_rawHeightfieldData, nBytes);
}

HeightMapTerrain::~HeightMapTerrain(){
  delete m_rawHeightfieldData;
}

HeightMapTerrain *HeightMapTerrain::clone() const {
  if(m_rawHeightfieldData != NULL)
    assert(!"Are you sure you want to clone this object with heavy data?");
  return new HeightMapTerrain(*this); //unreachable. avoid compiler warning for non return.
}

bool HeightMapTerrain::writeToFile(const string& fname) const{
  ofstream file;
  file.open(fname);

  Vector3f x;

  //Write vertices
  for(int i=0;i<nnodes(0);i++){
    for(int j=0;j<nnodes(1);j++){
      x << i*delta_ell(0) - size(0)/2.0, j*delta_ell(1) - size(1)/2.0, heightValue(i,j);
      file << "v " << x.transpose() << endl;
    }
  }

  //write connectivities (two triangles per cell)
  for(int i=0;i<ncells(0);i++){
    for(int j=0;j<ncells(1);j++){
      //Four corners
      int p1 = (j  )+nnodes(1)*(i  ) + 1;
      int p2 = (j  )+nnodes(1)*(i+1) + 1;
      int p3 = (j+1)+nnodes(1)*(i+1) + 1;
      int p4 = (j+1)+nnodes(1)*(i  ) + 1;

      //first triangle
      file << "f " << p1 << " " << p2 << " " << p4 << endl;

      //second triangle
      file << "f " << p2 << " " << p3 << " " << p4 << endl;

    }
  }

  file.close();

  return true;
}

void HeightMapTerrain::computeMinMaxHeights(){
  m_maxHeight = -numeric_limits<double>::infinity();
  m_minHeight = +numeric_limits<double>::infinity();
  for(int i=0;i<nTotNodes;i++){
    double z = cellValue(i);
    m_minHeight = std::min(m_minHeight,(double)z);
    m_maxHeight = std::max(m_maxHeight,(double)z);
  }

  //This next code ensure the terrain bounding box is centered around z=0 (local z)
  //Bullet takes what we pass as min/max heights and does not compute them.
  //The bounding box is computed based on these values. It is then useful to have minHeight=-maxHeight
  //so that what the user thinks as z=0 actually is z=0.
  //Another option would be to have an offset if keeping a tighter AABB is desired.
  if (m_maxHeight < -m_minHeight) {
    m_maxHeight = -m_minHeight;
  }
  if (m_minHeight > -m_maxHeight) {
    m_minHeight = -m_maxHeight;
  }
}

void HeightMapTerrain::getPoints(Matrix3Xd &points) const {
  assert(!"Implement me!!");
}

void HeightMapTerrain::getBoundingBoxPoints(Matrix3Xd &points) const {
  assert(!"Implement me!!");
}

void HeightMapTerrain::getTerrainContactPoints(Matrix3Xd &points) const {
  assert(!"Implement me!!");
}

void HeightMapTerrain::initialize() {
  assert(!"Implement me!!");
}

// todo: these are implemented assuming the data is stored in FLOAT format
double HeightMapTerrain::cellValue(int i) const{
  return *((double*)(m_rawHeightfieldData+bytesPerElement*i));
}

// todo: these are implemented assuming the data is stored in double format
double HeightMapTerrain::cellValue(int i, int j) const{
  //return *((double*)(m_rawHeightfieldData+(i+j*nnodes(0))*bytesPerElement);
  return *((double*)(m_rawHeightfieldData+(i+j*nnodes(1))*bytesPerElement));
}

double& HeightMapTerrain::cellValue(int i, int j) {
  //return *((double*)(m_rawHeightfieldData+(i+j*nnodes(0))*bytesPerElement);
  return *((double*)(m_rawHeightfieldData+(i+j*nnodes(1))*bytesPerElement));
}

double HeightMapTerrain::heightValue(int i, int j) const{
  //Bullet's local coordinate system is at the center of the height map's bounding box
  return cellValue(i,j) - (m_maxHeight-m_minHeight)/2.0;
}


/************************************************************************************************
*************************************************************************************************
** FLAT TERRAIN
*************************************************************************************************
************************************************************************************************/

FlatTerrain::FlatTerrain(const std::string& name, const Eigen::Vector2i &ncells, const Eigen::Vector2d &size,double angle) :
HeightMapTerrain(name,ncells, size), m_angle(angle)
{
  double slope = tan(m_angle);
  for (int i = 0; i < nnodes(0); ++i) {
    double x = i*delta_ell(0) - size(0)/2.0; //x=0 at the center
    for (int j = 0; j < nnodes(1); ++j) {
        double y = j*delta_ell(1);
        double z = x*slope;
        cellValue(i,j) = z;
      }
    }
  this->computeMinMaxHeights();
}

FlatTerrain::FlatTerrain(const FlatTerrain& other): HeightMapTerrain(other) {
}

HeightMapTerrain *FlatTerrain::clone() const {
  return new FlatTerrain(*this);
}

void FlatTerrain::getPoints(Matrix3Xd &points) const {
  assert(!"Implement me!!");
}

void FlatTerrain::getBoundingBoxPoints(Matrix3Xd &points) const {
  assert(!"Implement me!!");
}

void FlatTerrain::getTerrainContactPoints(Matrix3Xd &points) const {
  assert(!"Implement me!!");
}

void FlatTerrain::initialize() {
  assert(!"Implement me!!");
}

}//namespace DrakeShapes
