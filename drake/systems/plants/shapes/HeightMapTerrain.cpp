#include "HeightMapTerrain.h"
#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

namespace DrakeShapes {
// TODO: it would probably cleaner to have a struct per data type, so
//  you could lookup byte sizes, conversion functions, etc.
#if 0
static int getByteSize(PHY_ScalarType type)
{
  int size = 0;

  switch (type) {
  case PHY_FLOAT:
    size = sizeof(float);
    break;

  case PHY_UCHAR:
    size = sizeof(unsigned char);
    break;

  case PHY_SHORT:
    size = sizeof(short);
    break;

  default:
    assert(!"Bad heightfield data type");
  }

  return size;
}
#endif

bool is_power_of_2(int x) {
  return (x>0 && ((x&(x-1))==0) );
}

HeightMapTerrain::HeightMapTerrain(const std::string& name, const Eigen::Vector2i &ncells, const Eigen::Vector2d &size) : 
Geometry(HEIGHT_MAP_TERRAIN), name(name), ncells(ncells), size(size), m_gridHeightScale(1.0), m_upAxis(2),m_type(FLOAT)
{
    //check if ncells_along_width is of the form (2^N) + 1    
  PRINT_VAR(ncells(0));
  PRINT_VAR(is_power_of_2(ncells(0)));

  assert(is_power_of_2(ncells(0)) && "ncells must be of the form 2^n.");
  assert(is_power_of_2(ncells(1)) && "ncells must be of the form 2^n.");  
  nnodes = ncells.array()+1;
  bytesPerElement = getByteSize(m_type);
  m_rawHeightfieldData = NULL;

  delta_ell = size.array()/ncells.array().cast<double>(); 
}

HeightMapTerrain *HeightMapTerrain::clone() const {   
  if(m_rawHeightfieldData != NULL)
    assert(!"Are you sure you want to clone this object with heavy data?");
  return new HeightMapTerrain(*this);   
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

FlatTerrain::FlatTerrain(const std::string& name, const Eigen::Vector2i &ncells, const Eigen::Vector2d &size) : 
HeightMapTerrain(name,ncells, size)
{
  long nElements = ((long) nnodes(0)) * nnodes(1);  

  long nBytes = nElements * bytesPerElement;

  PRINT_VAR(ncells);
  PRINT_VAR(nnodes);
  PRINT_VAR(nElements);
  PRINT_VAR(bytesPerElement);
  PRINT_VAR(nBytes);

  m_rawHeightfieldData = new byte_t[nBytes];
  if(m_rawHeightfieldData==NULL) assert(!"Out of memory");

  m_maxHeight = -numeric_limits<double>::infinity();
  m_minHeight = +numeric_limits<double>::infinity();

  //byte_t * p = m_rawHeightfieldData;
  for (int i = 0; i < nnodes(0); ++i) {  
    double x = i*delta_ell(0);  
    for (int j = 0; j < nnodes(1); ++j) {        
        double y = j*delta_ell(1);        
        double z = sin(x*3.1416/size(0));

        //double z = 0.0; //simple flat terrain at z=0                

        cellValue(i,j) = z;

        m_minHeight = std::min(m_minHeight,(double)z);
        m_maxHeight = std::max(m_maxHeight,(double)z);
        //convertFromdouble(p, z, m_type);
        //p += bytesPerElement;
      }
    }
  
  //m_maxHeight = size.maxCoeff()/10.0;
  //m_minHeight = -m_maxHeight;

  fname = name+".obj";
  //writeToFile(fname);
}

HeightMapTerrain *FlatTerrain::clone() const {
  FlatTerrain *cloned = new FlatTerrain(*this); 
  cloned->m_rawHeightfieldData = m_rawHeightfieldData; //the clone points to the same data!!
  return cloned;
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
