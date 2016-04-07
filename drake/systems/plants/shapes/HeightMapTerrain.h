#ifndef DRAKE_SHAPES_HEIGHT_MAP_TERRAIN_H_
#define DRAKE_SHAPES_HEIGHT_MAP_TERRAIN_H_

#include <string>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <cstring> //for the definition of syze_t

#include "drake/drakeShapes_export.h"

#include "drake/systems/plants/shapes/Geometry.h"

namespace DrakeShapes {

/**
 *  A class representing a height map terrain shape.
 *  A height map is stored as a two dimensional array of heights.
 *  This allows for more efficient algorithms for collision detection since a heigh map's structure is much simpler
 *  than that of a general triangulated surface.
 *  Height maps are desired over general triangulated surfaces whenever they are suffucient to provide a given terrain's geometry.
 */
class DRAKESHAPES_EXPORT HeightMapTerrain : public Geometry {
 public:

  HeightMapTerrain(){}

  /** \brief HeightMapTerrain constructor.
   *  \param name This is only used to generate a file name where the height map is written as a triangular mesh for director to render. The file name will be <name>.obj.
   *  \param ncells Two dimensional vector of integers with the number of cells used to discretize the map in "local" x and y directions respectively. The number of cells MUST be a power of 2.
   *  \param size The provided array will be scaled so that the terrain is size(0) x size(1) in size.
   */
  HeightMapTerrain(const std::string& name, const Eigen::Vector2i &ncells, const Eigen::Vector2d &size);
  HeightMapTerrain(const HeightMapTerrain& other);
  ~HeightMapTerrain() override;
  HeightMapTerrain *clone() const override;

  //The following set of routines are used nowhere in the code however are required in Geometry's interface
  void getPoints(Eigen::Matrix3Xd &points) const override;
  void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const override;
  void getTerrainContactPoints(Eigen::Matrix3Xd &points) const override;

  int nTotCells()const{ return ncells.prod();}
  double cellValue(int i) const;
  double cellValue(int i,int j) const;
  double heightValue(int i,int j) const;
  double& cellValue(int i,int j);
  bool writeToFile(const std::string& fname) const;

  enum RawDataType {UCHAR, SHORT,FLOAT};
  typedef unsigned char byte_t;

  std::size_t nBytes;   //!< number of bytes allocated in m_rawHeightfieldData
  long nTotNodes;       //!< total number of nodes
  int bytesPerElement;

  //Arrays here are made not-aligned because aligned vectors are causing a strange behaviour during simulation:
  //The bouncing ball on a height map test fails with the ball moving sideways as it hits the height map.
  Eigen::Matrix<double,2,1,Eigen::DontAlign> size, delta_ell;
  Eigen::Matrix<int,2,1,Eigen::DontAlign> ncells, nnodes;

  double m_gridHeightScale;
  double m_minHeight, m_maxHeight;
  int m_upAxis;
  RawDataType m_type;
  byte_t *m_rawHeightfieldData; //use a unique_ptr here
  std::string name, fname;

  protected:
    void computeMinMaxHeights();

    static const char* getDataTypeName(RawDataType type)
    {
      switch (type) {
        case UCHAR:
        return "UnsignedChar";

        case SHORT:
        return "Short";

        case FLOAT:
        return "Float";

        default:
        assert(!"bad heightfield data type");
      }

      return NULL;
    }

    static int getByteSize(RawDataType type)
    {
      int size = 0;

      switch (type) {
        case FLOAT:
        size = sizeof(double);
        break;

        case UCHAR:
        size = sizeof(unsigned char);
        break;

        case SHORT:
        size = sizeof(short);
        break;

        default:
        assert(!"Bad heightfield data type");
      }

      return size;
    }
};

/** \brief Example class showing how to inherit from HeightMapTerrain.
  *  It represents a simple flat terrain at an angle.
  */
class DRAKESHAPES_EXPORT FlatTerrain : public HeightMapTerrain {
 public:
  FlatTerrain(const std::string& name, const Eigen::Vector2i &ncells, const Eigen::Vector2d &size, double angle=0.0);
  FlatTerrain(const FlatTerrain& other);
  virtual ~FlatTerrain() {}
  HeightMapTerrain *clone() const;
  void getPoints(Eigen::Matrix3Xd &points) const final;
  void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const final;
  void getTerrainContactPoints(Eigen::Matrix3Xd &points) const final;

  protected:
    double m_angle;
};

}//namespace DrakeShapes

#endif
//DRAKE_SHAPES_HEIGHT_MAP_TERRAIN_H_
