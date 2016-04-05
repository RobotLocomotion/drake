#ifndef __HeightMapTerrain_H__
#define __HeightMapTerrain_H__

#include <string>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <cstring>  //for the definition of syze_t

#include "drake/drakeShapes_export.h"

#include "drake/systems/plants/shapes/Geometry.h"

namespace DrakeShapes {

//!  A class representing a height map terrrain shape
/*!
*/
class DRAKESHAPES_EXPORT HeightMapTerrain : public Geometry {
 public:
  HeightMapTerrain() {}

  //! HeightMapTerrain constructor.
  /*!
    A more elaborate description of the constructor.

    \param size The provided array will be scaled so that the terrain size(0) x
    size(1) in size and has maximum height size(2). The minimum height will be
    located at zero height.
    \param pos Position of the terrain center in world's coordinates. The
    terrain center is located at the middle of the 2D plane and at zero height.
  */
  HeightMapTerrain(const std::string &name, const Eigen::Vector2i &ncells,
                   const Eigen::Vector2d &size);
  HeightMapTerrain(const HeightMapTerrain &other);
  virtual ~HeightMapTerrain();
  virtual HeightMapTerrain *clone() const;
  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;
  virtual void getTerrainContactPoints(Eigen::Matrix3Xd &points) const;

  virtual void initialize();
  int nTotCells() const { return ncells.prod(); }
  double cellValue(int i) const;
  double cellValue(int i, int j) const;
  double heightValue(int i, int j) const;
  double &cellValue(int i, int j);
  bool writeToFile(const std::string &fname) const;

  enum RawDataType { UCHAR, SHORT, FLOAT };
  typedef unsigned char byte_t;

  std::size_t nBytes;  //!< number of bytes allocated in m_rawHeightfieldData
  long nTotNodes;      //!< total number of nodes
  int bytesPerElement;

  // Arrays here are made not-aligned because aligned vectors are causing a
  // strange behaviour during simulation:
  // The bouncing ball on a height map test fails with the ball moving sideways
  // as it hits the height map.
  Eigen::Matrix<double, 2, 1, Eigen::DontAlign> size, delta_ell;
  Eigen::Matrix<int, 2, 1, Eigen::DontAlign> ncells, nnodes;

  double m_gridHeightScale;
  double m_minHeight, m_maxHeight;
  int m_upAxis;
  RawDataType m_type;
  byte_t *m_rawHeightfieldData;
  std::string name, fname;

 protected:
  void computeMinMaxHeights();

  static const char *getDataTypeName(RawDataType type) {
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

  static int getByteSize(RawDataType type) {
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
  *  It represents a simple flat terrain in an angle.
  */
class DRAKESHAPES_EXPORT FlatTerrain : public HeightMapTerrain {
 public:
  FlatTerrain(const std::string &name, const Eigen::Vector2i &ncells,
              const Eigen::Vector2d &size, double angle = 0.0);
  FlatTerrain(const FlatTerrain &other);
  virtual ~FlatTerrain() {}
  HeightMapTerrain *clone() const;
  void getPoints(Eigen::Matrix3Xd &points) const;
  void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;
  void getTerrainContactPoints(Eigen::Matrix3Xd &points) const;

  void initialize();

 protected:
  double m_angle;
};

}  // namespace DrakeShapes

#endif
//__HeightMapTerrain_H__
