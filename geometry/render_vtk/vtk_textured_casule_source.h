/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkTexturedCapsuleSource.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkTexturedCapsuleSource
 * @brief   Generate a capsule centered at the origin
 *
 * vtkTexturedCapsuleSource creates a capsule (represented by polygons) of
 * specified radius centered at the origin.  The resolution (polygonal
 * discretization) in both the latitude (phi) and longitude (theta) directions
 * can be specified as well as the length of the capsule cylinder
 * (CylinderLength).
 */

#pragma once

#include "vtkPolyDataAlgorithm.h"

namespace drake {
namespace geometry {
namespace render {

class vtkTexturedCapsuleSource : public vtkPolyDataAlgorithm  {
 public:
  vtkTypeMacro(vtkTexturedCapsuleSource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  /**
   * Construct a capsule with radius 0.5 and resolution 8 in both the Phi and
   * Theta directions and a cylinder of length 1.0.
   */
  static vtkTexturedCapsuleSource* New();

  ///@{
  /**
   * Set/get the radius of the capsule.  The initial value is 0.5.
   */
  vtkSetClampMacro(Radius, double, 0.0, VTK_DOUBLE_MAX);
  vtkGetMacro(Radius, double);
  ///@}

  ///@{
  /**
   * Set/get the center of the capsule.  The initial value is (0.0, 0.0, 0.0).
   */
  vtkSetVector3Macro(Center, double);
  vtkGetVectorMacro(Center, double, 3);
  ///@}

  ///@{
  /**
   * Set/get the length of the cylinder.  The initial value is 1.0.
   */
  vtkSetClampMacro(CylinderLength, double, 0.0, VTK_DOUBLE_MAX);
  vtkGetMacro(CylinderLength, double);
  ///@}

  ///@{
  /**
   * Set/get the number of points in the longitude direction for the spheres.
   * The initial value is 8.
   */
  vtkSetClampMacro(ThetaResolution, int, 8, VTK_INT_MAX);
  vtkGetMacro(ThetaResolution, int);
  ///@}

  ///@{
  /**
   * Set/get the number of points in the latitude direction for the spheres.
   * The initial value is 8.
   */
  vtkSetClampMacro(PhiResolution, int, 8, VTK_INT_MAX);
  vtkGetMacro(PhiResolution, int);
  ///@}

  ///@{
  /**
   * DEPRECATED: The sphere wireframe generates triangles within the natural
   * latitude and longitude lines.  This functionality currently achieves
   * nothing.
   *
   * Cause the spheres to be tessellated with edges along the latitude and
   * longitude lines.  If off, triangles are generated at non-polar regions,
   * which results in edges that are not parallel to latitude and longitude
   * lines.  If on, quadrilaterals are generated everywhere except at the
   * poles.  This can be useful for generating wireframe spheres with natural
   * latitude and longitude lines.
   */
  vtkSetMacro(LatLongTessellation, int);
  vtkGetMacro(LatLongTessellation, int);
  vtkBooleanMacro(LatLongTessellation, int);
  ///@}

  ///@{
  /**
   * Set/get the desired precision for the output points.
   * vtkAlgorithm::SINGLE_PRECISION - Output single-precision floating point.
   * vtkAlgorithm::DOUBLE_PRECISION - Output double-precision floating point.
   */
  vtkSetMacro(OutputPointsPrecision, int);
  vtkGetMacro(OutputPointsPrecision, int);
  ///@}

 protected:
  explicit vtkTexturedCapsuleSource(int res = 8);
  ~vtkTexturedCapsuleSource() override = default;

  int RequestData(vtkInformation*, vtkInformationVector**,
                  vtkInformationVector*) override;
  int RequestInformation(vtkInformation*, vtkInformationVector**,
                         vtkInformationVector*) override;

  double Radius;
  double Center[3];
  int ThetaResolution;
  int PhiResolution;
  int LatLongTessellation;
  double CylinderLength;
  int OutputPointsPrecision;

 private:
  vtkTexturedCapsuleSource(const vtkTexturedCapsuleSource&) = delete;
  void operator=(const vtkTexturedCapsuleSource&) = delete;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
