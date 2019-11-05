/*=========================================================================

  Program: Bender

  Copyright (c) Kitware Inc.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0.txt

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

=========================================================================*/
// .NAME vtkCapsuleSource - Generate a capsule centered at the origin
// .SECTION Description
// vtkCapsuleSource creates a capsule (represented by polygons) of specified
// radius centered at the origin. The resolution (polygonal discretization)
// in both the latitude (phi) and longitude (theta) directions can be
// specified as well as the length of the capsule cylinder (CylinderLength).
// By default, the surface tessellation of the sphere uses triangles;
// however you can set LatLongTessellation to
// produce a tessellation using quadrilaterals (except at the poles of the
// capsule).

#ifndef __vtkCapsuleSource_h
#define __vtkCapsuleSource_h

// VTK includes
#include <vtkPolyDataAlgorithm.h>

#define VTK_MAX_SPHERE_RESOLUTION 1024

namespace com_github_finetjul_bender {

class vtkCapsuleSource : public vtkPolyDataAlgorithm {
 public:
  vtkTypeMacro(vtkCapsuleSource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  // Description:
  // Construct sphere with radius=0.5 and default resolution 8 in both Phi
  // and Theta directions.
  static vtkCapsuleSource *New();

  // Description:
  // Set radius of sphere. Default is 0.5.
  vtkSetClampMacro(Radius,double,0.0,VTK_DOUBLE_MAX);
  vtkGetMacro(Radius,double);

  // Description:
  // Set the center of the sphere. Default is 0,0,0.
  vtkSetVector3Macro(Center,double);
  vtkGetVectorMacro(Center,double,3);

  // Description:
  // Set the length of the cylinder. Default is 1.0.
  vtkSetClampMacro(CylinderLength,double, 0.0, VTK_DOUBLE_MAX);
  vtkGetMacro(CylinderLength,double);

  // Description:
  // Set the number of points used in the longitude direction
  // for the capsule extremities.
  vtkSetClampMacro(ThetaResolution,int,3,VTK_MAX_SPHERE_RESOLUTION);
  vtkGetMacro(ThetaResolution,int);

  // Description:
  // Set the number of points used in the lattitude direction
  // for the capsule extremities.
  vtkSetClampMacro(PhiResolution,int,3,VTK_MAX_SPHERE_RESOLUTION);
  vtkGetMacro(PhiResolution,int);

  // Description:
  // Cause the sphere to be tessellated with edges along the latitude
  // and longitude lines. If off, triangles are generated at non-polar
  // regions, which results in edges that are not parallel to latitude and
  // longitude lines. If on, quadrilaterals are generated everywhere
  // except at the poles. This can be useful for generating a wireframe
  // sphere with natural latitude and longitude lines.
  vtkSetMacro(LatLongTessellation,int);
  vtkGetMacro(LatLongTessellation,int);
  vtkBooleanMacro(LatLongTessellation,int);

protected:
  vtkCapsuleSource(int res=8);
  ~vtkCapsuleSource() {}

  int RequestData(vtkInformation*, vtkInformationVector**,
                  vtkInformationVector*) override;
  int RequestInformation(vtkInformation*, vtkInformationVector**,
                         vtkInformationVector*) override;

  double Radius;
  double Center[3];
  int ThetaResolution;
  int PhiResolution;
  int LatLongTessellation;
  int FillPoles;
  double CylinderLength;

private:
  vtkCapsuleSource(const vtkCapsuleSource&);  // Not implemented.
  void operator=(const vtkCapsuleSource&);  // Not implemented.
};

}  // namespace com_github_finetjul_bender

#endif
