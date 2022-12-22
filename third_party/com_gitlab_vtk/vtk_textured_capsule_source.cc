/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkTexturedCapsuleSource.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.

  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "drake/third_party/com_gitlab_vtk/vtk_textured_capsule_source.h"

#include <cmath>

#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkMath.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkStreamingDemandDrivenPipeline.h>

namespace com_gitlab_vtk {

vtkStandardNewMacro(vtkTexturedCapsuleSource);

//------------------------------------------------------------------------------
vtkTexturedCapsuleSource::vtkTexturedCapsuleSource(int res) {
  res = res < 8 ? 8 : res;
  this->Radius = 0.5;
  this->Center[0] = 0.0;
  this->Center[1] = 0.0;
  this->Center[2] = 0.0;
  this->ThetaResolution = res;
  this->PhiResolution = res;
  this->CylinderLength = 1.0;
  this->OutputPointsPrecision = vtkAlgorithm::DEFAULT_PRECISION;
  this->SetNumberOfInputPorts(0);
}

int vtkTexturedCapsuleSource::RequestData(
    vtkInformation* vtkNotUsed(request),
    vtkInformationVector** vtkNotUsed(inputVector),
    vtkInformationVector* outputVector) {
  // get the info object
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  // get the output
  vtkPolyData* output = vtkPolyData::SafeDownCast(
      outInfo->Get(vtkDataObject::DATA_OBJECT()));

  int i, j;
  int numPts;
  int numPolys;
  int phiResolutionHalf;
  vtkPoints* newPoints;
  vtkFloatArray* newNormals;
  vtkFloatArray* newTCoords;
  vtkCellArray* newPolys;
  double x[3], n[3], deltaPhi, deltaTheta, phi, theta, radius, norm;
  vtkIdType pts[3];
  double tc[2], tcNorm;

  // NOTE: To simplify the math for the TCoords and ensure that there are
  // bands generated at the equator for both hemispheres, it is absolutely
  // necessary to ensure PhiResolution is an even number.
  if (this->PhiResolution % 2 != 0) {
    static bool logged = false;
    if (!logged) {
      drake::log()->warn(
          "Capsule PhiResolution must be even, changing from {} to {}!",
          this->PhiResolution, this->PhiResolution + 1);
      logged = true;
    }
    this->PhiResolution += 1;
  }

  //
  // Set things up; allocate memory
  //
  // NOTE: Instead of (phi_R + 1) it should be (phi_R + 2) (two equators).
  numPts = (this->PhiResolution + 2) * (this->ThetaResolution + 1);
  // NOTE: Num of polys increase by theta_R * 2 (i.e., one more band of
  // triangles around the barrel of the cylinder).
  numPolys = (this->PhiResolution + 1) * 2 * this->ThetaResolution;

  newPoints = vtkPoints::New();

  // Set the desired precision for the points in the output.
  if (this->OutputPointsPrecision == vtkAlgorithm::DOUBLE_PRECISION) {
    newPoints->SetDataType(VTK_DOUBLE);
  } else {
    newPoints->SetDataType(VTK_FLOAT);
  }

  newPoints->Allocate(numPts);
  newNormals = vtkFloatArray::New();
  newNormals->SetNumberOfComponents(3);
  newNormals->Allocate(3 * numPts);
  newTCoords = vtkFloatArray::New();
  newTCoords->SetNumberOfComponents(2);
  newTCoords->Allocate(2 * numPts);
  newPolys = vtkCellArray::New();
  newPolys->AllocateEstimate(numPolys, 3);
  //
  // Create capsule
  //
  // Create intermediate points
  // NOTE: We need to make sure that j * deltaPhi (for some integer j) is equal
  // to pi/2. Hard requirement. Generally, not true for arbitrary PhiResolution.
  // This is to ensure we always have a band at the equator for both
  // hemispheres.
  phiResolutionHalf = this->PhiResolution / 2;
  deltaPhi = vtkMath::Pi() / this->PhiResolution;
  deltaTheta = 2.0 * vtkMath::Pi() / this->ThetaResolution;
  tcNorm = (vtkMath::Pi() * this->Radius + this->CylinderLength);
  for (i = 0; i <= this->ThetaResolution; i++) {
    theta = i * deltaTheta;
    tc[0] = theta / (2.0 * vtkMath::Pi());
    for (j = 0; j <= phiResolutionHalf; j++) {
      phi = j * deltaPhi;
      radius = this->Radius * sin(static_cast<double>(phi));
      // The capsule is oriented along the y-axis.
      x[0] = this->Center[0] + radius * cos(static_cast<double>(theta));
      x[1] = this->Center[1] + this->Radius * cos(static_cast<double>(phi))
          + this->CylinderLength / 2;
      x[2] = this->Center[2] + -1 * radius * sin(static_cast<double>(theta));
      newPoints->InsertNextPoint(x);

      // CylinderLength is not needed for the normal computation.
      n[0] = radius * cos(static_cast<double>(theta));
      n[1] = this->Radius * cos(static_cast<double>(phi));
      n[2] = -1 * radius * sin(static_cast<double>(theta));
      if ((norm = vtkMath::Norm(n)) == 0.0) {
        norm = 1.0;
      }
      n[0] /= norm;
      n[1] /= norm;
      n[2] /= norm;
      newNormals->InsertNextTuple(n);

      // 1 - x means flipped image.
      tc[1] = 1.0 - (phi * this->Radius) / tcNorm;
      newTCoords->InsertNextTuple(tc);
    }

    // The 'phiResolutionHalf' is set twice for both the two equators.
    for (j = phiResolutionHalf; j <= 2 * phiResolutionHalf; j++) {
      phi = j * deltaPhi;
      radius = this->Radius * sin(static_cast<double>(phi));
      // The capsule is oriented along the y-axis.
      x[0] = this->Center[0] + radius * cos(static_cast<double>(theta));
      x[1] = this->Center[1] + this->Radius * cos(static_cast<double>(phi))
          - this->CylinderLength / 2;
      x[2] = this->Center[2] + -1 * radius * sin(static_cast<double>(theta));
      newPoints->InsertNextPoint(x);

      n[0] = radius * cos(static_cast<double>(theta));
      n[1] = this->Radius * cos(static_cast<double>(phi));
      n[2] = -1 * radius * sin(static_cast<double>(theta));
      if ((norm = vtkMath::Norm(n)) == 0.0) {
        norm = 1.0;
      }
      n[0] /= norm;
      n[1] /= norm;
      n[2] /= norm;
      newNormals->InsertNextTuple(n);

      // 1 - x means flipped image.
      tc[1] = 1.0 - (phi * this->Radius + this->CylinderLength) / tcNorm;
      newTCoords->InsertNextTuple(tc);
    }
  }

  //
  // Generate mesh connectivity
  //
  for (i = 0; i < this->ThetaResolution; i++) {
    for (j = 0; j <= 2 * phiResolutionHalf; j++) {
      // +2 is required to properly connect the points.
      pts[0] = (2 * phiResolutionHalf + 2) * i + j;
      pts[1] = pts[0] + 1;
      pts[2] = (2 * phiResolutionHalf + 2) * (i + 1) + (j + 1);
      newPolys->InsertNextCell(3, pts);

      pts[1] = pts[2];
      pts[2] = pts[1] - 1;
      newPolys->InsertNextCell(3, pts);
    }
  }

  //
  // Update ourselves and release memory
  //
  output->SetPoints(newPoints);
  newPoints->Delete();

  output->GetPointData()->SetNormals(newNormals);
  newNormals->Delete();

  output->GetPointData()->SetTCoords(newTCoords);
  newTCoords->Delete();

  output->SetPolys(newPolys);
  newPolys->Delete();

  return 1;
}

//------------------------------------------------------------------------------
void vtkTexturedCapsuleSource::PrintSelf(ostream& os, vtkIndent indent) {
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Center: (" << this->Center[0] << ", " << this->Center[1]
      << ", " << this->Center[2] << ")" << std::endl;
  os << indent << "CylinderLength: " << this->CylinderLength << std::endl;
  os << indent << "PhiResolution: " << this->PhiResolution << std::endl;
  os << indent << "ThetaResolution: " << this->ThetaResolution << std::endl;
  os << indent << "Radius: " << this->Radius << std::endl;
  os << indent << "Output Points Precision: " << this->OutputPointsPrecision
      << std::endl;
}

//------------------------------------------------------------------------------
int vtkTexturedCapsuleSource::RequestInformation(
    vtkInformation* vtkNotUsed(request),
    vtkInformationVector** vtkNotUsed(inputVector),
    vtkInformationVector* outputVector) {
  vtkInformation* outInfo = outputVector->GetInformationObject(0);
  outInfo->Set(vtkStreamingDemandDrivenPipeline::UPDATE_NUMBER_OF_PIECES(), -1);
  const double halfLength = this->CylinderLength * 0.5;
  outInfo->Set(vtkStreamingDemandDrivenPipeline::BOUNDS(),
      this->Center[0] - this->Radius - halfLength,
      this->Center[0] + this->Radius + halfLength,
      this->Center[1] - this->Radius,
      this->Center[1] + this->Radius, this->Center[2] - this->Radius,
      this->Center[2] + this->Radius);
  return 1;
}

}  // namespace com_gitlab_vtk
