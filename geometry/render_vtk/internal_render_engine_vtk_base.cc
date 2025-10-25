#include "drake/geometry/render_vtk/internal_render_engine_vtk_base.h"

#include <ostream>
#include <utility>
#include <vector>

#include <fmt/format.h>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkCellArray.h>                      // vtkCommonDataModel
#include <vtkCylinderSource.h>                 // vtkFiltersSources
#include <vtkFloatArray.h>                     // vtkCommonCore
#include <vtkInformation.h>                    // vtkCommonCore
#include <vtkInformationVector.h>              // vtkCommonCore
#include <vtkObjectFactory.h>                  // vtkCommonCore
#include <vtkPointData.h>                      // vtkCommonDataModel
#include <vtkPoints.h>                         // vtkCommonCore
#include <vtkPolyData.h>                       // vtkCommonDataModel
#include <vtkPolyDataAlgorithm.h>              // vtkCommonExecutionModel
#include <vtkStreamingDemandDrivenPipeline.h>  // vtkCommonExecutionModel

#include "drake/common/fmt_eigen.h"
#include "drake/common/scope_exit.h"
#include "drake/geometry/render/render_mesh.h"

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;
using geometry::internal::RenderMesh;

// TODO(SeanCurtis-TRI): The semantics of this textured box needs to be
//  explained *somewhere* in the documentation. The layout of the texture on the
//  box and (eventually) its ability to be transformed. But also for all shapes.

// TODO(SeanCurtis-TRI): The vtkCylinderSource has a similar artifact as the
//  cube source. The top and bottom faces tile the texture across those faces.
//  The corner of the texture is located at the center of the face and then
//  tiles once per each unit the face spans. This is a horrible behavior.
//  Create a DrakeCylinderSource that matches the Cylinder used in the
//  RenderEngineGl geometry for consistency.

/* A custom poly data algorithm for creating a Box for Drake. This differs from
 (and is preferred over) the vtkCubeSource for the following reasons.

   1. The definition is always centered around the origin of the box's canonical
      frame. This matches the shape specification cleanly.
   2. The vtkCubeSource generates texture coordinates (uvs) in a strange way.
      The vertices are assigned uv values based on where they are in space. This
      leads to tiling of texture for large boxes, and truncating of textures for
      small boxes.

 For texture coordinates, this cube source fits the image to each face,
 regardless of the size or aspect ratio of the face. It does support the ability
 to scale those texture coordinates if tiling is desired or aspect ratios
 should be tweaked.  */
class DrakeCubeSource : public vtkPolyDataAlgorithm {
 public:
  /* This represents adherence to the vtk standard for writing a source. The
   implementation is an explicit spelling of the vtkStandardNewMacro effect.  */
  static DrakeCubeSource* New() {
    DrakeCubeSource* result = new DrakeCubeSource;
    result->InitializeObjectBase();
    return result;
  }

  /* More VTK boiler plate. */
  vtkTypeMacro(DrakeCubeSource, vtkPolyDataAlgorithm);

  /* VTK boilerplate to support printing the source.  */
  void PrintSelf(std::ostream& os, vtkIndent indent) override {
    this->Superclass::PrintSelf(os, indent);
    os << indent << fmt::format("Size: {}\n", fmt_eigen(size_.transpose()));
    os << indent
       << fmt::format("UV Scale: {}\n", fmt_eigen(uv_scale_.transpose()));
  }

  /* Set the size of the box along each of its principal axes.
   @pre Measures must all be positive. */
  void set_size(const Vector3d& size) {
    DRAKE_DEMAND((size.array() > 0).all());
    size_ = size;
  }

  /* Set the scaling of the per-face uv coordinates. This can be used to adjust
   aspect ratio of the rendered texture or introduce tiling. Negative values are
   allowed and reverse the texture along the axes with the negative value.  */
  void set_uv_scale(const Vector2d& uv_scale) { uv_scale_ = uv_scale; }

 protected:
  DrakeCubeSource() : size_(1, 1, 1), uv_scale_(1, 1) {
    this->SetNumberOfInputPorts(0);
  }

  ~DrakeCubeSource() override {}

  int RequestData(vtkInformation*, vtkInformationVector**,
                  vtkInformationVector* outputVector) override {
    // Initializes and allocates the accumulators for the box data.
    vtkPoints* newPoints = vtkPoints::New(VTK_DOUBLE);
    vtkFloatArray* newNormals = vtkFloatArray::New();
    vtkFloatArray* newTCoords = vtkFloatArray::New();
    vtkCellArray* newPolys = vtkCellArray::New();

    ScopeExit guard([newPoints, newNormals, newTCoords, newPolys]() {
      newPoints->Delete();
      newNormals->Delete();
      newTCoords->Delete();
      newPolys->Delete();
    });

    const int num_polys = 6;
    const int num_points = 24;
    newPoints->Allocate(num_points);
    newNormals->SetNumberOfComponents(3);
    newNormals->Allocate(num_points);
    newNormals->SetName("Normals");
    newTCoords->SetNumberOfComponents(2);
    newTCoords->Allocate(num_points);
    newTCoords->SetName("TCoords");
    // This estimate is exact because every polygon is a quad.
    newPolys->AllocateEstimate(num_polys, 4);

    /* Each face is defined w.r.t. a particular axis (e.g., +x, -x, +y, ...,
     etc.,) Looking *down* the vector of that axis there is a Frame on which
     we define the face.

                       Fy
                       ^
               v3      │       v2
                ┏━━━━━━┿━━━━━━┓
                ┃      │      ┃
                ┃      └──────╂─>  Fx
                ┃             ┃
                ┗━━━━━━━━━━━━━┛
               v0              v1

     The texture coordinates for the vertices map the image directly:
       - v0 -> (0, 0)
       - v2 -> (1, 1), etc.

     The box itself is defined in frame B. So, for each face (+x, -x, etc.)
     there is a mapping from the bases of B to the bases of F.

     face axis |  Fx |  Fy | map key
     --------- | --- | --- | -------
         +x    | -By |  Bz |   0
         -x    |  By |  Bz |   1
         +y    | -Bx |  Bz |   2
         -y    |  Bx |  Bz |   3
         +z    |  Bx |  By |   4
         -z    |  Bx | -By |   5

    We encode the basis for each face in the table below as:
      {Bi, si, Bj, sj, Bk, sk} such that:
         Fx = si * Vector3d::Unit(Bi);
         Fy = sj * Vector3d::Unit(Bj);
         Fz = sk * Vector3d::Unit(Bk);
    */
    constexpr int basis_F[6][6] = {{1, 1, 2, 1, 0, 1},     // +x face.
                                   {1, -1, 2, 1, 0, -1},   // -x face.
                                   {0, -1, 2, 1, 1, 1},    // +y face.
                                   {0, 1, 2, 1, 1, -1},    // -y face.
                                   {0, 1, 1, 1, 2, 1},     // +z face.
                                   {0, 1, 1, -1, 2, -1}};  // -z face.
    for (int face = 0; face < 6; ++face) {
      // The indices of the basis of frame F.
      const int fx = basis_F[face][0];
      const int fy = basis_F[face][2];
      const int fz = basis_F[face][4];
      // The half sizes in the Frame F basis directions.
      const double sx = 0.5 * this->size_[fx];
      const double sy = 0.5 * this->size_[fy];
      const double sz = 0.5 * this->size_[fz];
      // The basis vectors in the box frame B.
      const Vector3d Fx = basis_F[face][1] * Vector3d::Unit(fx);
      const Vector3d Fy = basis_F[face][3] * Vector3d::Unit(fy);
      const Vector3d Fz = basis_F[face][5] * Vector3d::Unit(fz);

      // The face normal is always in the Fz direction.
      const double normal_F[] = {Fz[0], Fz[1], Fz[2]};

      // Position vector from Frame F's origin, to the center of the polygon.
      const Vector3d p_FPc = sz * Fz;
      // Coordinates of the "unit" faces. I.e., if the box were (2 x 2 x 2), it
      // would have vertices at positions <±1, ±1, ±1>. We'll scale it based
      // on the box dimensions.
      std::vector<std::pair<double, double>> vertices = {{-1, -1},  // v0.
                                                         {1, -1},   // v1.
                                                         {1, 1},    // v2.
                                                         {-1, 1}};  // v3.
      // The index of the first vertex we're about to add.
      const vtkIdType first = newPoints->GetNumberOfPoints();
      for (const auto& [x, y] : vertices) {
        const Vector3d p_FV = p_FPc + (x * sx) * Fx + (y * sy) * Fy;
        newPoints->InsertNextPoint(p_FV[0], p_FV[1], p_FV[2]);
        newNormals->InsertNextTuple(normal_F);
        // Note: The texture coordinates are a simple affine transformation of
        // the unit face coordinates. For vertex P in the unit face, its
        // corresponding uv coordinate UV = s * (P + 1) / 2. In other words, we:
        //   - shift the square from (-1, -1)x(1, 1) to (0, 0)x(2, 2)
        //   - scale to (0, 0)x(1, 1)
        //   - scale again by the uv_scale to (0, 0)x(s, s).
        const double uv[] = {(x + 1) * 0.5 * uv_scale_[0],
                             (y + 1) * 0.5 * uv_scale_[1]};
        newTCoords->InsertNextTuple(uv);
      }

      const vtkIdType polygon[] = {first, first + 1, first + 2, first + 3};
      newPolys->InsertNextCell(4, polygon);
    }

    // Populates the output with the compute data.
    vtkInformation* out_info = outputVector->GetInformationObject(0);
    vtkPolyData* output =
        vtkPolyData::SafeDownCast(out_info->Get(vtkDataObject::DATA_OBJECT()));
    output->SetPoints(newPoints);
    output->GetPointData()->SetNormals(newNormals);
    output->GetPointData()->SetTCoords(newTCoords);
    output->SetPolys(newPolys);

    return 1;  // Signal success per VTK convention.
  }

  // The full measure of the box along its principal axes.
  Vector3d size_;

  // TODO(SeanCurtis-TRI): This would be better off as a uv _transform_ such
  //  that the texture can be scaled _and_ positioned.
  // The scale factors to apply to the texture coordinates.
  Vector2d uv_scale_;

 private:
  DrakeCubeSource(const DrakeCubeSource&) = delete;
  void operator=(const DrakeCubeSource&) = delete;
};

class DrakeObjSource : public vtkPolyDataAlgorithm {
 public:
  static DrakeObjSource* New() {
    DrakeObjSource* result = new DrakeObjSource;
    result->InitializeObjectBase();
    return result;
  }

  vtkTypeMacro(DrakeObjSource, vtkPolyDataAlgorithm);

  void PrintSelf(std::ostream& os, vtkIndent indent) override {
    this->Superclass::PrintSelf(os, indent);
    os << indent << "Num triangles: " << mesh_.indices.rows();
  }

  void set_render_mesh(RenderMesh mesh) { mesh_ = std::move(mesh); }

 protected:
  DrakeObjSource() { this->SetNumberOfInputPorts(0); }

  ~DrakeObjSource() override {}

  int RequestData(vtkInformation*, vtkInformationVector**,
                  vtkInformationVector* outputVector) override {
    vtkPoints* newPoints = vtkPoints::New(VTK_DOUBLE);
    vtkFloatArray* newNormals = vtkFloatArray::New();
    vtkFloatArray* newTCoords = vtkFloatArray::New();
    vtkCellArray* newPolys = vtkCellArray::New();

    ScopeExit guard([newPoints, newNormals, newTCoords, newPolys]() {
      newPoints->Delete();
      newNormals->Delete();
      newTCoords->Delete();
      newPolys->Delete();
    });

    const int num_points = mesh_.positions.rows();
    const int num_tris = mesh_.indices.rows();
    newPoints->Allocate(num_points);
    newNormals->SetNumberOfComponents(3);
    newNormals->Allocate(num_points);
    newNormals->SetName("Normals");
    newTCoords->SetNumberOfComponents(2);
    newTCoords->Allocate(num_points);
    newTCoords->SetName("TCoords");
    // This estimate is exact because every polygon is a triangle.
    newPolys->AllocateEstimate(num_tris, 3);

    for (int p = 0; p < num_points; ++p) {
      newPoints->InsertNextPoint(mesh_.positions(p, 0), mesh_.positions(p, 1),
                                 mesh_.positions(p, 2));
      const double n[] = {mesh_.normals(p, 0), mesh_.normals(p, 1),
                          mesh_.normals(p, 2)};
      newNormals->InsertNextTuple(n);
      const double uv[] = {mesh_.uvs(p, 0), mesh_.uvs(p, 1)};
      newTCoords->InsertNextTuple(uv);
    }
    for (int t = 0; t < num_tris; ++t) {
      const auto& row = mesh_.indices.row(t);
      const vtkIdType triangle[] = {row[0], row[1], row[2]};
      newPolys->InsertNextCell(3, triangle);
    }

    // Populates the output with the compute data.
    vtkInformation* out_info = outputVector->GetInformationObject(0);
    vtkPolyData* output =
        vtkPolyData::SafeDownCast(out_info->Get(vtkDataObject::DATA_OBJECT()));
    output->SetPoints(newPoints);
    output->GetPointData()->SetNormals(newNormals);
    output->GetPointData()->SetTCoords(newTCoords);
    output->SetPolys(newPolys);

    return 1;  // Signal success per VTK convention.
  }

 private:
  DrakeObjSource(const DrakeObjSource&) = delete;
  void operator=(const DrakeObjSource&) = delete;

  RenderMesh mesh_;
};

}  // namespace

vtkSmartPointer<vtkPolyDataAlgorithm> CreateVtkCapsule(const Capsule& capsule) {
  // Note: VTK uses cylinder sources as capsules by turning on the caps and
  // making them spherical.
  vtkNew<vtkCylinderSource> vtk_capsule;
  vtk_capsule->CappingOn();
  vtk_capsule->CapsuleCapOn();
  SetCylinderOptions(vtk_capsule, capsule.length(), capsule.radius());
  return TransformToDrakeCylinder(vtk_capsule);
}

vtkSmartPointer<vtkPolyDataAlgorithm> CreateVtkBox(
    const Box& box, const PerceptionProperties& properties) {
  vtkSmartPointer<DrakeCubeSource> vtk_box =
      vtkSmartPointer<DrakeCubeSource>::New();
  vtk_box->set_size({box.width(), box.depth(), box.height()});
  const Vector2d& uv_scale =
      properties.GetPropertyOrDefault("phong", "diffuse_scale", Vector2d{1, 1});
  vtk_box->set_uv_scale(uv_scale);
  return vtk_box;
}

vtkSmartPointer<vtkPolyDataAlgorithm> CreateVtkEllipsoid(
    const Ellipsoid& ellipsoid) {
  vtkNew<vtkTexturedSphereSource> vtk_ellipsoid;
  vtk_ellipsoid->SetRadius(1.0);
  // TODO(SeanCurtis-TRI): Provide control for smoothness/tessellation.
  vtk_ellipsoid->SetThetaResolution(50);
  vtk_ellipsoid->SetPhiResolution(50);

  // Scale sphere by each axis extent to generate the ellipsoid.
  vtkNew<vtkTransform> transform;
  transform->Scale(ellipsoid.a(), ellipsoid.b(), ellipsoid.c());
  vtkSmartPointer<vtkTransformPolyDataFilter> transform_filter =
      vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transform_filter->SetInputConnection(vtk_ellipsoid->GetOutputPort());
  transform_filter->SetTransform(transform);
  transform_filter->Update();
  return transform_filter;
}

vtkSmartPointer<vtkPolyDataAlgorithm> CreateVtkMesh(RenderMesh mesh_data) {
  vtkSmartPointer<DrakeObjSource> mesh_source =
      vtkSmartPointer<DrakeObjSource>::New();
  mesh_source->set_render_mesh(std::move(mesh_data));
  mesh_source->Update();
  return mesh_source;
}

void SetSphereOptions(vtkTexturedSphereSource* vtk_sphere, double radius) {
  vtk_sphere->SetRadius(radius);
  // TODO(SeanCurtis-TRI): Provide control for smoothness/tessellation.
  vtk_sphere->SetThetaResolution(50);
  vtk_sphere->SetPhiResolution(50);
}

void SetCylinderOptions(vtkCylinderSource* vtk_cylinder, double height,
                        double radius) {
  vtk_cylinder->SetHeight(height);
  vtk_cylinder->SetRadius(radius);
  // TODO(SeanCurtis-TRI): Provide control for smoothness/tessellation.
  vtk_cylinder->SetResolution(50);
}

vtkSmartPointer<vtkPolyDataAlgorithm> TransformToDrakeCylinder(
    vtkCylinderSource* vtk_cylinder) {
  // We rotate around the x-axis to get the cylinder to be vertical along the
  // z-axis.
  vtkNew<vtkTransform> rotate;
  rotate->RotateX(-90);
  // The "prime meridian" on the cylinder is 90 degrees off of the sphere's. So,
  // we'll rotate once more so that they have the same sense of "front".
  vtkNew<vtkTransform> rotate_around_Z;
  rotate_around_Z->RotateY(90);
  rotate->Concatenate(rotate_around_Z);

  vtkNew<vtkTransformPolyDataFilter> transform_filter;
  transform_filter->SetInputConnection(vtk_cylinder->GetOutputPort());
  transform_filter->SetTransform(rotate);
  transform_filter->Update();
  return transform_filter;
}

}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
