#include "drake/geometry/proximity/mesh_to_vtk.h"

#include <fstream>
#include <iostream>

#include <fmt/format.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

// Most of the functions in this anonymous namespace are templated on
// the types of meshes or fields. They are used by the functions declared in
// the header file, which are defined at the end of this file.

void WriteVtkHeader(std::ofstream& out, const std::string& title) {
  out << "# vtk DataFile Version 3.0\n";
  out << title << std::endl;
  out << "ASCII\n";
  // An extra blank line makes the file more human readable.
  out << std::endl;
}

/*
 @tparam Mesh  VolumeMesh<double> or TriangleSurfaceMesh<double>
 */
template <typename Mesh>
void WriteVtkUnstructuredGrid(std::ofstream& out, const Mesh& mesh) {
  const int num_points = mesh.num_vertices();
  out << "DATASET UNSTRUCTURED_GRID\n";
  out << "POINTS " << num_points << " double\n";
  for (const Vector3<double> p_MV : mesh.vertices()) {
    out << fmt::format("{:12.8f} {:12.8f} {:12.8f}\n", p_MV.x(), p_MV.y(),
                       p_MV.z());
  }
  out << std::endl;

  // For a triangular mesh, an element is a triangle.
  // For a tetrahedral mesh, an element is a tetrahedron.
  const int num_elements = mesh.num_elements();
  // For a triangular mesh, we use 4 integers per triangle.
  // For a tetrahedral mesh, we use 5 integers per tetrahedron.
  const int num_vertices_per_element = Mesh::kVertexPerElement;
  const int num_integers = num_elements * (num_vertices_per_element + 1);
  out << "CELLS " << num_elements << " " << num_integers << std::endl;
  for (int i = 0; i < num_elements; ++i) {
    const auto& element = mesh.element(i);
    out << fmt::format("{}", num_vertices_per_element);
    for (int v = 0; v < num_vertices_per_element; ++v) {
      out << fmt::format(" {:6d}", element.vertex(v));
    }
    out << std::endl;
  }
  out << std::endl;

  const int kVtkCellTypeTriangle = 5;
  const int kVtkCellTypeTetrahedron = 10;
  const int cell_type = (Mesh::kVertexPerElement == 3)
                            ? kVtkCellTypeTriangle
                            : kVtkCellTypeTetrahedron;
  out << "CELL_TYPES " << num_elements << std::endl;
  for (int i = 0; i < num_elements; ++i) {
    out << fmt::format("{}\n", cell_type);
  }
  out << std::endl;
}

/*
 @tparam Mesh  VolumeMesh<double> or TriangleSurfaceMesh<double>
 */
template<typename Mesh>
void WriteMeshToVtk(const std::string& file_name,
                    const Mesh& mesh,
                    const std::string& title) {
  std::ofstream file(file_name);
  if (file.fail()) {
    throw std::runtime_error(fmt::format("Cannot create file: {}.", file_name));
  }
  WriteVtkHeader(file, title);
  WriteVtkUnstructuredGrid(file, mesh);
  file.close();
}

/*
 @tparam Field VolumeMeshFieldLinear<double, double> or
               TriangleSurfaceMeshFieldLinear<double, double>
 */
template <typename Field>
void WriteVtkScalarField(
    std::ofstream& out, std::string name,
    const Field& field) {
  out << fmt::format("POINT_DATA {}\n", field.values().size());
  // VTK doesn't like space ' ' in the name of the scalar field.
  // Replace space ' ' with underscore '_'.
  std::replace(name.begin(), name.end(), ' ', '_');
  out << fmt::format("SCALARS {} double 1\n", name);
  out << "LOOKUP_TABLE default\n";
  for (auto value : field.values()) {
    out << fmt::format("{:20.8f}\n", value);
  }
  out << std::endl;
}

/*
 @tparam Field VolumeMeshFieldLinear<double, double> or
               TriangleSurfaceMeshFieldLinear<double, double>
 */
template <typename Field>
void WriteMeshFieldLinearToVtk(const std::string& file_name,
                               const std::string& field_name,
                               const Field& field, const std::string& title) {
  std::ofstream file(file_name);
  if (file.fail()) {
    throw std::runtime_error(fmt::format("Cannot create file: {}.", file_name));
  }
  WriteVtkHeader(file, title);
  WriteVtkUnstructuredGrid(file, field.mesh());
  WriteVtkScalarField(file, field_name, field);
  file.close();
}

}  // namespace

void WriteVolumeMeshToVtk(const std::string& file_name,
                          const VolumeMesh<double>& mesh,
                          const std::string& title) {
  WriteMeshToVtk(file_name, mesh, title);
}

void WriteSurfaceMeshToVtk(const std::string& file_name,
                           const TriangleSurfaceMesh<double>& mesh,
                           const std::string& title) {
  WriteMeshToVtk(file_name, mesh, title);
}

void WriteVolumeMeshFieldLinearToVtk(
    const std::string& file_name, const std::string& field_name,
    const VolumeMeshFieldLinear<double, double>& field,
    const std::string& title) {
  WriteMeshFieldLinearToVtk(file_name, field_name, field, title);
}

void WriteTriangleSurfaceMeshFieldLinearToVtk(
    const std::string& file_name, const std::string& field_name,
    const TriangleSurfaceMeshFieldLinear<double, double>& field,
    const std::string& title) {
  WriteMeshFieldLinearToVtk(file_name, field_name, field, title);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
