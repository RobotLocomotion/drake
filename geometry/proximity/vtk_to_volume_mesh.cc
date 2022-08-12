#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace geometry {
namespace internal {

namespace {

VolumeMesh<double> DoReadVtkToVolumeMesh(
    std::istream* input_stream,
    const std::function<void(std::string_view)> on_warning) {
  std::string warn;
  std::string line;
  bool reading_points = false;
  bool reading_tets = false;
  int n_points = 0;
  int n_tets = 0;
  Vector3<double> position;
  Vector4<int> vertex_index;
  std::vector<VolumeElement> elements;
  std::vector<Vector3<double>> vertices;
  while (std::getline(*input_stream, line)) {
    std::stringstream ss(line);
    if (static_cast<int>(line.size()) == 0) {
    } else if (line.substr(0, 6) == "POINTS") {
      reading_points = true;
      reading_tets = false;
      ss.ignore(128, ' ');  // Ignore "POINTS".
      ss >> n_points;
      vertices.reserve(n_points);
    } else if (line.substr(0, 5) == "CELLS") {
      reading_points = false;
      reading_tets = true;
      ss.ignore(128, ' ');  // Ignore "CELLS".
      ss >> n_tets;
      elements.reserve(n_tets);
    } else if (line.substr(0, 10) == "CELL_TYPES") {
      reading_points = false;
      reading_tets = false;
    } else if (reading_points) {
      for (int i = 0; i < 3; ++i) ss >> position(i);
      vertices.emplace_back(position);
    } else if (reading_tets) {
      int d;
      ss >> d;
      // Only tetrahedral mesh is supported.
      if (d != 4) {
        warn = "There are unsupported mesh elements. Only tetrahedron "
               "(4 vertices per cell) is supported.";
      }
      ss.ignore(128, ' ');  // ignore "4"
      for (int i = 0; i < 4; i++) {
        ss >> vertex_index(i);
      }
      elements.emplace_back(vertex_index(0), vertex_index(1),
                            vertex_index(2), vertex_index(3));
    }
  }

  if (!warn.empty()) {
    warn = "Warning parsing VTK tetrahedral-mesh file : " + warn;
    if (warn.back() == '\n') {
      warn.pop_back();
    }
    if (on_warning) {
      on_warning(warn);
    } else {
      drake::log()->warn(warn);
    }
  }

  return {std::move(elements), std::move(vertices)};
}

}  // namespace

VolumeMesh<double> ReadVtkToVolumeMesh(
    const std::string& filename,
    std::function<void(std::string_view)> on_warning) {
  std::ifstream input_stream(filename);
  if (!input_stream.is_open()) {
    throw std::runtime_error("ReadVtkToVolumeMesh: cannot open file '" +
                             filename + "'");
  }
  return DoReadVtkToVolumeMesh(&input_stream, std::move(on_warning));
}

VolumeMesh<double> ReadVtkToVolumeMesh(
    std::istream* input_stream,
    std::function<void(std::string_view)> on_warning) {
  DRAKE_THROW_UNLESS(input_stream != nullptr);
  return DoReadVtkToVolumeMesh(input_stream, std::move(on_warning));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
