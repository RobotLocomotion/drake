#include "drake/automotive/maliput/utility/generate_urdf.h"

#include <fstream>
#include <iostream>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/utility/generate_obj.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace utility {

void generate_urdf(const std::string& dirname,
                   const std::string& fileroot,
                   const api::RoadGeometry* rg,
                   const double grid_unit) {
  const std::string obj_filename = fileroot + ".obj";

  std::ofstream os(dirname + "/" + fileroot + ".urdf");
  os << "<?xml version=\"1.0\" ?>" << std::endl;
  os << "<robot name=\"" << rg->id().id << "\">" << std::endl;
  os << "  <link name=\"world\"/>" << std::endl;
  os << std::endl;
  os << "  <joint name=\"world_to_road_joint\" type=\"continuous\">" << std::endl;
  os << "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" << std::endl;
  os << "    <parent link=\"world\"/>" << std::endl;
  os << "    <child link=\"surface\"/>" << std::endl;
  os << "  </joint>" << std::endl;
  os << std::endl;
  os << "  <link name=\"surface\">" << std::endl;
  os << "    <inertial/>" << std::endl;
  os << "    <visual name=\"v1\">" << std::endl;
  os << "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" << std::endl;
  os << "      <geometry>" << std::endl;
  os << "        <mesh filename=\"" << obj_filename
     << "\" scale=\"1.0 1.0 1.0\"/>" << std::endl;
  os << "      </geometry>" << std::endl;
  os << "    </visual>" << std::endl;
  os << "  </link>" << std::endl;
  os << "</robot>" << std::endl;

  generate_obj(rg, dirname + "/" + obj_filename, grid_unit);
}


}  // namespace utility
}  // namespace maliput
}  // namespace drake
