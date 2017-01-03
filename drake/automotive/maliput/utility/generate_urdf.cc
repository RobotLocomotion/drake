#include "drake/automotive/maliput/utility/generate_urdf.h"

#include <fstream>

#include "drake/automotive/maliput/utility/generate_obj.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace utility {

void GenerateUrdfFile(const api::RoadGeometry* road_geometry,
                      const std::string& dirname,
                      const std::string& fileroot,
                      const ObjFeatures& features) {
  GenerateObjFile(road_geometry, dirname, fileroot, features);

  const std::string obj_filename = fileroot + ".obj";
  const std::string urdf_filename = fileroot + ".urdf";

  std::ofstream os(dirname + "/" + urdf_filename, std::ios::binary);
  os << "<?xml version=\"1.0\" ?>\n"
     << "<robot name=\"" << road_geometry->id().id << "\">\n"
     << "  <link name=\"world\"/>\n"
     << "\n"
     << "  <joint name=\"world_to_road_joint\" type=\"continuous\">\n"
     << "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
     << "    <parent link=\"world\"/>\n"
     << "    <child link=\"surface\"/>\n"
     << "  </joint>\n"
     << "\n"
     << "  <link name=\"surface\">\n"
     << "    <inertial/>\n"
     << "    <visual name=\"v1\">\n"
     << "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
     << "      <geometry>\n"
     << "        <mesh filename=\""
     << obj_filename << "\" scale=\"1.0 1.0 1.0\"/>\n"
     << "      </geometry>\n"
     << "    </visual>\n"
     << "  </link>\n"
     << "</robot>\n";
}


}  // namespace utility
}  // namespace maliput
}  // namespace drake
