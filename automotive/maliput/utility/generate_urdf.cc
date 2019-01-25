#include "drake/automotive/maliput/utility/generate_urdf.h"

#include <fstream>

#include "fmt/ostream.h"

#include "drake/automotive/maliput/utility/generate_obj.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace utility {

void GenerateUrdfFile(const api::RoadGeometry* road_geometry,
                      const std::string& dirpath,
                      const std::string& fileroot,
                      const ObjFeatures& features) {
  GenerateObjFile(road_geometry, dirpath, fileroot, features);

  const std::string obj_filename = fileroot + ".obj";
  const std::string urdf_filename = fileroot + ".urdf";

  std::ofstream os(dirpath + "/" + urdf_filename, std::ios::binary);
  fmt::print(os,
             R"X(<?xml version="1.0" ?>
<robot name="{0}">
  <link name="world"/>

  <joint name="world_to_road_joint" type="continuous">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="surface"/>
  </joint>

  <link name="surface">
    <visual name="v1">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="{1}" scale="1.0 1.0 1.0"/>
      </geometry>
    </visual>
  </link>
</robot>
)X",
             road_geometry->id().string(),
             obj_filename);
}


}  // namespace utility
}  // namespace maliput
}  // namespace drake
