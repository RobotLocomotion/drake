
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

py::module apb11_pydrake_drake_py_register(py::module &model);
py::module apb11_pydrake_math_py_register(py::module &model);
py::module apb11_pydrake_lcm_py_register(py::module &model);
py::module apb11_pydrake_perception_py_register(py::module &model);
py::module apb11_pydrake_pc_flags_py_register(py::module &model);
py::module apb11_pydrake_symbolic_py_register(py::module &model);
py::module apb11_pydrake_systems_py_register(py::module &model);
py::module apb11_pydrake_internal_py_register(py::module &model);
py::module apb11_pydrake_geometry_py_register(py::module &model);
py::module apb11_pydrake_render_py_register(py::module &model);
void apb11_pydrake_AutoDiffXd_py_register(py::module &model);
void apb11_pydrake_RandomGenerator_py_register(py::module &model);
void apb11_pydrake_ClippingRange_py_register(py::module &model);
void apb11_pydrake_ColorRenderCamera_py_register(py::module &model);
void apb11_pydrake_DepthRange_py_register(py::module &model);
void apb11_pydrake_DepthRenderCamera_py_register(py::module &model);
void apb11_pydrake_RenderCameraCore_py_register(py::module &model);
void apb11_pydrake_DrakeLcmInterface_py_register(py::module &model);
void apb11_pydrake_DrakeSubscriptionInterface_py_register(py::module &model);
void apb11_pydrake_BarycentricMesh_py_register(py::module &model);
void apb11_pydrake_RollPitchYaw_py_register(py::module &model);
void apb11_pydrake_RotationMatrix_py_register(py::module &model);
void apb11_pydrake_PointCloud_py_register(py::module &model);
void apb11_pydrake_Fields_py_register(py::module &model);
void apb11_pydrake_Environment_py_register(py::module &model);
void apb11_pydrake_Expression_py_register(py::module &model);
void apb11_pydrake_Formula_py_register(py::module &model);
void apb11_pydrake_Monomial_py_register(py::module &model);
void apb11_pydrake_Variable_py_register(py::module &model);
void apb11_pydrake_SystemMessageInterface_py_register(py::module &model);
void apb11_pydrake_RenderEngine_py_register(py::module &model);
void apb11_pydrake_DrakeLcm_py_register(py::module &model);
void apb11_pydrake_SystemBase_py_register(py::module &model);
void apb11_pydrake_DrakeMockLcm_py_register(py::module &model);
void apb11_pydrake_LeafSystem_py_register(py::module &model);
void apb11_pydrake_System_py_register(py::module &model);
void apb11_pydrake_DepthImageToPointCloud_py_register(py::module &model);

PYBIND11_EXPORT void apb11_pydrake_register_types(py::module &model) {
  // make sure this module is only initialized once
  static bool called = false;
  if (called) {
    return;
  }
  called = true;

  // initialize class for module pydrake
  auto drake = apb11_pydrake_drake_py_register(model);
  auto math = apb11_pydrake_math_py_register(drake);
  auto lcm = apb11_pydrake_lcm_py_register(drake);
  auto perception = apb11_pydrake_perception_py_register(drake);
  auto pc_flags = apb11_pydrake_pc_flags_py_register(perception);
  auto symbolic = apb11_pydrake_symbolic_py_register(drake);
  auto systems = apb11_pydrake_systems_py_register(drake);
  auto internal = apb11_pydrake_internal_py_register(systems);
  auto geometry = apb11_pydrake_geometry_py_register(drake);
  auto render = apb11_pydrake_render_py_register(geometry);
  apb11_pydrake_AutoDiffXd_py_register(math);
  apb11_pydrake_RandomGenerator_py_register(drake);
  apb11_pydrake_ClippingRange_py_register(render);
  apb11_pydrake_ColorRenderCamera_py_register(render);
  apb11_pydrake_DepthRange_py_register(render);
  apb11_pydrake_DepthRenderCamera_py_register(render);
  apb11_pydrake_RenderCameraCore_py_register(render);
  apb11_pydrake_DrakeLcmInterface_py_register(lcm);
  apb11_pydrake_DrakeSubscriptionInterface_py_register(lcm);
  apb11_pydrake_BarycentricMesh_py_register(math);
  apb11_pydrake_RollPitchYaw_py_register(math);
  apb11_pydrake_RotationMatrix_py_register(math);
  apb11_pydrake_PointCloud_py_register(perception);
  apb11_pydrake_Fields_py_register(pc_flags);
  apb11_pydrake_Environment_py_register(symbolic);
  apb11_pydrake_Expression_py_register(symbolic);
  apb11_pydrake_Formula_py_register(symbolic);
  apb11_pydrake_Monomial_py_register(symbolic);
  apb11_pydrake_Variable_py_register(symbolic);
  apb11_pydrake_SystemMessageInterface_py_register(internal);
  apb11_pydrake_RenderEngine_py_register(render);
  apb11_pydrake_DrakeLcm_py_register(lcm);
  apb11_pydrake_SystemBase_py_register(systems);
  apb11_pydrake_DrakeMockLcm_py_register(lcm);
  apb11_pydrake_LeafSystem_py_register(systems);
  apb11_pydrake_System_py_register(systems);
  apb11_pydrake_DepthImageToPointCloud_py_register(perception);
};

PYBIND11_MODULE(pydrake, model) {
  // First initialize dependent modules
  // then this module.
  apb11_pydrake_register_types(model);
}
