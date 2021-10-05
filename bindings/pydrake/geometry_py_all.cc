/* @file The contains all of the public entities found in the
 drake::geometry::optimization namespace. They can be found in the
 pydrake.geometry.optimization module. */

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/geometry_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_properties.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/render/gl_renderer/render_engine_gl_factory.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"

// TODO(SeanCurtis-TRI) When pybind issue 3019 gets resolved, we won't need to
//  define this locally anymore. In fact, it will probably cause link errors.
namespace pybind11 {
namespace detail {
template <>
struct type_caster<std::monostate> {
 public:
  PYBIND11_TYPE_CASTER(std::monostate, _("None"));

  bool load(handle src, bool) { return src.ptr() == Py_None; }

  static handle cast(
      std::monostate, return_value_policy /* policy */, handle /* parent */) {
    Py_RETURN_NONE;
  }
};
}  // namespace detail
}  // namespace pybind11

namespace drake {
namespace pydrake {

  // DrakeVisualizer
  {
    using Class = DrakeVisualizer<T>;
    constexpr auto& cls_doc = doc.DrakeVisualizer;
    auto cls = DefineTemplateClassWithDefault<Class, LeafSystem<T>>(
        m, "DrakeVisualizer", param, cls_doc.doc);
    cls  // BR
        .def(py::init<lcm::DrakeLcmInterface*, DrakeVisualizerParams>(),
            py::arg("lcm") = nullptr,
            py::arg("params") = DrakeVisualizerParams{},
            // Keep alive, reference: `self` keeps `lcm` alive.
            py::keep_alive<1, 2>(),  // BR
            cls_doc.ctor.doc)
        .def("query_object_input_port", &Class::query_object_input_port,
            py_rvp::reference_internal, cls_doc.query_object_input_port.doc)
        .def_static("AddToBuilder",
            py::overload_cast<systems::DiagramBuilder<T>*, const SceneGraph<T>&,
                lcm::DrakeLcmInterface*, DrakeVisualizerParams>(
                &DrakeVisualizer<T>::AddToBuilder),
            py::arg("builder"), py::arg("scene_graph"),
            py::arg("lcm") = nullptr,
            py::arg("params") = DrakeVisualizerParams{},
            // Keep alive, ownership: `return` keeps `builder` alive.
            py::keep_alive<0, 1>(),
            // Keep alive, reference: `builder` keeps `lcm` alive.
            py::keep_alive<1, 3>(), py_rvp::reference,
            cls_doc.AddToBuilder.doc_4args_builder_scene_graph_lcm_params)
        .def_static("AddToBuilder",
            py::overload_cast<systems::DiagramBuilder<T>*,
                const systems::OutputPort<T>&, lcm::DrakeLcmInterface*,
                DrakeVisualizerParams>(&DrakeVisualizer<T>::AddToBuilder),
            py::arg("builder"), py::arg("query_object_port"),
            py::arg("lcm") = nullptr,
            py::arg("params") = DrakeVisualizerParams{},
            // Keep alive, ownership: `return` keeps `builder` alive.
            py::keep_alive<0, 1>(),
            // Keep alive, reference: `builder` keeps `lcm` alive.
            py::keep_alive<1, 3>(), py_rvp::reference,
            cls_doc.AddToBuilder.doc_4args_builder_query_object_port_lcm_params)
        .def_static("DispatchLoadMessage",
            &DrakeVisualizer<T>::DispatchLoadMessage, py::arg("scene_graph"),
            py::arg("lcm"), py::arg("params") = DrakeVisualizerParams{},
            cls_doc.DispatchLoadMessage.doc);
  }

  // MeshcatVisualizer
  {
    using Class = MeshcatVisualizer<T>;
    constexpr auto& cls_doc = doc.MeshcatVisualizer;
    // Note that we are temporarily re-mapping MeshcatVisualizer =>
    // MeshcatVisualizerCpp to avoid collisions with the python
    // MeshcatVisualizer.  See #13038.
    auto cls = DefineTemplateClassWithDefault<Class, LeafSystem<T>>(
        m, "MeshcatVisualizerCpp", param, cls_doc.doc);
    cls  // BR
        .def(py::init<std::shared_ptr<Meshcat>, MeshcatVisualizerParams>(),
            py::arg("meshcat"), py::arg("params") = MeshcatVisualizerParams{},
            // `meshcat` is a shared_ptr, so does not need a keep_alive.
            cls_doc.ctor.doc)
        .def("Delete", &Class::Delete, cls_doc.Delete.doc)
        .def("query_object_input_port", &Class::query_object_input_port,
            py_rvp::reference_internal, cls_doc.query_object_input_port.doc)
        .def_static("AddToBuilder",
            py::overload_cast<systems::DiagramBuilder<T>*, const SceneGraph<T>&,
                std::shared_ptr<Meshcat>, MeshcatVisualizerParams>(
                &MeshcatVisualizer<T>::AddToBuilder),
            py::arg("builder"), py::arg("scene_graph"), py::arg("meshcat"),
            py::arg("params") = MeshcatVisualizerParams{},
            // Keep alive, ownership: `return` keeps `builder` alive.
            py::keep_alive<0, 1>(),
            // `meshcat` is a shared_ptr, so does not need a keep_alive.
            py_rvp::reference,
            cls_doc.AddToBuilder.doc_4args_builder_scene_graph_meshcat_params)
        .def_static("AddToBuilder",
            py::overload_cast<systems::DiagramBuilder<T>*,
                const systems::OutputPort<T>&, std::shared_ptr<Meshcat>,
                MeshcatVisualizerParams>(&MeshcatVisualizer<T>::AddToBuilder),
            py::arg("builder"), py::arg("query_object_port"),
            py::arg("meshcat"), py::arg("params") = MeshcatVisualizerParams{},
            // Keep alive, ownership: `return` keeps `builder` alive.
            py::keep_alive<0, 1>(),
            // `meshcat` is a shared_ptr, so does not need a keep_alive.
            py_rvp::reference,
            cls_doc.AddToBuilder
                .doc_4args_builder_query_object_port_meshcat_params);
  }

  // VolumeMesh
  {
    using Class = VolumeMesh<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "VolumeMesh", param, doc.VolumeMesh.doc);
    cls  // BR
        .def(py::init<std::vector<VolumeElement>,
                 std::vector<VolumeVertex<T>>>(),
            py::arg("elements"), py::arg("vertices"), doc.VolumeMesh.ctor.doc)
        .def("vertices", &Class::vertices, py_rvp::reference_internal,
            doc.VolumeMesh.vertices.doc)
        .def("tetrahedra", &Class::tetrahedra, py_rvp::reference_internal,
            doc.VolumeMesh.tetrahedra.doc)
        .def("CalcTetrahedronVolume", &Class::CalcTetrahedronVolume,
            py::arg("e"), doc.VolumeMesh.CalcTetrahedronVolume.doc)
        .def("CalcVolume", &Class::CalcVolume, doc.VolumeMesh.CalcVolume.doc);
  }

  // VolumeVertex
  {
    using Class = VolumeVertex<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "VolumeVertex", param, doc.VolumeVertex.doc);
    cls  // BR
        .def(py::init<const Vector3<T>&>(), py::arg("r_MV"),
            doc.VolumeVertex.ctor.doc_1args)
        .def("r_MV", &Class::r_MV, py_rvp::reference_internal,
            doc.VolumeVertex.r_MV.doc);
  }

  m.def("ConvertVolumeToSurfaceMesh", &ConvertVolumeToSurfaceMesh<T>,
      py::arg("volume"), doc.ConvertVolumeToSurfaceMesh.doc);
}  // NOLINT(readability/fn_size)

void DoScalarIndependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  constexpr auto& doc = pydrake_doc.drake.geometry;

  // All the index types up front, so they'll be available to every other type.
  {
    BindTypeSafeIndex<SurfaceVertexIndex>(
        m, "SurfaceVertexIndex", doc.SurfaceVertexIndex.doc);
    BindTypeSafeIndex<SurfaceFaceIndex>(
        m, "SurfaceFaceIndex", doc.SurfaceFaceIndex.doc);
    BindTypeSafeIndex<VolumeVertexIndex>(
        m, "VolumeVertexIndex", doc.VolumeVertexIndex.doc);
    BindTypeSafeIndex<VolumeElementIndex>(
        m, "VolumeElementIndex", doc.VolumeElementIndex.doc);
  }

  // Rgba
  {
    using Class = Rgba;
    constexpr auto& cls_doc = doc.Rgba;
    py::class_<Class> cls(m, "Rgba", cls_doc.doc);
    cls  // BR
        .def(py::init<double, double, double, double>(), py::arg("r"),
            py::arg("g"), py::arg("b"), py::arg("a") = 1., cls_doc.ctor.doc)
        .def("r", &Class::r, cls_doc.r.doc)
        .def("g", &Class::g, cls_doc.g.doc)
        .def("b", &Class::b, cls_doc.b.doc)
        .def("a", &Class::a, cls_doc.a.doc)
        .def("set", &Class::set, py::arg("r"), py::arg("g"), py::arg("b"),
            py::arg("a") = 1., cls_doc.set.doc)
        .def(py::self == py::self)
        .def(py::self != py::self)
        .def("__repr__", [](const Class& self) {
          return py::str("Rgba(r={}, g={}, b={}, a={})")
              .format(self.r(), self.g(), self.b(), self.a());
        });
    DefCopyAndDeepCopy(&cls);
    AddValueInstantiation<Rgba>(m);
  }

  BindIdentifier<FilterId>(m, "FilterId", doc.FilterId.doc);
  BindIdentifier<SourceId>(m, "SourceId", doc.SourceId.doc);
  BindIdentifier<FrameId>(m, "FrameId", doc.FrameId.doc);
  BindIdentifier<GeometryId>(m, "GeometryId", doc.GeometryId.doc);

  // CollisionFilterDeclaration.
  {
    using Class = CollisionFilterDeclaration;
    constexpr auto& cls_doc = doc.CollisionFilterDeclaration;

    py::class_<Class>(m, "CollisionFilterDeclaration", cls_doc.doc)
        .def(py::init(), cls_doc.ctor.doc)
        .def("AllowBetween", &Class::AllowBetween, py::arg("set_A"),
            py::arg("set_B"), py_rvp::reference, cls_doc.AllowBetween.doc)
        .def("AllowWithin", &Class::AllowWithin, py::arg("geometry_set"),
            py_rvp::reference, cls_doc.AllowWithin.doc)
        .def("ExcludeBetween", &Class::ExcludeBetween, py::arg("set_A"),
            py::arg("set_B"), py_rvp::reference, cls_doc.ExcludeBetween.doc)
        .def("ExcludeWithin", &Class::ExcludeWithin, py::arg("geometry_set"),
            py_rvp::reference, cls_doc.ExcludeWithin.doc);
  }

  //  CollisionFilterManager
  {
    using Class = CollisionFilterManager;
    constexpr auto& cls_doc = doc.CollisionFilterManager;
    py::class_<Class>(m, "CollisionFilterManager", cls_doc.doc)
        .def("Apply", &Class::Apply, py::arg("declaration"), cls_doc.Apply.doc)
        .def("ApplyTransient", &Class::ApplyTransient, py::arg("declaration"),
            cls_doc.ApplyTransient.doc)
        .def("RemoveDeclaration", &Class::RemoveDeclaration,
            py::arg("filter_id"), cls_doc.RemoveDeclaration.doc)
        .def("has_transient_history", &Class::has_transient_history,
            cls_doc.has_transient_history.doc)
        .def("IsActive", &Class::IsActive, py::arg("filter_id"),
            cls_doc.IsActive.doc);
  }

  // Role enumeration
  {
    constexpr auto& cls_doc = doc.Role;
    py::enum_<Role>(m, "Role", py::arithmetic(), cls_doc.doc)
        .value("kUnassigned", Role::kUnassigned, cls_doc.kUnassigned.doc)
        .value("kProximity", Role::kProximity, cls_doc.kProximity.doc)
        .value("kIllustration", Role::kIllustration, cls_doc.kIllustration.doc)
        .value("kPerception", Role::kPerception, cls_doc.kPerception.doc);
  }

  // RoleAssign enumeration
  {
    constexpr auto& cls_doc = doc.RoleAssign;
    using Class = RoleAssign;
    py::enum_<Class>(m, "RoleAssign", cls_doc.doc)
        .value("kNew", Class::kNew, cls_doc.kNew.doc)
        .value("kReplace", Class::kReplace, cls_doc.kReplace.doc);
  }

  // DrakeVisualizerParams
  {
    using Class = DrakeVisualizerParams;
    constexpr auto& cls_doc = doc.DrakeVisualizerParams;
    py::class_<Class>(
        m, "DrakeVisualizerParams", py::dynamic_attr(), cls_doc.doc)
        .def(ParamInit<Class>())
        .def_readwrite("publish_period", &DrakeVisualizerParams::publish_period,
            cls_doc.publish_period.doc)
        .def_readwrite("role", &DrakeVisualizerParams::role, cls_doc.role.doc)
        .def_readwrite("default_color", &DrakeVisualizerParams::default_color,
            cls_doc.default_color.doc)
        .def_readwrite("show_hydroelastic",
            &DrakeVisualizerParams::show_hydroelastic,
            cls_doc.show_hydroelastic.doc)
        .def("__repr__", [](const Class& self) {
          return py::str(
              "DrakeVisualizerParams("
              "publish_period={}, "
              "role={}, "
              "default_color={}, "
              "show_hydroelastic={})")
              .format(self.publish_period, self.role, self.default_color,
                  self.show_hydroelastic);
        });
  }

  // Shape constructors
  {
    py::class_<Shape> shape_cls(m, "Shape", doc.Shape.doc);
    DefClone(&shape_cls);
    py::class_<Sphere, Shape>(m, "Sphere", doc.Sphere.doc)
        .def(py::init<double>(), py::arg("radius"), doc.Sphere.ctor.doc)
        .def("radius", &Sphere::radius, doc.Sphere.radius.doc)
        .def(py::pickle([](const Sphere& self) { return self.radius(); },
            [](const double radius) { return Sphere(radius); }));
    py::class_<Cylinder, Shape>(m, "Cylinder", doc.Cylinder.doc)
        .def(py::init<double, double>(), py::arg("radius"), py::arg("length"),
            doc.Cylinder.ctor.doc)
        .def("radius", &Cylinder::radius, doc.Cylinder.radius.doc)
        .def("length", &Cylinder::length, doc.Cylinder.length.doc)
        .def(py::pickle(
            [](const Cylinder& self) {
              return std::make_pair(self.radius(), self.length());
            },
            [](std::pair<double, double> dims) {
              return Cylinder(dims.first, dims.second);
            }));
    py::class_<Box, Shape>(m, "Box", doc.Box.doc)
        .def(py::init<double, double, double>(), py::arg("width"),
            py::arg("depth"), py::arg("height"), doc.Box.ctor.doc)
        .def("width", &Box::width, doc.Box.width.doc)
        .def("depth", &Box::depth, doc.Box.depth.doc)
        .def("height", &Box::height, doc.Box.height.doc)
        .def("size", &Box::size, py_rvp::reference_internal, doc.Box.size.doc)
        .def(py::pickle(
            [](const Box& self) {
              return std::make_tuple(self.width(), self.depth(), self.height());
            },
            [](std::tuple<double, double, double> dims) {
              return Box(
                  std::get<0>(dims), std::get<1>(dims), std::get<2>(dims));
            }));
    py::class_<Capsule, Shape>(m, "Capsule", doc.Capsule.doc)
        .def(py::init<double, double>(), py::arg("radius"), py::arg("length"),
            doc.Capsule.ctor.doc)
        .def("radius", &Capsule::radius, doc.Capsule.radius.doc)
        .def("length", &Capsule::length, doc.Capsule.length.doc)
        .def(py::pickle(
            [](const Capsule& self) {
              return std::make_pair(self.radius(), self.length());
            },
            [](std::pair<double, double> dims) {
              return Capsule(dims.first, dims.second);
            }));
    py::class_<Ellipsoid, Shape>(m, "Ellipsoid", doc.Ellipsoid.doc)
        .def(py::init<double, double, double>(), py::arg("a"), py::arg("b"),
            py::arg("c"), doc.Ellipsoid.ctor.doc)
        .def("a", &Ellipsoid::a, doc.Ellipsoid.a.doc)
        .def("b", &Ellipsoid::b, doc.Ellipsoid.b.doc)
        .def("c", &Ellipsoid::c, doc.Ellipsoid.c.doc)
        .def(py::pickle(
            [](const Ellipsoid& self) {
              return std::make_tuple(self.a(), self.b(), self.c());
            },
            [](std::tuple<double, double, double> dims) {
              return Ellipsoid(
                  std::get<0>(dims), std::get<1>(dims), std::get<2>(dims));
            }));
    py::class_<HalfSpace, Shape>(m, "HalfSpace", doc.HalfSpace.doc)
        .def(py::init<>(), doc.HalfSpace.ctor.doc)
        .def_static("MakePose", &HalfSpace::MakePose, py::arg("Hz_dir_F"),
            py::arg("p_FB"), doc.HalfSpace.MakePose.doc);
    py::class_<Mesh, Shape>(m, "Mesh", doc.Mesh.doc)
        .def(py::init<std::string, double>(), py::arg("absolute_filename"),
            py::arg("scale") = 1.0, doc.Mesh.ctor.doc)
        .def("filename", &Mesh::filename, doc.Mesh.filename.doc)
        .def("scale", &Mesh::scale, doc.Mesh.scale.doc)
        .def(py::pickle(
            [](const Mesh& self) {
              return std::make_pair(self.filename(), self.scale());
            },
            [](std::pair<std::string, double> info) {
              return Mesh(info.first, info.second);
            }));
    py::class_<Convex, Shape>(m, "Convex", doc.Convex.doc)
        .def(py::init<std::string, double>(), py::arg("absolute_filename"),
            py::arg("scale") = 1.0, doc.Convex.ctor.doc)
        .def("filename", &Convex::filename, doc.Convex.filename.doc)
        .def("scale", &Convex::scale, doc.Convex.scale.doc)
        .def(py::pickle(
            [](const Convex& self) {
              return std::make_pair(self.filename(), self.scale());
            },
            [](std::pair<std::string, double> info) {
              return Convex(info.first, info.second);
            }));
  }

  // GeometryFrame
  {
    using Class = GeometryFrame;
    constexpr auto& cls_doc = doc.GeometryFrame;
    py::class_<Class> cls(m, "GeometryFrame", cls_doc.doc);
    cls  // BR
        .def(py::init<const std::string&, int>(), py::arg("frame_name"),
            py::arg("frame_group_id") = 0, cls_doc.ctor.doc)
        .def("id", &Class::id, cls_doc.id.doc)
        .def("name", &Class::name, cls_doc.name.doc)
        .def("frame_group", &Class::frame_group, cls_doc.frame_group.doc);
    DefCopyAndDeepCopy(&cls);
  }

  // GeometryInstance
  {
    using Class = GeometryInstance;
    constexpr auto& cls_doc = doc.GeometryInstance;
    py::class_<Class> cls(m, "GeometryInstance", cls_doc.doc);
    cls  // BR
        .def(py::init<const math::RigidTransform<double>&,
                 std::unique_ptr<Shape>, const std::string&>(),
            py::arg("X_PG"), py::arg("shape"), py::arg("name"),
            cls_doc.ctor.doc)
        .def("id", &Class::id, cls_doc.id.doc)
        .def("pose", &Class::pose, py_rvp::reference_internal, cls_doc.pose.doc)
        .def(
            "set_pose", &Class::set_pose, py::arg("X_PG"), cls_doc.set_pose.doc)
        .def("shape", &Class::shape, py_rvp::reference_internal,
            cls_doc.shape.doc)
        .def("release_shape", &Class::release_shape, cls_doc.release_shape.doc)
        .def("name", &Class::name, cls_doc.name.doc)
        .def("set_name", &Class::set_name, cls_doc.set_name.doc)
        .def("set_proximity_properties", &Class::set_proximity_properties,
            py::arg("properties"), cls_doc.set_proximity_properties.doc)
        .def("set_illustration_properties", &Class::set_illustration_properties,
            py::arg("properties"), cls_doc.set_illustration_properties.doc)
        .def("set_perception_properties", &Class::set_perception_properties,
            py::arg("properties"), cls_doc.set_perception_properties.doc)
        .def("mutable_proximity_properties",
            &Class::mutable_proximity_properties, py_rvp::reference_internal,
            cls_doc.mutable_proximity_properties.doc)
        .def("proximity_properties", &Class::proximity_properties,
            py_rvp::reference_internal, cls_doc.proximity_properties.doc)
        .def("mutable_illustration_properties",
            &Class::mutable_illustration_properties, py_rvp::reference_internal,
            cls_doc.mutable_illustration_properties.doc)
        .def("illustration_properties", &Class::illustration_properties,
            py_rvp::reference_internal, cls_doc.illustration_properties.doc)
        .def("mutable_perception_properties",
            &Class::mutable_perception_properties, py_rvp::reference_internal,
            cls_doc.mutable_perception_properties.doc)
        .def("perception_properties", &Class::perception_properties,
            py_rvp::reference_internal, cls_doc.perception_properties.doc);
    DefCopyAndDeepCopy(&cls);
  }

  // GeometryProperties
  {
    using Class = GeometryProperties;
    constexpr auto& cls_doc = doc.GeometryProperties;
    py::handle abstract_value_cls =
        py::module::import("pydrake.common.value").attr("AbstractValue");
    py::class_<Class>(m, "GeometryProperties", cls_doc.doc)
        .def("HasGroup", &Class::HasGroup, py::arg("group_name"),
            cls_doc.HasGroup.doc)
        .def("num_groups", &Class::num_groups, cls_doc.num_groups.doc)
        .def(
            "GetPropertiesInGroup",
            [](const Class& self, const std::string& group_name) {
              py::dict out;
              py::object py_self = py::cast(&self, py_rvp::reference);
              for (auto& [name, abstract] :
                  self.GetPropertiesInGroup(group_name)) {
                out[name.c_str()] = py::cast(
                    abstract.get(), py_rvp::reference_internal, py_self);
              }
              return out;
            },
            py::arg("group_name"), cls_doc.GetPropertiesInGroup.doc)
        .def("GetGroupNames", &Class::GetGroupNames, cls_doc.GetGroupNames.doc)
        .def(
            "AddProperty",
            [abstract_value_cls](Class& self, const std::string& group_name,
                const std::string& name, py::object value) {
              py::object abstract = abstract_value_cls.attr("Make")(value);
              self.AddPropertyAbstract(
                  group_name, name, abstract.cast<const AbstractValue&>());
            },
            py::arg("group_name"), py::arg("name"), py::arg("value"),
            cls_doc.AddProperty.doc)
        .def(
            "UpdateProperty",
            [abstract_value_cls](Class& self, const std::string& group_name,
                const std::string& name, py::object value) {
              py::object abstract = abstract_value_cls.attr("Make")(value);
              self.UpdatePropertyAbstract(
                  group_name, name, abstract.cast<const AbstractValue&>());
            },
            py::arg("group_name"), py::arg("name"), py::arg("value"),
            cls_doc.UpdateProperty.doc)
        .def("HasProperty", &Class::HasProperty, py::arg("group_name"),
            py::arg("name"), cls_doc.HasProperty.doc)
        .def(
            "GetProperty",
            [](const Class& self, const std::string& group_name,
                const std::string& name) {
              py::object abstract =
                  py::cast(self.GetPropertyAbstract(group_name, name),
                      py_rvp::reference);
              return abstract.attr("get_value")();
            },
            py::arg("group_name"), py::arg("name"), cls_doc.GetProperty.doc)
        .def(
            "GetPropertyOrDefault",
            [](const Class& self, const std::string& group_name,
                const std::string& name, py::object default_value) {
              // For now, ignore typing. This is less efficient, but eh, it's
              // Python.
              if (self.HasProperty(group_name, name)) {
                py::object py_self = py::cast(&self, py_rvp::reference);
                return py_self.attr("GetProperty")(group_name, name);
              } else {
                return default_value;
              }
            },
            py::arg("group_name"), py::arg("name"), py::arg("default_value"),
            cls_doc.GetPropertyOrDefault.doc)
        .def("RemoveProperty", &Class::RemoveProperty, py::arg("group_name"),
            py::arg("name"), cls_doc.RemoveProperty.doc)
        .def_static("default_group_name", &Class::default_group_name,
            cls_doc.default_group_name.doc)
        .def(
            "__str__",
            [](const Class& self) {
              std::stringstream ss;
              ss << self;
              return ss.str();
            },
            "Returns formatted string.");
  }

  // GeometrySet
  {
    using Class = GeometrySet;
    constexpr auto& cls_doc = doc.GeometrySet;
    constexpr char extra_ctor_doc[] = "See main constructor";
    // N.B. For containers, we use `std::vector<>` rather than abstract
    // iterators / containers.
    py::class_<Class>(m, "GeometrySet", cls_doc.doc)
        .def(py::init(), cls_doc.ctor.doc)
        .def(py::init<GeometryId>(), py::arg("geometry_id"), extra_ctor_doc)
        .def(py::init<FrameId>(), py::arg("frame_id"), extra_ctor_doc)
        .def(py::init([](std::vector<GeometryId> geometry_ids) {
          return Class(geometry_ids);
        }),
            py::arg("geometry_ids"), extra_ctor_doc)
        .def(py::init([](std::vector<FrameId> frame_ids) {
          return Class(frame_ids);
        }),
            py::arg("frame_ids"), extra_ctor_doc)
        .def(py::init([](std::vector<GeometryId> geometry_ids,
                          std::vector<FrameId> frame_ids) {
          return Class(geometry_ids, frame_ids);
        }),
            py::arg("geometry_ids"), py::arg("frame_ids"), extra_ctor_doc)
        .def(
            "Add",
            [](Class* self, const GeometryId& geometry_id) {
              self->Add(geometry_id);
            },
            py::arg("geometry_id"), cls_doc.Add.doc)
        .def(
            "Add",
            [](Class* self, const FrameId& frame_id) { self->Add(frame_id); },
            py::arg("frame_id"), cls_doc.Add.doc)
        .def(
            "Add",
            [](Class* self, std::vector<GeometryId> geometry_ids) {
              self->Add(geometry_ids);
            },
            py::arg("geometry_ids"), extra_ctor_doc)
        .def(
            "Add",
            [](Class* self, std::vector<FrameId> frame_ids) {
              self->Add(frame_ids);
            },
            py::arg("frame_ids"), extra_ctor_doc)
        .def(
            "Add",
            [](Class* self, std::vector<GeometryId> geometry_ids,
                std::vector<FrameId> frame_ids) {
              self->Add(geometry_ids, frame_ids);
            },
            py::arg("geometry_ids"), py::arg("frame_ids"), extra_ctor_doc);
  }

  // GeometryVersion
  {
    using Class = GeometryVersion;
    constexpr auto& cls_doc = doc.GeometryVersion;
    py::class_<Class> cls(m, "GeometryVersion", cls_doc.doc);
    cls.def(py::init(), cls_doc.ctor.doc)
        .def(py::init<const GeometryVersion&>(), py::arg("other"),
            "Creates a copy of the GeometryVersion.")
        .def("IsSameAs", &Class::IsSameAs, py::arg("other"), py::arg("role"),
            cls_doc.IsSameAs.doc);
    DefCopyAndDeepCopy(&cls);
  }

  // ProximityProperties
  {
    py::class_<ProximityProperties, GeometryProperties> cls(
        m, "ProximityProperties", doc.ProximityProperties.doc);
    cls.def(py::init(), doc.ProximityProperties.ctor.doc)
        .def(py::init<const ProximityProperties&>(), py::arg("other"),
            "Creates a copy of the properties");
    DefCopyAndDeepCopy(&cls);
  }

  // IllustrationProperties
  {
    py::class_<IllustrationProperties, GeometryProperties> cls(
        m, "IllustrationProperties", doc.IllustrationProperties.doc);
    cls.def(py::init(), doc.IllustrationProperties.ctor.doc)
        .def(py::init<const IllustrationProperties&>(), py::arg("other"),
            "Creates a copy of the properties");
    DefCopyAndDeepCopy(&cls);
  }

  // PerceptionProperties
  {
    py::class_<PerceptionProperties, GeometryProperties> cls(
        m, "PerceptionProperties", doc.PerceptionProperties.doc);
    cls.def(py::init(), doc.PerceptionProperties.ctor.doc)
        .def(py::init<const PerceptionProperties&>(), py::arg("other"),
            "Creates a copy of the properties");
    DefCopyAndDeepCopy(&cls);
  }

  // SurfaceFace
  {
    using Class = SurfaceFace;
    constexpr auto& cls_doc = doc.SurfaceFace;
    py::class_<Class> cls(m, "SurfaceFace", cls_doc.doc);
    cls  // BR
        .def(py::init<SurfaceVertexIndex, SurfaceVertexIndex,
                 SurfaceVertexIndex>(),
            py::arg("v0"), py::arg("v1"), py::arg("v2"), cls_doc.ctor.doc_3args)
        // TODO(SeanCurtis-TRI): Bind constructor that takes array of ints.
        .def("vertex", &Class::vertex, py::arg("i"), cls_doc.vertex.doc);
    DefCopyAndDeepCopy(&cls);
  }

  // VolumeElement
  {
    using Class = VolumeElement;
    constexpr auto& cls_doc = doc.VolumeElement;
    py::class_<Class> cls(m, "VolumeElement", cls_doc.doc);
    cls  // BR
        .def(py::init<VolumeVertexIndex, VolumeVertexIndex, VolumeVertexIndex,
                 VolumeVertexIndex>(),
            py::arg("v0"), py::arg("v1"), py::arg("v2"), py::arg("v3"),
            cls_doc.ctor.doc_4args)
        // TODO(SeanCurtis-TRI): Bind constructor that takes array of ints.
        .def("vertex", &Class::vertex, py::arg("i"), cls_doc.vertex.doc);
    DefCopyAndDeepCopy(&cls);
  }

  m.def("MakePhongIllustrationProperties", &MakePhongIllustrationProperties,
      py_rvp::reference_internal, py::arg("diffuse"),
      doc.MakePhongIllustrationProperties.doc);

  m.def(
      "ReadObjToSurfaceMesh",
      [](const std::string& filename, double scale) {
        return geometry::ReadObjToSurfaceMesh(filename, scale);
      },
      py::arg("filename"), py::arg("scale") = 1.0,
      // N.B. We have not bound the optional "on_warning" argument.
      doc.ReadObjToSurfaceMesh.doc_3args_filename_scale_on_warning);

  m.def("AddRigidHydroelasticProperties",
      py::overload_cast<double, ProximityProperties*>(
          &AddRigidHydroelasticProperties),
      py::arg("resolution_hint"), py::arg("properties"),
      doc.AddRigidHydroelasticProperties.doc_2args);

  m.def("AddRigidHydroelasticProperties",
      py::overload_cast<ProximityProperties*>(&AddRigidHydroelasticProperties),
      py::arg("properties"), doc.AddRigidHydroelasticProperties.doc_1args);

  m.def("AddSoftHydroelasticProperties",
      py::overload_cast<double, ProximityProperties*>(
          &AddSoftHydroelasticProperties),
      py::arg("resolution_hint"), py::arg("properties"),
      doc.AddSoftHydroelasticProperties.doc_2args);

  m.def("AddSoftHydroelasticProperties",
      py::overload_cast<ProximityProperties*>(&AddSoftHydroelasticProperties),
      py::arg("properties"), doc.AddSoftHydroelasticProperties.doc_1args);

  m.def("AddSoftHydroelasticPropertiesForHalfSpace",
      &AddSoftHydroelasticPropertiesForHalfSpace, py::arg("slab_thickness"),
      py::arg("properties"), doc.AddSoftHydroelasticPropertiesForHalfSpace.doc);

  m.def("AddContactMaterial",
      py::overload_cast<const std::optional<double>&,
          const std::optional<double>&, const std::optional<double>&,
          const std::optional<multibody::CoulombFriction<double>>&,
          ProximityProperties*>(&AddContactMaterial),
      py::arg("elastic_modulus") = std::nullopt,
      py::arg("dissipation") = std::nullopt,
      py::arg("point_stiffness") = std::nullopt,
      py::arg("friction") = std::nullopt, py::arg("properties"),
      doc.AddContactMaterial.doc_5args);

  // Meshcat
  {
    using Class = Meshcat;
    constexpr auto& cls_doc = doc.Meshcat;
    py::class_<Class, std::shared_ptr<Class>> cls(m, "Meshcat", cls_doc.doc);
    cls  // BR
        .def(py::init<const std::optional<int>&>(),
            py::arg("port") = std::nullopt, cls_doc.ctor.doc)
        .def("web_url", &Class::web_url, cls_doc.web_url.doc)
        .def("port", &Class::port, cls_doc.port.doc)
        .def("ws_url", &Class::ws_url, cls_doc.ws_url.doc)
        .def("SetObject",
            py::overload_cast<std::string_view, const Shape&, const Rgba&>(
                &Class::SetObject),
            py::arg("path"), py::arg("shape"),
            py::arg("rgba") = Rgba(.9, .9, .9, 1.), cls_doc.SetObject.doc_shape)
        .def("SetObject",
            py::overload_cast<std::string_view, const perception::PointCloud&,
                double, const Rgba&>(&Class::SetObject),
            py::arg("path"), py::arg("cloud"), py::arg("point_size") = 0.001,
            py::arg("rgba") = Rgba(.9, .9, .9, 1.), cls_doc.SetObject.doc_cloud)
        // TODO(russt): Bind SetCamera.
        .def("Set2dRenderMode", &Class::Set2dRenderMode,
            py::arg("X_WC") = RigidTransformd{Eigen::Vector3d{0, -1, 0}},
            py::arg("xmin") = -1.0, py::arg("xmax") = 1.0,
            py::arg("ymin") = -1.0, py::arg("ymax") = 1.0,
            cls_doc.Set2dRenderMode.doc)
        .def("ResetRenderMode", &Class::ResetRenderMode,
            cls_doc.ResetRenderMode.doc)
        .def("SetTransform", &Class::SetTransform, py::arg("path"),
            py::arg("X_ParentPath"), cls_doc.SetTransform.doc)
        .def("Delete", &Class::Delete, py::arg("path") = "", cls_doc.Delete.doc)
        .def("SetProperty",
            py::overload_cast<std::string_view, std::string, bool>(
                &Class::SetProperty),
            py::arg("path"), py::arg("property"), py::arg("value"),
            cls_doc.SetProperty.doc_bool)
        .def("SetProperty",
            py::overload_cast<std::string_view, std::string, double>(
                &Class::SetProperty),
            py::arg("path"), py::arg("property"), py::arg("value"),
            cls_doc.SetProperty.doc_double)
        .def("AddButton", &Class::AddButton, py::arg("name"),
            cls_doc.AddButton.doc)
        .def("GetButtonClicks", &Class::GetButtonClicks, py::arg("name"),
            cls_doc.GetButtonClicks.doc)
        .def("DeleteButton", &Class::DeleteButton, py::arg("name"),
            cls_doc.DeleteButton.doc)
        .def("AddSlider", &Class::AddSlider, py::arg("name"), py::arg("min"),
            py::arg("max"), py::arg("step"), py::arg("value"),
            cls_doc.AddSlider.doc)
        .def("SetSliderValue", &Class::SetSliderValue, py::arg("name"),
            py::arg("value"), cls_doc.SetSliderValue.doc)
        .def("GetSliderValue", &Class::GetSliderValue, py::arg("name"),
            cls_doc.GetSliderValue.doc)
        .def("DeleteSlider", &Class::DeleteSlider, py::arg("name"),
            cls_doc.DeleteSlider.doc)
        .def("DeleteAddedControls", &Class::DeleteAddedControls,
            cls_doc.DeleteAddedControls.doc);
    // Note: we intentionally do not bind the advanced methods (HasProperty and
    // GetPacked*) which were intended primarily for testing in C++.
  }

  // MeshcatVisualizerParams
  {
    using Class = MeshcatVisualizerParams;
    constexpr auto& cls_doc = doc.MeshcatVisualizerParams;
    py::class_<Class>(
        m, "MeshcatVisualizerParams", py::dynamic_attr(), cls_doc.doc)
        .def(ParamInit<Class>())
        .def_readwrite("publish_period",
            &MeshcatVisualizerParams::publish_period,
            cls_doc.publish_period.doc)
        .def_readwrite("role", &MeshcatVisualizerParams::role, cls_doc.role.doc)
        .def_readwrite("default_color", &MeshcatVisualizerParams::default_color,
            cls_doc.default_color.doc)
        .def_readwrite(
            "prefix", &MeshcatVisualizerParams::prefix, cls_doc.prefix.doc)
        .def_readwrite("delete_on_initialization_event",
            &MeshcatVisualizerParams::delete_on_initialization_event,
            cls_doc.delete_on_initialization_event.doc);
  }
}  // NOLINT(readability/fn_size)

void def_geometry(py::module m) {
  DoScalarIndependentDefinitions(m);
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      NonSymbolicScalarPack{});
}

// Test-only code.
namespace testing {
// For use with `test_geometry_properties_cpp_types`.
template <typename T>
void DefGetPropertyCpp(py::module m) {
  auto func = [](const geometry::GeometryProperties& properties,
                  const std::string& group, const std::string& name) {
    return properties.GetProperty<T>(group, name);
  };
  AddTemplateFunction(m, "GetPropertyCpp", func, GetPyParam<T>());
}

// For use with test_proximity_properties. The hydroelastic compliance type is
// internal. But we want to test that the compliance type has been successfully
// defined in set of properties. If we ever move HydroelasticType out of
// internal and bind it, we can eliminate this helper.
//
// Return true if the properties indicate soft compliance, false if rigid, and
// throws if the property isn't set at all (or set to undefined).
bool PropertiesIndicateSoftHydro(const geometry::ProximityProperties& props) {
  using geometry::internal::HydroelasticType;
  const HydroelasticType hydro_type =
      props.GetPropertyOrDefault(geometry::internal::kHydroGroup,
          geometry::internal::kComplianceType, HydroelasticType::kUndefined);
  if (hydro_type == HydroelasticType::kUndefined) {
    throw std::runtime_error("No specification of rigid or soft");
  }
  return hydro_type == HydroelasticType::kSoft;
}

void def_testing_module(py::module m) {
  // The get_constant_id() returns a fresh object every time, but always with
  // the same underlying get_value().
  const auto constant_id = geometry::FilterId::get_new_id();
  m.def("get_constant_id", [constant_id]() { return constant_id; });

  m.def("PropertiesIndicateSoftHydro", &PropertiesIndicateSoftHydro);

  // For use with `test_geometry_properties_cpp_types`.
  DefGetPropertyCpp<std::string>(m);
  DefGetPropertyCpp<bool>(m);
  DefGetPropertyCpp<double>(m);
}
}  // namespace testing

void def_geometry_all(py::module m) {
  py::dict vars = m.attr("__dict__");
  py::exec(
      "from pydrake.geometry import *\n"
      "from pydrake.geometry.render import *\n"
      "from pydrake.geometry.optimization import *\n",
      py::globals(), vars);
}

PYBIND11_MODULE(geometry, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  py::module::import("pydrake.common");
  py::module::import("pydrake.math");
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.lcm");

  def_geometry(m);
  def_geometry_render(m.def_submodule("render"));
  DefineGeometryOptimization(m.def_submodule("optimization"));
  def_geometry_all(m.def_submodule("all"));
  testing::def_testing_module(m.def_submodule("_testing"));
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
