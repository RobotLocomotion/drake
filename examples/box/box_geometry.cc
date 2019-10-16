#include "drake/examples/box/box_geometry.h"

#include <memory>
#include <utility>

#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"


namespace drake {
namespace examples {
namespace box {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector4d;
using geometry::Box;
using geometry::Cylinder;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::MakePhongIllustrationProperties;
using geometry::Sphere;
using std::make_unique;
// CGS units
//#define CGS
#ifdef CGS
const double LENGTH_SCALE = 0.3;
const double DENSITY = 1.07; /* lego ABS plastic */
#else
// m, kg, s
const double LENGTH_SCALE = 3.;
const double DENSITY = 1070.;
#endif

template <>
void BoxGeometryTemplate<double>::OutputGeometryPose(
    const systems::Context<double>& context,
    geometry::FramePoseVector<double>* poses) const {
  DRAKE_DEMAND(frame_id_.is_valid());

  auto& input = this->get_input_port(0).Eval<drake::systems::BasicVector<double>>(context);
  // TODO: change this to a translation
  const double q = input[0] ;
  //std::cout << "Geom: " << frame_id_ << " " << q * LENGTH_SCALE << std::endl;
  math::RigidTransformd pose(Eigen::Vector3d(LENGTH_SCALE * q, 0.,0.));

  *poses = {{frame_id_, pose}};
}

template <typename T>
const BoxGeometryTemplate<T>* BoxGeometryTemplate<T>::AddToBuilder(
    systems::DiagramBuilder<T>* builder,
    const BoxPlant<T>& box, 
    const systems::OutputPort<T>& box_state_output_port,
    geometry::SceneGraph<T>* scene_graph, 
    std::string srcName) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);

  auto box_geometry = builder->AddSystem(
      std::unique_ptr<BoxGeometryTemplate<T>>(
          new BoxGeometryTemplate<T>(scene_graph, box, srcName)));
  // input is state, output is geometry
  builder->Connect(
      box_state_output_port,
      box_geometry->get_input_port(0));
  builder->Connect(
      box_geometry->get_output_port(0),
      scene_graph->get_source_pose_port(box_geometry->source_id_));

  return box_geometry;
}
template <typename T>
BoxGeometryTemplate<T>::BoxGeometryTemplate(geometry::SceneGraph<T>* scene_graph, const BoxPlant<T>& box, std::string srcName)
  : systems::LeafSystem<T>(systems::SystemTypeTag<box::BoxGeometryTemplate>{}), scene_graph_(scene_graph)
  {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  std::string boxname = "box" + srcName;
  source_id_ = scene_graph->RegisterSource( boxname );
  frame_id_ = scene_graph->RegisterFrame(source_id_, GeometryFrame("boxstate" + srcName));

  this->DeclareVectorInputPort("state", drake::systems::BasicVector<T>(2));
  this->DeclareAbstractOutputPort(
      "geometry_pose", &BoxGeometryTemplate<T>::OutputGeometryPose);


  const double length = box.get_length();
  double volume = 0.33 / DENSITY; /* hack to initialize it to coco water size */
  if (box.get_inv_mass() != 0.) 
     volume = 1.0 / (box.get_inv_mass() * DENSITY);
  using std::sqrt;

  // The base.
  GeometryId id = scene_graph->RegisterGeometry(
      source_id_,frame_id_,
      make_unique<GeometryInstance>(math::RigidTransformd(Translation3d(0., 0., 0.5 * LENGTH_SCALE * sqrt(volume / length))),
                                    make_unique<Box>(LENGTH_SCALE * length, 
                                    LENGTH_SCALE * sqrt(volume / length), 
                                    LENGTH_SCALE * sqrt(volume / length)), boxname));
  if( srcName == "1" )
    scene_graph->AssignRole( /* green */
      source_id_, id, MakePhongIllustrationProperties(Vector4d(.3, .6, .4, 1)));
  else if (srcName == "4" )
    scene_graph->AssignRole( /* red */
      source_id_, id, MakePhongIllustrationProperties(Vector4d(.8, .2, .2, 1)));
  else if (srcName == "3" )
    scene_graph->AssignRole( /* dark red */
      source_id_, id, MakePhongIllustrationProperties(Vector4d(.4, .1, .1, 1)));
  else
    scene_graph->AssignRole( /* purple */
      source_id_, id, MakePhongIllustrationProperties(Vector4d(.4, .3, .6, 1)));
}

template <typename T>
template <typename U>
  BoxGeometryTemplate<T>::BoxGeometryTemplate(const BoxGeometryTemplate<U>& other) 
  : systems::LeafSystem<T>(systems::SystemTypeTag<box::BoxGeometryTemplate>{})
  {
    unused(other);
    
    this->DeclareVectorInputPort("state", drake::systems::BasicVector<T>(2));
    this->DeclareAbstractOutputPort(
        "geometry_pose", &BoxGeometryTemplate<T>::OutputGeometryPose);
    
  }

template <typename T>
BoxGeometryTemplate<T>::~BoxGeometryTemplate() = default;

template <typename T>
void BoxGeometryTemplate<T>::OutputGeometryPose(
    const systems::Context<T>& ,
    geometry::FramePoseVector<T>* poses) const {
  *poses = {{frame_id_, math::RigidTransform<T>()}};
}
}  // namespace box
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::examples::box::BoxGeometryTemplate)
