#include "drake/systems/sensors/rgbd_sensor_async.h"

#include <future>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/rgbd_sensor.h"

/* Here's an outline of how RgbdSensorAsync is implemented.

There are two logical state variables (stored as a single abstract state
variable):

1. The Worker; this is essentially a std::future for the rendering task.
2. The RenderedImages; this is the result of rendering.

The system has two periodic update events, both at the same rate but with
different phase offsets:

1. The "capture" event updates the Worker state by launching a new render task.
2. The "output" event updates the RenderedImages state by waiting for the worker
   to finish and storing the resulting images.

The only real trick is how to sufficiently encapsulate a rendering task so that
it can run on a background thread. The Worker accomplishes that using a helper
class, the SnapshotSensor. The SnapshotSensor (itself a diagram) contains a
QueryObjectChef and RgbdSensor connected in series. The Worker allocates a
standalone SnapshotSensor Context and fixes the chef's input port(s) to be a
copy of the scene graph's FramePoseVector input port(s), and uses the RgbdSensor
to produce a rendered image on its output port. */

namespace drake {
namespace systems {
namespace sensors {

using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryVersion;
using geometry::QueryObject;
using geometry::Role;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using geometry::render::ClippingRange;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRange;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderCameraCore;
using internal::RgbdSensorAsyncParameters;
using math::RigidTransformd;

namespace {

/* A stateless system with the exact same input ports and output ports as the
given SceneGraph, but with a private scratch Context to allow fixed input ports
for the frame kinematics input. */
class QueryObjectChef final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QueryObjectChef);

  explicit QueryObjectChef(const SceneGraph<double>* scene_graph)
      : scene_graph_(scene_graph) {
    DRAKE_DEMAND(scene_graph != nullptr);
    // Our input ports are exactly the same as the underlying scene_graph.
    for (InputPortIndex i{0}; i < scene_graph->num_input_ports(); ++i) {
      const auto& port = scene_graph->get_input_port(i);
      DeclareAbstractInputPort(port.get_name(), *port.Allocate());
    }
    // We allocate a SceneGraph context into scratch storage in *our* context.
    Scratch scratch;
    scratch.scene_graph_context = scene_graph->CreateDefaultContext();
    scratch_index_ =
        this->DeclareCacheEntry(
                "scratch", ValueProducer(scratch, &ValueProducer::NoopCalc),
                {this->nothing_ticket()})
            .cache_index();
    // Our output is calculated by the SceneGraph. We use the lambda-based port
    // declaration syntax here (vs member function callback syntax) to avoid
    // extra copying.
    DeclareAbstractOutputPort(
        "query",
        []() {
          return Value<QueryObject<double>>().Clone();
        },
        [this](const Context<double>& context, AbstractValue* output) {
          this->CalcOutput(context, output);
        },
        {all_input_ports_ticket()});
  }

  void CalcOutput(const Context<double>& context, AbstractValue* output) const {
    Scratch& scratch = get_cache_entry(scratch_index_)
                           .get_mutable_cache_entry_value(context)
                           .GetMutableValueOrThrow<Scratch>();
    DRAKE_DEMAND(scratch.scene_graph_context != nullptr);
    Context<double>& scene_graph_context = *scratch.scene_graph_context;
    for (InputPortIndex i{0}; i < num_input_ports(); ++i) {
      const auto& port = get_input_port(i);
      if (port.HasValue(context)) {
        const auto& value = port.template Eval<AbstractValue>(context);
        scene_graph_context.FixInputPort(i, value);
      }
    }
    scene_graph_->get_query_output_port().Calc(scene_graph_context, output);
  }

 private:
  struct Scratch {
    copyable_unique_ptr<Context<double>> scene_graph_context;
  };

  const SceneGraph<double>* const scene_graph_;
  CacheIndex scratch_index_;
};

/* Combines QueryObjectChef + RgbdSensor in series, creating a sensor that
renders from a static FramePoseVector instead of a live QueryObject. */
class SnapshotSensor final : public Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SnapshotSensor);

  SnapshotSensor(const SceneGraph<double>* scene_graph, FrameId parent_id,
                 const RigidTransformd& X_PB, ColorRenderCamera color_camera,
                 DepthRenderCamera depth_camera) {
    DRAKE_DEMAND(scene_graph != nullptr);
    geometry_version_ = scene_graph->model_inspector().geometry_version();
    DiagramBuilder<double> builder;
    auto* chef = builder.AddNamedSystem<QueryObjectChef>("chef", scene_graph);
    auto* rgbd = builder.AddNamedSystem<RgbdSensor>("camera", parent_id, X_PB,
                                                    color_camera, depth_camera);
    builder.Connect(*chef, *rgbd);
    for (InputPortIndex i{0}; i < chef->num_input_ports(); ++i) {
      const auto& input_port = chef->get_input_port(i);
      builder.ExportInput(input_port, input_port.get_name());
    }
    for (OutputPortIndex i{0}; i < rgbd->num_output_ports(); ++i) {
      const auto& output_port = rgbd->get_output_port(i);
      builder.ExportOutput(output_port, output_port.get_name());
    }
    builder.BuildInto(this);
  }

  /* Returns the version as of when this sensor was created. */
  const GeometryVersion& geometry_version() const { return geometry_version_; }

 private:
  GeometryVersion geometry_version_;
};

/* The results of camera rendering. */
struct RenderedImages {
  RigidTransformd X_WB;
  double time{std::numeric_limits<double>::quiet_NaN()};
  std::shared_ptr<const ImageRgba8U> color;
  std::shared_ptr<const ImageDepth32F> depth;
  std::shared_ptr<const ImageLabel16I> label;
};

/* The worker is an object where Start() copies the pose input for camera
rendering and launches an async task, and Finish() blocks for the task to
complete. The expected workflow is to create a Worker and then repeatedly
Start and Finish in alternation (with exactly one Finish per Start). This
encapsulates the lifetime of the async task along with the objects it must
keep alive during rendering. */
class Worker {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Worker);

  Worker(std::shared_ptr<const SnapshotSensor> sensor, bool color, bool depth,
         bool label, uint64_t parameters_version)
      : sensor_{std::move(sensor)},
        color_{color},
        depth_{depth},
        label_{label},
        parameters_version_{parameters_version} {
    DRAKE_DEMAND(sensor_ != nullptr);
    sensor_context_ = sensor_->CreateDefaultContext();
  }

  uint64_t parameters_version() const { return parameters_version_; }

  /* Begins rendering the given geometry as an async task. */
  void Start(double context_time, const QueryObject<double>& query);

  /* Waits until image rendering for the most recent call to Start() is finished
  and then returns the result. When there is no async task (e.g., if Start has
  not been called since the most recent Finish), returns a default-constructed
  value (i.e., with nullptr for the images). */
  RenderedImages Finish();

 private:
  const std::shared_ptr<const SnapshotSensor> sensor_;
  const bool color_;
  const bool depth_;
  const bool label_;
  const uint64_t parameters_version_;
  std::unique_ptr<Context<double>> sensor_context_;
  std::future<RenderedImages> future_;
};

}  // namespace

/* The abstract state for an RgbdSensorAsync. The `output` is what appears on
RgbdSensorAsync output ports. The `worker` encapsulates the background task.

If the systems framework offered unrestricted update events that only updated
one specific AbstractStateIndex instead of the entire State, then it would make
sense to split this up into two separate abstract states. In the meantime, it's
simplest to keep all of our state in one place. */
struct RgbdSensorAsync::TickTockState {
  std::shared_ptr<Worker> worker;
  RenderedImages output;
};

RgbdSensorAsync::RgbdSensorAsync(const SceneGraph<double>* scene_graph,
                                 FrameId parent_id, const RigidTransformd& X_PB,
                                 double fps, double capture_offset,
                                 double output_delay,
                                 std::optional<ColorRenderCamera> color_camera,
                                 std::optional<DepthRenderCamera> depth_camera,
                                 bool render_label_image)
    : scene_graph_{scene_graph},
      fps_{fps},
      capture_offset_{capture_offset},
      output_delay_{output_delay},
      defaults_{.parent_frame_id = parent_id,
                .X_PB = X_PB,
                .color_camera = std::move(color_camera),
                .depth_camera = std::move(depth_camera)},
      render_label_image_{render_label_image} {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  DRAKE_THROW_UNLESS(std::isfinite(fps) && (fps > 0));
  DRAKE_THROW_UNLESS(std::isfinite(capture_offset) && (capture_offset >= 0));
  DRAKE_THROW_UNLESS(std::isfinite(output_delay) && (output_delay > 0));
  DRAKE_THROW_UNLESS(output_delay < (1 / fps));
  DRAKE_THROW_UNLESS(HasColorCamera() || HasDepthCamera());
  DRAKE_THROW_UNLESS(!render_label_image || HasColorCamera());
  // TODO(jwnimmer-tri) Check that the render engine named by either of the two
  // cameras is not the (known non-threadsafe) VTK engine.

  // Input.
  DeclareAbstractInputPort("geometry_query", Value<QueryObject<double>>{});

  // State.
  using Self = RgbdSensorAsync;
  auto state_index = DeclareAbstractState(Value<TickTockState>{});
  DeclareInitializationUnrestrictedUpdateEvent(&Self::Initialize);
  DeclarePeriodicUnrestrictedUpdateEvent(1 / fps, capture_offset,
                                         &Self::CalcTick);
  DeclarePeriodicUnrestrictedUpdateEvent(1 / fps, capture_offset + output_delay,
                                         &Self::CalcTock);

  // Output. These port names are intended to match RgbdSensor's port names.
  const std::set<DependencyTicket> state = {abstract_state_ticket(state_index)};
  if (HasColorCamera()) {
    DeclareAbstractOutputPort("color_image", &Self::CalcColor, state);
  }
  if (HasDepthCamera()) {
    DeclareAbstractOutputPort("depth_image_32f", &Self::CalcDepth32F, state);
    DeclareAbstractOutputPort("depth_image_16u", &Self::CalcDepth16U, state);
  }
  if (render_label_image) {
    DeclareAbstractOutputPort("label_image", &Self::CalcLabel, state);
  }
  DeclareAbstractOutputPort("body_pose_in_world", &Self::CalcX_WB, state);
  DeclareVectorOutputPort("image_time", 1, &Self::CalcImageTime, state);

  parameter_index_ = AbstractParameterIndex{this->DeclareAbstractParameter(
      Value<RgbdSensorAsyncParameters>(defaults_))};
}

geometry::FrameId RgbdSensorAsync::default_parent_frame_id() const {
  return defaults_.parent_frame_id;
}

void RgbdSensorAsync::set_default_parent_frame_id(geometry::FrameId id) {
  defaults_.parent_frame_id = id;
}

geometry::FrameId RgbdSensorAsync::GetParentFrameId(
    const Context<double>& context) const {
  this->ValidateContext(context);
  return GetParameters(context).parent_frame_id;
}

void RgbdSensorAsync::SetParentFrameId(Context<double>* context,
                                       geometry::FrameId id) const {
  this->ValidateContext(context);
  GetMutableParameters(context)->parent_frame_id = id;
}

const math::RigidTransformd& RgbdSensorAsync::default_X_PB() const {
  return defaults_.X_PB;
}

void RgbdSensorAsync::set_default_X_PB(
    const math::RigidTransformd& sensor_pose) {
  defaults_.X_PB = sensor_pose;
}

const math::RigidTransformd& RgbdSensorAsync::GetX_PB(
    const Context<double>& context) const {
  this->ValidateContext(context);
  return GetParameters(context).X_PB;
}

void RgbdSensorAsync::SetX_PB(Context<double>* context,
                              const math::RigidTransformd& sensor_pose) const {
  this->ValidateContext(context);
  GetMutableParameters(context)->X_PB = sensor_pose;
}

const std::optional<geometry::render::ColorRenderCamera>&
RgbdSensorAsync::default_color_render_camera() const {
  return defaults_.color_camera;
}

void RgbdSensorAsync::set_default_color_render_camera(
    const geometry::render::ColorRenderCamera& color_camera) {
  DRAKE_THROW_UNLESS(HasColorCamera());
  defaults_.color_camera = std::move(color_camera);
}

const std::optional<geometry::render::ColorRenderCamera>&
RgbdSensorAsync::GetColorRenderCamera(const Context<double>& context) const {
  this->ValidateContext(context);
  return GetParameters(context).color_camera;
}

void RgbdSensorAsync::SetColorRenderCamera(
    Context<double>* context,
    const geometry::render::ColorRenderCamera& color_camera) const {
  DRAKE_THROW_UNLESS(HasColorCamera());
  this->ValidateContext(context);
  GetMutableParameters(context)->color_camera = color_camera;
}

const std::optional<geometry::render::DepthRenderCamera>&
RgbdSensorAsync::default_depth_render_camera() const {
  return defaults_.depth_camera;
}

void RgbdSensorAsync::set_default_depth_render_camera(
    const geometry::render::DepthRenderCamera& depth_camera) {
  DRAKE_THROW_UNLESS(HasDepthCamera());
  defaults_.depth_camera = std::move(depth_camera);
}

const std::optional<geometry::render::DepthRenderCamera>&
RgbdSensorAsync::GetDepthRenderCamera(const Context<double>& context) const {
  this->ValidateContext(context);
  return GetParameters(context).depth_camera;
}

void RgbdSensorAsync::SetDepthRenderCamera(
    Context<double>* context,
    const geometry::render::DepthRenderCamera& depth_camera) const {
  DRAKE_THROW_UNLESS(HasDepthCamera());
  this->ValidateContext(context);
  GetMutableParameters(context)->depth_camera = depth_camera;
}

const OutputPort<double>& RgbdSensorAsync::color_image_output_port() const {
  constexpr char name[] = "color_image";
  return this->GetOutputPort(name);
}

const OutputPort<double>& RgbdSensorAsync::depth_image_32F_output_port() const {
  constexpr char name[] = "depth_image_32f";
  return this->GetOutputPort(name);
}

const OutputPort<double>& RgbdSensorAsync::depth_image_16U_output_port() const {
  constexpr char name[] = "depth_image_16u";
  return this->GetOutputPort(name);
}

const OutputPort<double>& RgbdSensorAsync::label_image_output_port() const {
  constexpr char name[] = "label_image";
  return this->GetOutputPort(name);
}

const OutputPort<double>& RgbdSensorAsync::body_pose_in_world_output_port()
    const {
  constexpr char name[] = "body_pose_in_world";
  return this->GetOutputPort(name);
}

const OutputPort<double>& RgbdSensorAsync::image_time_output_port() const {
  constexpr char name[] = "image_time";
  return this->GetOutputPort(name);
}

const RgbdSensorAsync::TickTockState& RgbdSensorAsync::get_state(
    const Context<double>& context) const {
  return context.template get_abstract_state<TickTockState>(0);
}

RgbdSensorAsync::TickTockState& RgbdSensorAsync::get_mutable_state(
    State<double>* state) const {
  DRAKE_DEMAND(state != nullptr);
  return state->template get_mutable_abstract_state<TickTockState>(0);
}

bool RgbdSensorAsync::HasColorCamera() const {
  return defaults_.color_camera.has_value();
}

bool RgbdSensorAsync::HasDepthCamera() const {
  return defaults_.depth_camera.has_value();
}

EventStatus RgbdSensorAsync::Initialize(const Context<double>& context,
                                        State<double>* state) const {
  // Grab the downcast reference from our argument.
  TickTockState& next_state = get_mutable_state(state);
  const RgbdSensorAsyncParameters& params = GetParameters(context);

  // If we are only going to render one of color or depth, invent dummy
  // properties for the other one to simplify the SnapshotSensor code.
  std::optional<ColorRenderCamera> color = params.color_camera;
  std::optional<DepthRenderCamera> depth = params.depth_camera;
  if (!HasColorCamera()) {
    DRAKE_DEMAND(HasDepthCamera());
    const RenderCameraCore& core = depth->core();
    color.emplace(core);
  }
  if (!HasDepthCamera()) {
    DRAKE_DEMAND(HasColorCamera());
    const RenderCameraCore& core = color->core();
    const ClippingRange& clip = core.clipping();
    // N.B. Avoid using clip.far() here; it can trip the "16 bit mm depth"
    // logger spam from RgbdSensor.
    depth.emplace(core, DepthRange(clip.near(), clip.near() * 1.001));
  }

  // The `sensor` is a separate, nested system that actually renders images.
  // The outer system (`this`) is just the event shims that will tick it. Our
  // job during initialization is to reset the nested system and any prior
  // output.
  auto sensor = std::make_shared<const SnapshotSensor>(
      scene_graph_, params.parent_frame_id, params.X_PB, std::move(*color),
      std::move(*depth));
  next_state.worker = std::make_shared<Worker>(
      std::move(sensor), HasColorCamera(), HasDepthCamera(),
      render_label_image_, params.version);
  next_state.output = {};
  return EventStatus::Succeeded();
}

void RgbdSensorAsync::CalcTick(const Context<double>& context,
                               State<double>* state) const {
  // Get the geometry pose updates.
  const auto& query = get_input_port().Eval<QueryObject<double>>(context);

  // Grab the downcast references from our arguments.
  const TickTockState& prior_state = get_state(context);
  TickTockState& next_state = get_mutable_state(state);

  // Preserve the current output from the prior state. We're only going to
  // launch a worker task, without any changes to our output.
  next_state.output = prior_state.output;

  // Latch-initialize a worker if we didn't have one yet, or if the parameters
  // are out of date.
  if (prior_state.worker == nullptr ||
      (prior_state.worker->parameters_version() !=
       GetParameters(context).version)) {
    Initialize(context, state);
  } else {
    next_state.worker = prior_state.worker;
  }

  // Start the worker on its next task.
  next_state.worker->Start(context.get_time(), query);
}

void RgbdSensorAsync::CalcTock(const Context<double>& context,
                               State<double>* state) const {
  // Grab the downcast references from our arguments.
  const TickTockState& prior_state = get_state(context);
  TickTockState& next_state = get_mutable_state(state);

  // If the user manually changes the State outside of a Simulator, we might hit
  // a tock without having been initialized. Guard that here to avoid crashing.
  if (prior_state.worker == nullptr) {
    next_state = {};
    return;
  }

  // TODO(jwnimmer-tri) Consider adding a guard here so that out-of-sequence
  // tocks due to the user manually screwing with the State result in empty
  // images instead of stale images.

  // Finish the worker task, and copy it to the output ports.
  next_state.worker = prior_state.worker;
  next_state.output = next_state.worker->Finish();
}

namespace {
/* If the size of `image` already matches `camera`, then does nothing.
Otherwise, resizes the `image` to match `camera`. */
template <typename SomeImage>
void Resize(const RenderCameraCore& camera, SomeImage* image) {
  const int width = camera.intrinsics().width();
  const int height = camera.intrinsics().height();
  if (image->width() == width && image->height() == height) {
    return;
  }
  image->resize(width, height);
}

/* If the rendered image is non-null, copy it to output. Otherwise, set the
the output to be empty (i.e., zero-sized width and height). */
template <typename ImageIn, typename ImageOut>
void CopyImage(const ImageIn* rendered, ImageOut* output) {
  DRAKE_DEMAND(output != nullptr);
  if (rendered == nullptr) {
    output->resize(0, 0);
    return;
  }
  if constexpr (std::is_same_v<ImageIn, ImageOut>) {
    *output = *rendered;
  } else {
    ConvertDepth32FTo16U(*rendered, output);
  }
}
}  // namespace

void RgbdSensorAsync::CalcColor(const Context<double>& context,
                                ImageRgba8U* output) const {
  const std::optional<ColorRenderCamera>& camera =
      GetColorRenderCamera(context);
  Resize(camera->core(), output);
  CopyImage(get_state(context).output.color.get(), output);
}

void RgbdSensorAsync::CalcLabel(const Context<double>& context,
                                ImageLabel16I* output) const {
  const std::optional<ColorRenderCamera>& camera =
      GetColorRenderCamera(context);
  Resize(camera->core(), output);
  CopyImage(get_state(context).output.label.get(), output);
}

void RgbdSensorAsync::CalcDepth32F(const Context<double>& context,
                                   ImageDepth32F* output) const {
  const std::optional<DepthRenderCamera>& camera =
      GetDepthRenderCamera(context);
  Resize(camera->core(), output);
  CopyImage(get_state(context).output.depth.get(), output);
}

void RgbdSensorAsync::CalcDepth16U(const Context<double>& context,
                                   ImageDepth16U* output) const {
  const std::optional<DepthRenderCamera>& camera =
      GetDepthRenderCamera(context);
  Resize(camera->core(), output);
  CopyImage(get_state(context).output.depth.get(), output);
}

void RgbdSensorAsync::CalcX_WB(const Context<double>& context,
                               RigidTransformd* output) const {
  *output = get_state(context).output.X_WB;
}

void RgbdSensorAsync::CalcImageTime(const Context<double>& context,
                                    BasicVector<double>* output) const {
  output->SetFromVector(Vector1d{get_state(context).output.time});
}

void RgbdSensorAsync::SetDefaultParameters(
    const Context<double>& context, Parameters<double>* parameters) const {
  LeafSystem<double>::SetDefaultParameters(context, parameters);
  parameters->get_mutable_abstract_parameter(parameter_index_)
      .set_value(defaults_);
}

const RgbdSensorAsyncParameters& RgbdSensorAsync::GetParameters(
    const Context<double>& context) const {
  return context.get_abstract_parameter(parameter_index_)
      .get_value<RgbdSensorAsyncParameters>();
}

RgbdSensorAsyncParameters* RgbdSensorAsync::GetMutableParameters(
    Context<double>* context) const {
  auto* params =
      &(context->get_mutable_abstract_parameter(parameter_index_)
            .template get_mutable_value<RgbdSensorAsyncParameters>());
  ++params->version;
  return params;
}

namespace {

void Worker::Start(double context_time, const QueryObject<double>& query) {
  // Confirm that the geometry version number has not changed since Initialize.
  const GeometryVersion& initialize_version = sensor_->geometry_version();
  const GeometryVersion& current_version = query.inspector().geometry_version();
  if (!current_version.IsSameAs(initialize_version, Role::kPerception)) {
    // TODO(jwnimmer-tri) Ideally, here we would update our copy of the geometry
    // to match `query` instead of throwing. That's somewhat complicated to
    // implement and test at the moment, so for now instead we have a warning in
    // our class overview docs cautioning about this.
    throw std::logic_error(
        "As the moment, RgbdSensorAsync cannot respond to changes in geometry "
        "shapes, textures, etc. after a simulation has started. If you change "
        "anything beyond poses, you must manually call Simulator::Initialize() "
        "to reset things before resuming the simulation.");
  }

  // Read the frame poses from the QueryObject. Note that deformable geometry is
  // not supported because this only propagates poses, not configurations.
  // TODO(jwnimmer-tri) After the render engine API adds support for deformable
  // geometry, we should upgrade this sensor to support deformables as well.
  std::map<std::string, FramePoseVector<double>> poses;
  const SceneGraphInspector<double>& inspector = query.inspector();
  for (bool first = true; const auto& source_id : inspector.GetAllSourceIds()) {
    if (first) {
      first = false;
      // Skip the SceneGraph source that holds the "world" frame.
      DRAKE_DEMAND(
          inspector.BelongsToSource(inspector.world_frame_id(), source_id));
      continue;
    }
    const std::string& source_name = inspector.GetName(source_id);
    auto& source_poses = poses[source_name + "_pose"];
    for (const auto& frame_id : inspector.FramesForSource(source_id)) {
      source_poses.set_value(frame_id, query.GetPoseInParent(frame_id));
    }
  }

  // Abandon our prior task (typically not necessary; valid() is usually false).
  if (future_.valid()) {
    future_.wait();
    future_ = {};
  }

  // Launch the rendering task.
  auto task = [this, context_time,
               poses = std::move(poses)]() -> RenderedImages {
    for (const auto& [port_name, pose_vector] : poses) {
      const auto& input_port = sensor_->GetInputPort(port_name);
      input_port.FixValue(sensor_context_.get(), pose_vector);
    }
    RenderedImages result;
    // TODO(jwnimmer-tri) Is there any way we can steal (move) the images from
    // the output ports, to avoid extra copying? I suppose we could call the
    // QueryObject directly instead of the RgbdSensor, but that would involve
    // duplicating (copying) some of its functionality into this class.
    if (color_) {
      result.color = std::make_shared<const ImageRgba8U>(
          sensor_->GetOutputPort("color_image")
              .template Eval<ImageRgba8U>(*sensor_context_));
    }
    if (depth_) {
      result.depth = std::make_shared<const ImageDepth32F>(
          sensor_->GetOutputPort("depth_image_32f")
              .template Eval<ImageDepth32F>(*sensor_context_));
    }
    if (label_) {
      result.label = std::make_shared<const ImageLabel16I>(
          sensor_->GetOutputPort("label_image")
              .template Eval<ImageLabel16I>(*sensor_context_));
    }
    result.X_WB = sensor_->GetOutputPort("body_pose_in_world")
                      .template Eval<RigidTransformd>(*sensor_context_);
    result.time = context_time;
    return result;
  };
  future_ = std::async(std::launch::async, std::move(task));
}

RenderedImages Worker::Finish() {
  if (!future_.valid()) {
    return {};
  }
  future_.wait();
  return future_.get();
}

}  // namespace

}  // namespace sensors
}  // namespace systems
}  // namespace drake
