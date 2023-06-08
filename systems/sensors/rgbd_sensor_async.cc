#include "drake/systems/sensors/rgbd_sensor_async.h"

#include <future>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/rgbd_sensor.h"

namespace drake {
namespace systems {
namespace sensors {

using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::QueryObject;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using geometry::SourceId;
using geometry::render::ClippingRange;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRange;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderCameraCore;
using math::RigidTransformd;

namespace {

/* A stateless system with the exact same input ports and output ports as the
given SceneGraph, but with a private scratch Context to allow fixed input ports
for the frame kinematics input. */
class QueryObjectChef final : public LeafSystem<double> {
 public:
  explicit QueryObjectChef(const SceneGraph<double>* scene_graph)
      : scene_graph_(scene_graph) {
    DRAKE_DEMAND(scene_graph != nullptr);
    // Our input ports are exactly the same as the underlying scene_graph.
    for (InputPortIndex i{0}; i < scene_graph->num_input_ports(); ++i) {
      const auto& port = scene_graph->get_input_port(i);
      DeclareAbstractInputPort(port.get_name(), *port.Allocate());
    }
    // We allocate a SceneGraph context into scratch storage in *our* context.
    // TODO(jwnimmer-tri) This is currently the *only* use of scene_graph. If
    // that remains true after all async features are finished, perhaps we could
    // store only a scene graph Context instead of the whole SceneGraph, which
    // might be less burdensome on the user in terms of lifetime preconditions.
    Scratch scratch;
    scratch.scene_graph_context = scene_graph->CreateDefaultContext();
    scratch_index_ =
        this->DeclareCacheEntry(
                "scratch", ValueProducer(scratch, &ValueProducer::NoopCalc),
                {this->nothing_ticket()})
            .cache_index();
    // Our output is calculated by the SceneGraph. We use the lambda-based port
    // declaration syntax here (vs member function callback syntax) to avoid
    // extry copying.
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
  SnapshotSensor(const SceneGraph<double>* scene_graph, FrameId parent_id,
                 const RigidTransformd& X_PB, ColorRenderCamera color_camera,
                 DepthRenderCamera depth_camera) {
    DRAKE_DEMAND(scene_graph != nullptr);
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
};

/* The results of camera rendering. */
struct RenderedImages {
  RigidTransformd X_WB;
  std::shared_ptr<const ImageRgba8U> color;
  std::shared_ptr<const ImageDepth32F> depth;
  std::shared_ptr<const ImageLabel16I> label;
};

/* The worker object where Start() copies the pose input for camera rendering
and launches an async task, and Finish() blocks for the task to complete. */
class Worker {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Worker)

  Worker(std::shared_ptr<const SnapshotSensor> sensor, bool color, bool depth,
         bool label)
      : sensor_{std::move(sensor)},
        color_{color},
        depth_{depth},
        label_{label} {
    DRAKE_DEMAND(sensor_ != nullptr);
    sensor_context_ = sensor_->CreateDefaultContext();
  }

  /* Begins rendering the given geometry as an async task. */
  void Start(const QueryObject<double>& query);

  /* Waits until image rendering for the most recent call to Start() is finished
  and then returns the result. The returned reference is valid up until the next
  call to Start(). */
  const RenderedImages& Finish();

 private:
  const std::shared_ptr<const SnapshotSensor> sensor_;
  const bool color_;
  const bool depth_;
  const bool label_;
  std::unique_ptr<Context<double>> sensor_context_;
  std::future<RenderedImages> future_;
  RenderedImages result_;
};

}  // namespace

/* The abstract state for an RgbdSensorAsync. The `output` is what appears on
RgbdSensorAsync output ports. The `worker` encapsultate the background task. */
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
      parent_id_{parent_id},
      X_PB_{X_PB},
      fps_{fps},
      capture_offset_{capture_offset},
      output_delay_{output_delay},
      color_camera_{std::move(color_camera)},
      depth_camera_{std::move(depth_camera)},
      render_label_image_{render_label_image} {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  DRAKE_THROW_UNLESS(std::isfinite(fps) && (fps > 0));
  DRAKE_THROW_UNLESS(std::isfinite(capture_offset));
  DRAKE_THROW_UNLESS(std::isfinite(output_delay) && (output_delay > 0));
  DRAKE_THROW_UNLESS(output_delay < (1 / fps));
  const double output_offset = capture_offset + output_delay;
  DRAKE_THROW_UNLESS(color_camera_.has_value() || depth_camera_.has_value());
  DRAKE_THROW_UNLESS(!render_label_image || color_camera_.has_value());

  // Input.
  DeclareAbstractInputPort("geometry_query", Value<QueryObject<double>>{});

  // State.
  using Self = RgbdSensorAsync;
  auto state_index = DeclareAbstractState(Value<TickTockState>{});
  DeclareInitializationUnrestrictedUpdateEvent(&Self::Initialize);
  DeclarePeriodicUnrestrictedUpdateEvent(1 / fps, capture_offset,
                                         &Self::CalcTick);
  DeclarePeriodicUnrestrictedUpdateEvent(1 / fps, output_offset,
                                         &Self::CalcTock);

  // Output.
  const std::set<DependencyTicket> state = {abstract_state_ticket(state_index)};
  if (color_camera_.has_value()) {
    DeclareAbstractOutputPort("color_image", &Self::CalcColor, state);
  }
  if (depth_camera_.has_value()) {
    DeclareAbstractOutputPort("depth_image_32f", &Self::CalcDepth32F, state);
    DeclareAbstractOutputPort("depth_image_16u", &Self::CalcDepth16U, state);
  }
  if (render_label_image) {
    DeclareAbstractOutputPort("label_image", &Self::CalcLabel, state);
  }
  DeclareAbstractOutputPort("body_pose_in_world", &Self::CalcX_WB, state);
}

const OutputPort<double>* RgbdSensorAsync::color_image_output_port() const {
  constexpr char name[] = "color_image";
  return this->HasOutputPort(name) ? &(this->GetOutputPort(name)) : nullptr;
}

const OutputPort<double>* RgbdSensorAsync::depth_image_32F_output_port() const {
  constexpr char name[] = "depth_image_32f";
  return this->HasOutputPort(name) ? &(this->GetOutputPort(name)) : nullptr;
}

const OutputPort<double>* RgbdSensorAsync::depth_image_16U_output_port() const {
  constexpr char name[] = "depth_image_16u";
  return this->HasOutputPort(name) ? &(this->GetOutputPort(name)) : nullptr;
}

const OutputPort<double>* RgbdSensorAsync::label_image_output_port() const {
  constexpr char name[] = "label_image";
  return this->HasOutputPort(name) ? &(this->GetOutputPort(name)) : nullptr;
}

const OutputPort<double>& RgbdSensorAsync::body_pose_in_world_output_port()
    const {
  constexpr char name[] = "body_pose_in_world";
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

EventStatus RgbdSensorAsync::Initialize(const Context<double>& context,
                                        State<double>* state) const {
  // Grab the downcast reference from our argument.
  unused(context);
  TickTockState& next_state = get_mutable_state(state);

  // If we are only going to render one of color or depth, invent dummy
  // properties for the other one to simplify the SnapshotSensor code.
  std::optional<ColorRenderCamera> color = color_camera_;
  std::optional<DepthRenderCamera> depth = depth_camera_;
  if (!color.has_value()) {
    DRAKE_DEMAND(depth.has_value());
    const RenderCameraCore& core = depth->core();
    color.emplace(core);
  }
  if (!depth.has_value()) {
    DRAKE_DEMAND(color.has_value());
    const RenderCameraCore& core = color->core();
    const ClippingRange& clip = core.clipping();
    depth.emplace(core, DepthRange(clip.near(), clip.far()));
  }

  // The `sensor` is a separate, nested system that actually renders images.
  // The outer system (`this`) is just the event shims that will tick it. Our
  // job during initialization is to reset the nested system and any prior
  // output.
  auto sensor = std::make_shared<const SnapshotSensor>(
      scene_graph_, parent_id_, X_PB_, std::move(*color), std::move(*depth));
  next_state.worker =
      std::make_shared<Worker>(std::move(sensor), color_camera_.has_value(),
                               depth_camera_.has_value(), render_label_image_);
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

  // Latch-initialize a worker we didn't have one yet.

  // TODO(jwnimmer-tri) Here we could check the perception geometry's serial
  // number and upon any change update our copy of the geometry instances.
  // That's somewhat complicated to implement at the moment, so instead for now
  // we have a warning in our class overview docs cautioning about this.
  if (prior_state.worker == nullptr) {
    Initialize(context, state);
  } else {
    next_state.worker = prior_state.worker;
  }

  // Start the worker on its next task.
  next_state.worker->Start(query);
}

void RgbdSensorAsync::CalcTock(const Context<double>& context,
                               State<double>* state) const {
  // Grab the downcast references from our arguments.
  const TickTockState& prior_state = get_state(context);
  TickTockState& next_state = get_mutable_state(state);

  // If the user manually changed the context time outside of the simulator,
  // we might hit a tock without having launched a tick.
  if (prior_state.worker == nullptr) {
    next_state = {};
    return;
  }

  // Finish the worker task, and copy it to the output ports.
  next_state.worker = prior_state.worker;
  next_state.output = next_state.worker->Finish();
}

namespace {
/* If the rendered image is non-null, copy it to output. Otherwise, zero the
output using the given camera's width and height. */
template <typename ImageIn, typename ImageOut>
void CopyImage(const RenderCameraCore& camera, const ImageIn* rendered,
               ImageOut* output) {
  DRAKE_DEMAND(output != nullptr);
  if (rendered == nullptr) {
    output->resize(camera.intrinsics().width(), camera.intrinsics().height());
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
  DRAKE_DEMAND(color_camera_.has_value());
  CopyImage(color_camera_->core(), get_state(context).output.color.get(),
            output);
}

void RgbdSensorAsync::CalcLabel(const Context<double>& context,
                                ImageLabel16I* output) const {
  DRAKE_DEMAND(color_camera_.has_value());
  CopyImage(color_camera_->core(), get_state(context).output.label.get(),
            output);
}

void RgbdSensorAsync::CalcDepth32F(const Context<double>& context,
                                   ImageDepth32F* output) const {
  DRAKE_DEMAND(depth_camera_.has_value());
  CopyImage(depth_camera_->core(), get_state(context).output.depth.get(),
            output);
}

void RgbdSensorAsync::CalcDepth16U(const Context<double>& context,
                                   ImageDepth16U* output) const {
  DRAKE_DEMAND(depth_camera_.has_value());
  CopyImage(depth_camera_->core(), get_state(context).output.depth.get(),
            output);
}

void RgbdSensorAsync::CalcX_WB(const Context<double>& context,
                               RigidTransformd* output) const {
  *output = get_state(context).output.X_WB;
}

namespace {

void Worker::Start(const QueryObject<double>& query) {
  // Abandon our prior task.
  future_ = {};
  result_ = {};

  // Read the frame poses from the QueryObject. Note that deformable geometry is
  // not supported because this only propagates poses, not configurations.
  std::map<std::string, FramePoseVector<double>> poses;
  const SceneGraphInspector<double>& inspector = query.inspector();
  for (bool first = true; const auto& source_id : inspector.GetAllSourceIds()) {
    if (first) {
      first = false;
      // Skip the SceneGraph source that holds the "world" frame.
      DRAKE_DEMAND(inspector.FramesForSource(source_id).count(
                       inspector.world_frame_id()) > 0);
      continue;
    }
    const std::string& source_name = inspector.GetName(source_id);
    auto& source_poses = poses[source_name + "_pose"];
    for (const auto& frame_id : inspector.FramesForSource(source_id)) {
      // TODO(jwnimmer-tri) This is not reached by unit tests.
      // Once we have an e2e acceptance test, it will be.
      DRAKE_DEMAND(false);
      source_poses.set_value(frame_id, query.GetPoseInParent(frame_id));
    }
  }

  // Launch the rendering task.
  auto task = [this, poses = std::move(poses)]() -> RenderedImages {
    for (const auto& [port_name, pose_vector] : poses) {
      const auto& input_port = sensor_->GetInputPort(port_name);
      input_port.FixValue(sensor_context_.get(), pose_vector);
    }
    RenderedImages result;
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
    return result;
  };
  future_ = std::async(std::launch::async, std::move(task));
}

const RenderedImages& Worker::Finish() {
  if (future_.valid()) {
    future_.wait();
    result_ = future_.get();
  } else {
    result_ = {};
  }
  return result_;
}

}  // namespace

}  // namespace sensors
}  // namespace systems
}  // namespace drake
