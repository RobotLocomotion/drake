"""
Play back a demonstration using the Convex Integrator.

Track joint targets with a stiff PD controller so we can avoid going through the
(discrete-time) DiffIK.

Usage:
   bazel run //examples/hero_demo:convex_integrator_playback
"""

import argparse
import time

import numpy as np

from pydrake.common import FindResourceOrThrow, Parallelism
from pydrake.common.yaml import yaml_dump, yaml_load_file
from pydrake.geometry import StartMeshcat
from pydrake.multibody import cenic  # noqa
from pydrake.multibody.parsing import PackageMap, Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import PdControllerGains
from pydrake.systems.analysis import (
    ApplySimulatorConfig,
    PrintSimulatorStatistics,
    Simulator,
    SimulatorConfig,
)
from pydrake.systems.controllers import PidController
from pydrake.systems.framework import (
    DiagramBuilder,
    EventStatus,
    LeafSystem,
)
from pydrake.trajectories import PiecewisePolynomial
from pydrake.visualization import (
    ApplyVisualizationConfig,
    VisualizationConfig,
)

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--visualize", type=int, default=1)
arg_parser.add_argument("--log_times", type=int, default=0)
arg_parser.add_argument(
    "--num_threads",
    type=int,
    default=1,
    help="Number of threads used to solve constraint islands in parallel. "
    "1 (the default) is serial; 0 uses the maximum available. Only has an "
    "effect with --use_islands enabled.",
)
arg_parser.add_argument("--use_islands", type=int, default=1)
args = arg_parser.parse_args()


class JointTargetSource(LeafSystem):
    """
    This simple leaf system sends out joint targets to be tracked by a PID
    controller, as recorded in a keyframes.txt file.
    """

    def __init__(self):
        super().__init__()

        # Parse joint targets from the keyframes file
        keyframes_file = FindResourceOrThrow(
            "drake/examples/hero_demo/keyframes.txt"
        )
        with open(keyframes_file, "r") as f:
            lines = f.readlines()

        times = []
        left_arm_targets = []
        right_arm_targets = []
        left_gripper_targets = []
        right_gripper_targets = []
        for line in lines:
            if line.startswith("time: "):
                time_str = line.split(":")[1].strip()
                times.append(float(time_str))
            elif line.startswith("left::panda: "):
                target_str = line.split(":")[-1].strip()
                target = np.array([float(x) for x in target_str.split()])
                assert len(target) == 7
                left_arm_targets.append(target)
            elif line.startswith("right::panda: "):
                target_str = line.split(":")[-1].strip()
                target = np.array([float(x) for x in target_str.split()])
                assert len(target) == 7
                right_arm_targets.append(target)
            elif line.startswith("left::panda_hand: "):
                target_str = line.split(":")[-1].strip()
                target = np.array([float(x) for x in target_str.split()])
                assert len(target) == 2
                left_gripper_targets.append(target)
            elif line.startswith("right::panda_hand: "):
                target_str = line.split(":")[-1].strip()
                target = np.array([float(x) for x in target_str.split()])
                assert len(target) == 2
                right_gripper_targets.append(target)

        self.left_arm_spline = (
            PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
                times, np.array(left_arm_targets).T
            )
        )
        self.right_arm_spline = (
            PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
                times, np.array(right_arm_targets).T
            )
        )
        self.left_gripper_spline = (
            PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
                times, np.array(left_gripper_targets).T
            )
        )
        self.right_gripper_spline = (
            PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
                times, np.array(right_gripper_targets).T
            )
        )

        self.DeclareVectorOutputPort("left_arm", 14, self.CalcLeftArmTarget)
        self.DeclareVectorOutputPort("right_arm", 14, self.CalcRightArmTarget)
        self.DeclareVectorOutputPort(
            "left_gripper", 4, self.CalcLeftGripperTarget
        )
        self.DeclareVectorOutputPort(
            "right_gripper", 4, self.CalcRightGripperTarget
        )

    def CalcLeftArmTarget(self, context, output):
        q_nom = self.left_arm_spline.value(context.get_time()).flatten()
        v_nom = np.zeros(7)
        x_nom = np.hstack((q_nom, v_nom))
        output.SetFromVector(x_nom)

    def CalcRightArmTarget(self, context, output):
        q_nom = self.right_arm_spline.value(context.get_time()).flatten()
        v_nom = np.zeros(7)
        x_nom = np.hstack((q_nom, v_nom))
        output.SetFromVector(x_nom)

    def CalcLeftGripperTarget(self, context, output):
        q_nom = self.left_gripper_spline.value(context.get_time()).flatten()
        if q_nom[1] < 0.03:
            # When the gripper is closed, squeeze it closed tightly
            q_nom = np.array([0.01, -0.01])
        v_nom = np.zeros(2)
        x_nom = np.hstack((q_nom, v_nom))
        output.SetFromVector(x_nom)

    def CalcRightGripperTarget(self, context, output):
        q_nom = self.right_gripper_spline.value(context.get_time()).flatten()
        if q_nom[1] < 0.03:
            # When the gripper is closed, squeeze it closed tightly
            q_nom = np.array([0.01, -0.01])
        v_nom = np.zeros(2)
        x_nom = np.hstack((q_nom, v_nom))
        output.SetFromVector(x_nom)


class TimeLogger(LeafSystem):
    """
    A simple logger to record the simulation time.
    """

    def __init__(self):
        super().__init__()
        self.DeclareForcedPublishEvent(self.RecordTimes)
        self.sim_times = []
        self.wall_times = []

    def SaveRecordedTimes(self):
        save_path = "/tmp/dishrack_times.csv"
        with open(save_path, "w") as f:
            f.write("sim_time,wall_time\n")
            for sim_time, wall_time in zip(self.sim_times, self.wall_times):
                f.write(f"{sim_time},{wall_time}\n")
        print(f"Timing logs written to {save_path}")

    def RecordTimes(self, context):
        self.sim_times.append(context.get_time())
        self.wall_times.append(time.monotonic())


# Load model directives from the scenario file saved with the recording
model_directives_file = FindResourceOrThrow(
    "drake/examples/hero_demo/resolved_scenario.yaml"
)
data = yaml_load_file(model_directives_file)
directives_data = data["directives"]
directives_string = yaml_dump({"directives": directives_data})

# Set up the system diagram
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
parser = Parser(builder)

remote_params = PackageMap.RemoteParams(
    urls=[
        "https://github.com/ToyotaResearchInstitute/lbm_eval"
        "/releases/download/1.1.0/lbm_eval_models-1.1.0-py3-none-any.whl",
    ],
    sha256="97d61eb617d2d409d7c5873824ff79d26f6dd1a5532e428d4d320fac15c2957d",
    archive_type="zip",
    strip_prefix="lbm_eval_models",
)
parser.package_map().AddRemote(
    package_name="lbm_eval_models", params=remote_params
)
parser.AddModelsFromString(
    file_contents=directives_string, file_type="dmd.yaml"
)

# Remove implicit PD actuation
for idx in plant.GetJointActuatorIndices():
    actuator = plant.get_joint_actuator(idx)
    if actuator.has_controller():
        actuator.set_controller_gains(PdControllerGains(p=0.0, d=0.0))

plant.Finalize()

# Connect to meshcat for visualization
if args.visualize:
    meshcat = StartMeshcat()
    vis_config = VisualizationConfig()
    vis_config.publish_period = 1e30
    vis_config.publish_contacts = False
    vis_config.publish_inertia = False
    vis_config.publish_proximity = False
    ApplyVisualizationConfig(vis_config, builder=builder, meshcat=meshcat)

    # Configure meshcat parameters for nicer visualization
    meshcat_params_file = FindResourceOrThrow(
        "drake/examples/hero_demo/meshcat_params.yaml"
    )
    meshcat_params = yaml_load_file(meshcat_params_file)
    for p in meshcat_params["initial_properties"]:
        meshcat.SetProperty(p["path"], p["property"], p["value"])
    meshcat.SetProperty("/Axes", "visible", False)
    meshcat.SetCameraPose([0.7, -0.3, 0.7], [0.0, 0.0, 0.2])

# Connect stiff joint-level PID controllers to the robot
Kp_arm = 5e3 * np.ones(7)
Ki_arm = 0.0 * np.ones(7)
Kd_arm = 1e3 * np.ones(7)

Px_gripper = np.eye(4)
Py_gripper = np.array([[0.5, -0.5]])
Kp_gripper = 5e3 * np.ones(2)
Ki_gripper = 0.0 * np.ones(2)
Kd_gripper = 1e3 * np.ones(2)

joint_target_source = builder.AddSystem(JointTargetSource())

left_arm_ctrl = builder.AddSystem(PidController(Kp_arm, Ki_arm, Kd_arm))
right_arm_ctrl = builder.AddSystem(PidController(Kp_arm, Ki_arm, Kd_arm))
left_gripper_ctrl = builder.AddSystem(
    PidController(Px_gripper, Py_gripper, Kp_gripper, Ki_gripper, Kd_gripper)
)
right_gripper_ctrl = builder.AddSystem(
    PidController(Px_gripper, Py_gripper, Kp_gripper, Ki_gripper, Kd_gripper)
)

left_arm = plant.GetModelInstanceByName("left::panda")
right_arm = plant.GetModelInstanceByName("right::panda")
left_gripper = plant.GetModelInstanceByName("left::panda_hand")
right_gripper = plant.GetModelInstanceByName("right::panda_hand")

builder.Connect(
    joint_target_source.GetOutputPort("left_arm"),
    left_arm_ctrl.get_input_port_desired_state(),
)
builder.Connect(
    plant.get_state_output_port(left_arm),
    left_arm_ctrl.get_input_port_estimated_state(),
)
builder.Connect(
    left_arm_ctrl.get_output_port_control(),
    plant.get_actuation_input_port(left_arm),
)

builder.Connect(
    joint_target_source.GetOutputPort("right_arm"),
    right_arm_ctrl.get_input_port_desired_state(),
)
builder.Connect(
    plant.get_state_output_port(right_arm),
    right_arm_ctrl.get_input_port_estimated_state(),
)
builder.Connect(
    right_arm_ctrl.get_output_port_control(),
    plant.get_actuation_input_port(right_arm),
)

builder.Connect(
    joint_target_source.GetOutputPort("left_gripper"),
    left_gripper_ctrl.get_input_port_desired_state(),
)
builder.Connect(
    plant.get_state_output_port(left_gripper),
    left_gripper_ctrl.get_input_port_estimated_state(),
)
builder.Connect(
    left_gripper_ctrl.get_output_port_control(),
    plant.get_actuation_input_port(left_gripper),
)

builder.Connect(
    joint_target_source.GetOutputPort("right_gripper"),
    right_gripper_ctrl.get_input_port_desired_state(),
)
builder.Connect(
    plant.get_state_output_port(right_gripper),
    right_gripper_ctrl.get_input_port_estimated_state(),
)
builder.Connect(
    right_gripper_ctrl.get_output_port_control(),
    plant.get_actuation_input_port(right_gripper),
)

# Set initial conditions from the recording
initial_positions = data["initial_position"]
for model in initial_positions.keys():
    model_instance = plant.GetModelInstanceByName(model)
    for joint_name in initial_positions[model].keys():
        joint = plant.GetJointByName(joint_name, model_instance)
        joint.set_default_positions(initial_positions[model][joint_name])

# Add a time logger to record simulation time
if args.log_times:
    time_logger = builder.AddSystem(TimeLogger())

# Compile the system diagram
diagram = builder.Build()
context = diagram.CreateDefaultContext()

# Set up the simulator
config = SimulatorConfig()
if plant.time_step() == 0.0:
    config.integration_scheme = "cenic"
config.accuracy = 1e-3
config.max_step_size = 0.1
config.target_realtime_rate = 0.0
config.use_error_control = True

# Importing cenic above registers CenicIntegrator so get_mutable_integrator()
# can downcast to it (giving access to {get,Set}SolverParameters).
simulator = Simulator(diagram, context)
ApplySimulatorConfig(config, simulator)
integrator = simulator.get_mutable_integrator()
params = integrator.get_solver_parameters()
params.print_solver_stats = False
params.use_islands = args.use_islands
integrator.SetSolverParameters(params)

# Optionally solve independent constraint islands in parallel. 0 requests the
# maximum available; any other value sets an explicit thread count. Results are
# independent of the number of threads.
parallelism = (
    Parallelism.Max()
    if args.num_threads == 0
    else Parallelism(args.num_threads)
)
integrator.set_parallelism(parallelism)

if args.visualize:
    # Force a publish (and thus a recording frame) once per integration step,
    # independent of the step size, by forcing a publish from a monitor. The
    # visualizer's periodic publish is effectively disabled (publish_period set
    # to 1e30 above) so it does not pin the integrator to small, evenly-spaced
    # steps that would distort CENIC's step selection and timing.
    def monitor(root_context):
        diagram.ForcedPublish(root_context)
        return EventStatus.Succeeded()

    simulator.set_monitor(monitor)

simulator.Initialize()

if args.visualize:
    input("Waiting for meshcat... press [ENTER] to continue")

# Run the simulation
if args.visualize:
    meshcat.StartRecording()
start_time = time.time()
simulator.AdvanceTo(100.0)
wall_time = time.time() - start_time
if args.visualize:
    meshcat.StopRecording()
    meshcat.PublishRecording()

# Save recorded times
if args.log_times:
    time_logger.SaveRecordedTimes()

print("")
print(f"Wall time : {wall_time:.4f} seconds")
print(f"Sim time  : {context.get_time():.4f} seconds")
print("")

PrintSimulatorStatistics(simulator)

if args.visualize:
    input("\nWaiting for meshcat... press [ENTER] to quit")
