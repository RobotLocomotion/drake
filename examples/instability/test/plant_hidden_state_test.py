import copy
import dataclasses as dc

import numpy as np

from pydrake.all import (
    AddMultibodyPlant,
    BasicVector,
    ConnectContactResultsToDrakeVisualizer,
    DiagramBuilder,
    EventStatus,
    LeafSystem,
    MultibodyPlantConfig,
    Parser,
    PortDataType,
    Simulator,
    Value,
)


class ModifiedZoh(LeafSystem):
    """
    Modified zero-order hold that adds an initialization event.
    e.g. https://github.com/RobotLocomotion/drake/pull/18356
    """
    def __init__(self, period_sec, vector_size):
        super().__init__()
        self.DeclareVectorInputPort("u", BasicVector(vector_size))
        state_index = self.DeclareDiscreteState(vector_size)

        def update(context, discrete_state):
            input = self.get_input_port().Eval(context)
            discrete_state.set_value(0, input)

        self.DeclareInitializationDiscreteUpdateEvent(update)
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec,
            0.0,
            update,
        )
        self.DeclareStateOutputPort("y", state_index)


def attach_zoh(builder, output_port, dt):
    assert dt > 0
    assert output_port.get_data_type() != PortDataType.kAbstractValued
    zoh = ModifiedZoh(dt, output_port.size())
    builder.AddSystem(zoh)
    builder.Connect(output_port, zoh.get_input_port())
    return zoh.get_output_port()


class LatchInitial(LeafSystem):
    def __init__(self, vector_size):
        super().__init__()
        self.DeclareVectorInputPort("u", BasicVector(vector_size))
        state_index = self.DeclareDiscreteState(vector_size)

        def init(context, discrete_state):
            input = self.get_input_port().Eval(context)
            discrete_state.set_value(0, input)

        self.DeclareInitializationDiscreteUpdateEvent(init)
        self.DeclareStateOutputPort("y", state_index)

    @staticmethod
    def AddToBuilder(builder, output_port, input_port):
        model_value = output_port.Allocate()
        assert isinstance(model_value, Value[BasicVector])
        latch = LatchInitial(model_value.get_value().size())
        builder.AddSystem(latch)
        builder.Connect(output_port, latch.get_input_port())
        builder.Connect(latch.get_output_port(), input_port)
        return latch


def eval_port(port, parent_context):
    context = port.get_system().GetMyContextFromRoot(parent_context)
    return port.Eval(context)


class SimpleController(LeafSystem):
    def __init__(self):
        super().__init__()
        K = 1.0

        self.x_actual = self.DeclareVectorInputPort("x_actual", size=2)
        self.x_desired = self.DeclareVectorInputPort("x_desired", size=2)

        def calc_force(context, output):
            q_a, _ = self.x_actual.Eval(context)
            q_d, _ = self.x_desired.Eval(context)
            u = -K * (q_a - q_d)
            output.set_value([u])

        self.u_desired = self.DeclareVectorOutputPort(
            "forces_output", 1, calc=calc_force,
        )


# Needs collision.
MODEL_TEXT = """\
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="object">
    <joint name="z_axis" type="prismatic">
      <parent>world</parent>
      <child>object</child>
      <axis>0 0 1</axis>
    </joint>
    <link name="object">
      <collision name="collision">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""


class BadVelocity(Exception):
    pass


@dc.dataclass
class Setup:
    discrete_plant: bool
    add_contact_viz: bool
    add_zoh: bool
    add_latch: bool
    clean_plant_state: bool


def change_one_field_at_time(base, other):
    out = []
    for field in dc.fields(base):
        new = copy.deepcopy(base)
        other_value = getattr(other, field.name)
        setattr(new, field.name, other_value)
        out.append(new)
    return out


def run(name, setup, should_fail=False):
    print(f"[ {name} ]")
    print(setup)
    print()

    try:
        run_inner(setup)
        if should_fail:
            raise RuntimeError("Did not fail as expected!")
        else:
            print("Good!")
    except BadVelocity as e:
        if should_fail:
            print(f"Failed as expected: {repr(e)}")
        else:
            raise
    print()


def clean_plant_state(plant, context):
    x = plant.GetPositionsAndVelocities(context)
    context.SetStateAndParametersFrom(plant.CreateDefaultContext())
    plant.SetPositionsAndVelocities(context, x)


def run_inner(setup):
    time_step = 0.25
    if setup.discrete_plant:
        plant_time_step = time_step  # any non-zero time step
    else:
        plant_time_step = 0.0
    builder = DiagramBuilder()
    config = MultibodyPlantConfig(
        time_step=plant_time_step,
        discrete_contact_solver="sap",  # or "tamsi",
        contact_model="hydroelastic",  # any contact model with drake#18647
    )
    plant, scene_graph = AddMultibodyPlant(config, builder)
    # Disable gravity.
    plant.mutable_gravity_field().set_gravity_vector(np.zeros(3))
    # Add model.
    model, = Parser(plant).AddModelsFromString(MODEL_TEXT, "sdf")
    plant.Finalize()
    if setup.add_contact_viz:
        ConnectContactResultsToDrakeVisualizer(
            builder,
            plant,
            scene_graph,
            publish_period=time_step,
        )

    # Add controller.
    controller = builder.AddSystem(SimpleController())
    u_desired_input = plant.get_actuation_input_port(model)
    x_actual_output = plant.get_state_output_port()
    if setup.add_zoh:
        u_desired_output = attach_zoh(
            builder, controller.u_desired, time_step
        )
    else:
        u_desired_output = controller.u_desired
    builder.Connect(u_desired_output, u_desired_input)
    builder.Connect(x_actual_output, controller.x_actual)
    if setup.add_latch:
        LatchInitial.AddToBuilder(
            builder, x_actual_output, controller.x_desired
        )
    else:
        builder.Connect(x_actual_output, controller.x_desired)

    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()

    context = plant.GetMyMutableContextFromRoot(diagram_context)
    # This must be non-default (zero) value.
    z0 = 0.125
    plant.SetPositions(context, [z0])

    def monitor(diagram_context):
        t = diagram_context.get_time()
        u = eval_port(u_desired_input, diagram_context)
        x_a = eval_port(controller.x_actual, diagram_context)
        print(f"  t: {t}")
        print(f"  x_a: {x_a}")
        print(f"  u: {u}")
        q_a, v_a = x_a
        if v_a != 0.0:
            raise BadVelocity()
        return EventStatus.DidNothing()

    simulator = Simulator(diagram, diagram_context)
    simulator.set_monitor(monitor)

    # Initialize twice per
    # https://github.com/RobotLocomotion/drake/pull/18551#issuecomment-1384319905
    for i in range(2):
        print(f"Initialize: {i}")
        simulator.Initialize()

    print("Post-Init")
    monitor(simulator.get_context())

    if setup.clean_plant_state:
        clean_plant_state(plant, context)

    print("Run")
    simulator.AdvanceTo(time_step)


def main():
    all_good = Setup(
        discrete_plant=False,
        add_contact_viz=False,
        add_zoh=False,
        add_latch=False,
        clean_plant_state=True,
    )

    bad = Setup(
        discrete_plant=True,
        add_contact_viz=True,
        add_zoh=True,
        add_latch=True,
        clean_plant_state=False,
    )

    # Changing any of the bad flags makes it work.
    goods = change_one_field_at_time(bad, all_good)
    for i, good in enumerate(goods):
        run(f"good {i}", good)

    run("bad", bad, should_fail=True)


if __name__ == "__main__":
    main()
