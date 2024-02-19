import time
from pydrake.common.value import Value
from pydrake.geometry import StartMeshcat, SceneGraphConfig
from pydrake.math import RigidTransform
from pydrake.multibody.meshcat import (
    ContactVisualizer, ContactVisualizerParams)
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlant, MultibodyPlantConfig, ContactModel, ContactResults)
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem
from pydrake.visualization import ApplyVisualizationConfig, VisualizationConfig


# Create a table top
table_top_sdf = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="TableTop">
    <link name="table_top_link">
      <visual name="visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.6 1.0 0.05</size>
          </box>
        </geometry>
        <material>
         <diffuse>0.9 0.8 0.7 0.5</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.6 1.0 0.05</size>
          </box>
        </geometry>
      </collision>
    </link>
    <frame name="top_surface">
      <pose relative_to="table_top_link">0 0 0.025 0 0 0</pose>
    </frame>
  </model>
</sdf>
"""

free_body_sdf = """\
<?xml version="1.0"?>
<sdf version='1.9'>
<model name='mustard'>
<include>
  <uri>package://drake/manipulation/models/ycb/sdf/006_mustard_bottle.sdf</uri>
</include>
</model>
</sdf>
"""

def clear_meshcat():
    # Clear MeshCat window from the previous blocks.
    meshcat.Delete()
    meshcat.DeleteAddedControls()

def add_scene(time_step=1e-3, solver="tamsi", free_body_name="compliant_box"):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlant(
        MultibodyPlantConfig(
            time_step=time_step,
            discrete_contact_solver=solver),
        builder)
    parser = Parser(plant)

    # Load the table top and the box we created.
    parser.AddModelsFromString(free_body_sdf, "sdf")
    parser.AddModelsFromString(table_top_sdf, "sdf")

    # Weld the rigid box to the world so that it's fixed during simulation.
    # The top surface passes the world's origin.
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("top_surface"))

    # Finalize the plant after loading the scene.
    plant.set_contact_model(ContactModel.kHydroelastic)
    config = SceneGraphConfig()
    config.default_proximity_properties.compliance_type = "compliant"
    scene_graph.set_config(config)
    plant.Finalize()

    # Set how high the center of the compliant box is from the world's origin.
    # W = the world's frame
    # C = frame at the center of the compliant box
    X_WC = RigidTransform(p=[0, 0, 1])
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName(free_body_name), X_WC)

    return builder, plant


def add_viz(builder, plant):
    ApplyVisualizationConfig(
        config=VisualizationConfig(
                   publish_period = 1 / 256.0,
                   publish_contacts = False),
        builder=builder, meshcat=meshcat)

    return builder, plant



class ContactReporter(LeafSystem):
    def __init__(self):
        super().__init__()  # Don't forget to initialize the base class.
        self.DeclareAbstractInputPort(
            name="contact_results",
            model_value=Value(
                # Input port will take ContactResults from MultibodyPlant
                ContactResults()))
        # Calling `ForcedPublish()` will trigger the callback.
        self.DeclareForcedPublishEvent(self.Publish)

    def Publish(self, context):
        print()
        print(f"ContactReporter::Publish() called at time={context.get_time()}")
        contact_results = self.get_input_port().Eval(context)

        num_hydroelastic_contacts = contact_results.num_hydroelastic_contacts()
        print(f"num_hydroelastic_contacts() = {num_hydroelastic_contacts}")

        for c in range(num_hydroelastic_contacts):
            print()
            print(f"hydroelastic_contact_info({c}): {c}-th hydroelastic contact patch")
            hydroelastic_contact_info = contact_results.hydroelastic_contact_info(c)

            spatial_force = hydroelastic_contact_info.F_Ac_W()
            print("F_Ac_W(): spatial force (on body A, at centroid of contact surface, in World frame) = ")
            print(f"{spatial_force}")

            print("contact_surface()")
            contact_surface = hydroelastic_contact_info.contact_surface()
            num_faces = contact_surface.num_faces()
            total_area = contact_surface.total_area()
            centroid = contact_surface.centroid()
            print(f"total_area(): area of contact surface in m^2 = {total_area}")
            print(f"num_faces(): number of polygons or triangles = {num_faces}")
            print(f"centroid(): centroid (in World frame) = {centroid}")

        print()

def add_contact_report(builder, plant):
    contact_reporter = builder.AddSystem(ContactReporter())
    builder.Connect(plant.get_contact_results_output_port(),
                    contact_reporter.get_input_port(0))

    return builder, plant

def add_contact_viz(builder, plant):
    contact_viz = ContactVisualizer.AddToBuilder(
        builder, plant, meshcat,
        ContactVisualizerParams(
            publish_period= 1.0 / 256.0,
            newtons_per_meter= 2e1,
            newton_meters_per_meter= 1e-1))

    return builder, plant


def run_simulation_with_contact_report_and_viz(sim_time, time_step=1e-3, solver="sap", free_body_name="compliant_box"):
    clear_meshcat()

    builder, plant = add_scene(time_step, solver, free_body_name)
    add_viz(builder, plant)
    add_contact_report(builder, plant)
    add_contact_viz(builder, plant)

    diagram = builder.Build()

    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)

    meshcat.StartRecording(frames_per_second=256.0)
    simulator.AdvanceTo(sim_time)
    meshcat.StopRecording()

    # Numerically report contact results at the end of simulation.
    diagram.ForcedPublish(simulator.get_context())

# Start the visualizer. The cell will output an HTTP link after the execution.
# Click the link and a MeshCat tab should appear in your browser.
meshcat = StartMeshcat()

while meshcat.GetNumActiveConnections() <= 0:
    time.sleep(0.01)

run_simulation_with_contact_report_and_viz(sim_time=2, free_body_name="base_link_mustard")
# meshcat.Delete("/drake/contact_forces/hydroelastic/compliant_box+table_top_link/contact_surface")
meshcat.PublishRecording()

while meshcat.GetNumActiveConnections() > 0:
    time.sleep(0.01)

