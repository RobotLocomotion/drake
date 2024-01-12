from pydrake.geometry import StartMeshcat
from pydrake.systems.analysis import Simulator
from pydrake.visualization import ModelVisualizer
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import ApplyVisualizationConfig, VisualizationConfig
from pydrake.multibody.meshcat import (ContactVisualizer,
                                       ContactVisualizerParams)

# Start the visualizer. The cell will output an HTTP link after the execution.
# Click the link and a MeshCat tab should appear in your browser.
meshcat = StartMeshcat()

visual_bell_pepper_sdf = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="BellPepper">
    <pose>0 0 0 0 0 0</pose>
    <link name="bell_pepper">
      <visual name="yellow_bell_pepper_no_stem">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>package://drake_models/veggies/yellow_bell_pepper_no_stem_low.obj</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""

# Visualize the SDFormat string you just defined.
visualizer = ModelVisualizer(meshcat=meshcat, visualize_frames=True)
visualizer.parser().AddModelsFromString(visual_bell_pepper_sdf, "sdf")
visualizer.Run(loop_once=True)


collision_bell_pepper_sdf = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="BellPepper">
    <pose>0 0 0 0 0 0</pose>
    <link name="bell_pepper">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>package://drake_models/veggies/yellow_bell_pepper_no_stem_low.vtk</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
        <drake:proximity_properties>
          <drake:compliant_hydroelastic/>
          <drake:hydroelastic_modulus>1e6</drake:hydroelastic_modulus>
          <drake:mu_dynamic>0.5</drake:mu_dynamic>
          <drake:hunt_crossley_dissipation>
            1.25
          </drake:hunt_crossley_dissipation>
        </drake:proximity_properties>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>package://drake_models/veggies/yellow_bell_pepper_no_stem_low.obj</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""

# Visualize the SDFormat string you just defined.
visualizer = ModelVisualizer(meshcat=meshcat, visualize_frames=True)
visualizer.parser().AddModelsFromString(collision_bell_pepper_sdf, "sdf")
visualizer.Run(loop_once=True)


# Define a compliant-hydroelastic bell pepper from
# a given hydroelastic modulus.
def bell_pepper(hydroelastic_modulus='1e6'):
    return f"""<?xml version="1.0"?>
    <sdf version="1.7">
      <model name="BellPepper">
        <pose>0 0 0 0 0 0</pose>
        <link name="bell_pepper">
          <inertial>
            <pose>0.000537 -0.00272 0.0384 0 0 0</pose>
            <mass>0.159</mass>
            <inertia>
              <ixx> 0.000101</ixx>
              <ixy>-0.000001</ixy>
              <ixz>-0.000004</ixz>
              <iyy> 0.000105</iyy>
              <iyz> 0.000007</iyz>
              <izz> 0.000107</izz>
            </inertia>
          </inertial>
          <collision name="collision">
            <geometry>
              <mesh>
                <uri>package://drake_models/veggies/yellow_bell_pepper_no_stem_low.vtk</uri>
                <scale>1 1 1</scale>
              </mesh>
            </geometry>
            <drake:proximity_properties>
              <drake:compliant_hydroelastic/>
              <drake:hydroelastic_modulus>{hydroelastic_modulus}</drake:hydroelastic_modulus>
              <drake:mu_dynamic>0.5</drake:mu_dynamic>
              <drake:hunt_crossley_dissipation>1.25</drake:hunt_crossley_dissipation>
            </drake:proximity_properties>
          </collision>
          <visual name="visual">
            <geometry>
              <mesh>
                <uri>package://drake_models/veggies/yellow_bell_pepper_no_stem_low.obj</uri>
                <scale>1 1 1</scale>
              </mesh>
            </geometry>
            <material>
              <diffuse>1.0 1.0 1.0 0.5</diffuse>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
    """


bell_pepper_sdf = bell_pepper()

# Visualize the SDFormat string you just defined.
visualizer = ModelVisualizer(meshcat=meshcat, visualize_frames=True)
visualizer.parser().AddModelsFromString(bell_pepper_sdf, "sdf")
visualizer.Run(loop_once=True)


# Create a rigid-hydroelastic table top
table_top_urdf = """<?xml version="1.0"?>
  <robot name="TableTop">
    <link name="table_top_link">
      <visual name="visual">
        <geometry>
          <box size="0.6 1.0 0.05"/>
        </geometry>
        <material>
         <color rgba="0.9 0.8 0.7 0.5"/>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box size="0.6 1.0 0.05"/>
        </geometry>
        <drake:proximity_properties>
          <drake:compliant_hydroelastic/>
          <drake:hydroelastic_modulus value="1e7"/>
          <drake:mu_dynamic value="0.5"/>
          <drake:hunt_crossley_dissipation value="1.25"/>
        </drake:proximity_properties>
      </collision>
    </link>
    <frame name="top_surface" link="table_top_link" xyz="0 0 0.025"
           rpy="0 0 0"/>
  </robot>
"""

# Visualize the URDF string you just defined.
visualizer = ModelVisualizer(meshcat=meshcat, visualize_frames=True)
visualizer.parser().AddModelsFromString(table_top_urdf, "urdf")
visualizer.Run(loop_once=True)

bowl_urdf = """<?xml version="1.0"?>
  <robot name="Bowl">
    <link name="bowl">
      <inertial>
        <mass value="0.5"/>
        <origin xyz="0 0 -0.00613"/>
        <inertia ixx="0.00389"
                 ixy="0.00000"
                 ixz="0.00000"
                 iyy="0.00392"
                 iyz="0.00000"
                 izz="0.00637"/>
      </inertial>
      <visual name="visual">
        <geometry>
          <mesh filename=
            "package://drake_models/dishes/bowls/evo_bowl_no_mtl.obj"/>
        </geometry>
        <material>
         <color rgba="0.9 0.8 0.7 0.5"/>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <mesh filename="package://drake_models/dishes/bowls/evo_bowl.vtk"/>
        </geometry>
        <drake:proximity_properties>
          <drake:compliant_hydroelastic/>
          <drake:hydroelastic_modulus value="1e7"/>
          <drake:mu_dynamic value="0.5"/>
          <drake:hunt_crossley_dissipation value="1.25"/>
        </drake:proximity_properties>
      </collision>
    </link>
  </robot>
"""

# Visualize the URDF string you just defined.
visualizer = ModelVisualizer(meshcat=meshcat, visualize_frames=True)
visualizer.parser().AddModelsFromString(bowl_urdf, "urdf")
visualizer.Run(loop_once=True)


def clear_meshcat():
    # Clear MeshCat window from the previous blocks.
    meshcat.Delete()
    meshcat.DeleteAddedControls()


def add_scene(time_step):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlant(
        MultibodyPlantConfig(
            time_step=time_step,
            discrete_contact_approximation="similar"),
        builder)
    parser = Parser(plant)

    # Load the table top and the box we created.
    parser.AddModelsFromString(table_top_urdf, "urdf")
    parser.AddModelsFromString(bowl_urdf, "urdf")
    parser.AddModelsFromString(bell_pepper_sdf, "sdf")

    # Weld the table top to the world so that it's fixed during simulation.
    # The top surface passes the world's origin.
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("top_surface"))

    # Finalize the plant after loading the scene.
    plant.Finalize()

    # Set initial position of the bowl
    X_WB = RigidTransform(p=[0, 0, 0.061])
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName("bowl"), X_WB)

    # Set how high the center of the bell pepper is from the world's origin.
    # W = the world's frame
    # C = frame at the center of the bell pepper
    X_WC = RigidTransform(p=[0, 0, 0.301])
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName("bell_pepper"), X_WC)

    return builder, plant


def add_viz(builder, plant):
    ApplyVisualizationConfig(
        config=VisualizationConfig(
            publish_period=1 / 32.0,
            publish_contacts=False),
        builder=builder, meshcat=meshcat)

    return builder, plant


def add_contact_viz(builder, plant):
    contact_viz = ContactVisualizer.AddToBuilder(
        builder, plant, meshcat,
        ContactVisualizerParams(
            publish_period=1.0 / 32.0,
            newtons_per_meter=2e1,
            newton_meters_per_meter=1e-1))

    return builder, plant


def run_simulation_with_contact_viz(sim_time, time_step=1e-2):
    clear_meshcat()

    builder, plant = add_scene(time_step)
    add_viz(builder, plant)
    add_contact_viz(builder, plant)

    diagram = builder.Build()

    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)

    meshcat.StartRecording(frames_per_second=64.0)
    simulator.AdvanceTo(sim_time)
    meshcat.StopRecording()


run_simulation_with_contact_viz(sim_time=2)
