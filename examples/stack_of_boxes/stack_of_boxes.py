time_step = 0.01 
sap_model = "similar" # sap (default), lagged, or similar
hydro_modulus = 1.0e8 # need 4e4 for stable contact
hc_dissipation = 0.0
relaxation_time = 0.1
sg_margin = 1e-4  # in mm.
stiction_tolerance = 1e-3 # no effect on instability 
sap_beta = 0.0
# cup_modulus = 6e5

import math
from pydrake.geometry import (StartMeshcat, SceneGraphConfig)

# Start the visualizer. The cell will output an HTTP link after the execution.
# Click the link and a MeshCat tab should appear in your browser.
meshcat = StartMeshcat()

# scratchwork for computing he modulus
h = 0.083 # 0.0826020
l = 0.23 # 0.2270280  
w = 0.15 # 0.1499300
area = l*w

m = 0.2
g = 9.81
# phi = 1/1000 # 1 mm
# e = m*g/2/l/phi
# print(e)
e = hydro_modulus
print(m*g*h/(area*2*e)) # predicted penetration in m

from pydrake.visualization import ModelVisualizer

# Create a table top
def rigid_box(hydro_modulus=hydro_modulus,hc_dissipation=hc_dissipation,relaxation_time=relaxation_time): 
    return f"""<?xml version="1.0"?>
    <sdf version="1.7">
    <model name="RigidBox">
        <link name="rigid_box_link">
            <visual name="visual">
                <pose>0 0 0 0 0 0</pose>
                <geometry>
                <box>
                    <size> 1.269 0.800 0.08 </size>
                </box>
                </geometry>
                <material>
                <diffuse>0.9 0.8 0.7 0.5</diffuse>
                </material>
            </visual>
            <collision name="base_collision">
                <geometry>
                    <box>
                        <size> 1.269 0.800 0.08 </size>
                    </box>
                </geometry>
                <drake:proximity_properties>
                    <drake:compliant_hydroelastic/>
                    <drake:hydroelastic_modulus>{hydro_modulus}</drake:hydroelastic_modulus>
                    <drake:hunt_crossley_dissipation>{hc_dissipation}</drake:hunt_crossley_dissipation>
                    <!-- Material properties: Metal w. paint coating -->
                    <drake:relaxation_time>{relaxation_time}</drake:relaxation_time>
                    <drake:mu_dynamic>0.3</drake:mu_dynamic>
                    <drake:mu_static>0.3</drake:mu_static>
                </drake:proximity_properties>
            </collision>
        </link>
        <frame name="top_surface">
        <pose relative_to="rigid_box_link">0 0 0.005 0 0 0</pose>
        </frame>
    </model>
    </sdf>
    """
rigid_box_sdf = rigid_box()

# Define a compliant-hydroelastic object
def box(hydro_modulus=hydro_modulus,hc_dissipation=hc_dissipation,relaxation_time=relaxation_time):
    return f"""<?xml version='1.0'?>
    <sdf version='1.6'>
    <!-- Auto generated with SdfGen.py -->
    <model name="box" xmlns:drake='drake'>
    <link name="box">
        <inertial>
        <inertia>
            <ixx>0.000491</ixx>
            <iyy>0.001240</iyy>
            <izz>0.000978</izz>
            <ixy>0.000000</ixy>
            <ixz>0.000000</ixz>
            <iyz>0.000000</iyz>
        </inertia>
        <pose>0 0 0 0 0 0</pose>
        <mass>0.201032</mass>
        </inertial>
        <visual name="box_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
            <box>
            <size>0.2 0.08 0.15</size>
            </box>
        </geometry>
            <material>
            <diffuse>1.0 0.0 0.0 0.9</diffuse>
            </material>
        </visual>
        <collision name="box_collision">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
            <box>
            <size>0.2 0.08 0.15</size>
            </box>
        </geometry>
        <drake:proximity_properties>
            <drake:compliant_hydroelastic/>
            <drake:hydroelastic_modulus>{hydro_modulus}</drake:hydroelastic_modulus>
            <drake:hunt_crossley_dissipation>{hc_dissipation}</drake:hunt_crossley_dissipation>
            <drake:relaxation_time>{relaxation_time}</drake:relaxation_time>
            <drake:mesh_resolution_hint>0.1</drake:mesh_resolution_hint>
            <drake:point_contact_stiffness>1000000.0</drake:point_contact_stiffness>
            <drake:mu_dynamic>0.4</drake:mu_dynamic>
            <drake:mu_static>0.4</drake:mu_static>
        </drake:proximity_properties>
        </collision>
    </link>
    </model>
    </sdf>
    """
box_sdf = box()

# Define a compliant-hydroelastic object
def second_box(hydro_modulus=hydro_modulus,hc_dissipation=hc_dissipation,relaxation_time=relaxation_time):
    return f"""<?xml version='1.0'?>
    <sdf version='1.6'>
    <!-- Auto generated with SdfGen.py -->
    <model name="box2" xmlns:drake='drake'>
    <link name="box2">
        <inertial>
        <inertia>
            <ixx>0.000491</ixx>
            <iyy>0.001240</iyy>
            <izz>0.000978</izz>
            <ixy>0.000000</ixy>
            <ixz>0.000000</ixz>
            <iyz>0.000000</iyz>
        </inertia>
        <pose>0 0 0 0 0 0</pose>
        <mass>0.201032</mass>
        </inertial>
        <visual name="box_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
            <box>
            <size>0.2 0.08 0.15</size>
            </box>
        </geometry>
            <material>
            <diffuse>0.0 0.0 1.0 0.9</diffuse>
            </material>
        </visual>
        <collision name="box_collision">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
            <box>
            <size>0.2 0.08 0.15</size>
            </box>
        </geometry>
        <drake:proximity_properties>
            <drake:compliant_hydroelastic/>
            <drake:hydroelastic_modulus>{hydro_modulus}</drake:hydroelastic_modulus>
            <drake:hunt_crossley_dissipation>{hc_dissipation}</drake:hunt_crossley_dissipation>
            <drake:relaxation_time>{relaxation_time}</drake:relaxation_time>
            <drake:mesh_resolution_hint>0.1</drake:mesh_resolution_hint>
            <drake:point_contact_stiffness>1000000.0</drake:point_contact_stiffness>
            <drake:mu_dynamic>0.4</drake:mu_dynamic>
            <drake:mu_static>0.4</drake:mu_static>
        </drake:proximity_properties>
        </collision>
    </link>
    </model>

    </sdf>
    """
second_box_sdf = second_box()

from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.systems.framework import DiagramBuilder

def clear_meshcat():
    # Clear MeshCat window from the previous blocks.
    meshcat.Delete()
    meshcat.DeleteAddedControls()

def add_scene(time_step=0.01): # time step Gemini would like to use
    builder = DiagramBuilder()
    plant_config = MultibodyPlantConfig(
            time_step=time_step,
            discrete_contact_approximation=sap_model,
            sap_near_rigid_threshold=sap_beta)
    sg_config = SceneGraphConfig()
    sg_config.default_proximity_properties.margin = sg_margin # in mm.

    plant, scene_graph = AddMultibodyPlant(
        plant_config, # Gemini currently uses Drake v1.23
        sg_config,
        builder)
    parser = Parser(plant)

    # Load the table top and the object we created.
    parser.AddModelsFromString(box_sdf, "sdf")
    parser.AddModelsFromString(second_box_sdf, "sdf")
    parser.AddModelsFromString(rigid_box_sdf, "sdf")

    # Weld rigid box to the world so that it's fixed during simulation.
    # The center of box passes through the world's origin.
    plant.WeldFrames(plant.world_frame(), 
                     plant.GetFrameByName("rigid_box_link"))

    # Finalize the plant after loading the scene.
    plant.set_stiction_tolerance(stiction_tolerance)
    plant.Finalize()

    # Set how high the center of the object is from the world's origin. 
    # W = the world's frame
    # C = frame at the center of the object
    drop_height = 0.08  # At 0.08, "box" seats right on top of the table.
    X_WC = RigidTransform(p=[0, 0, drop_height], rpy=RollPitchYaw([math.pi/2.0, 0, 0]))
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName("box"), X_WC)

    drop_height = 0.5 # at 0.16 "box2" would seat right on top of "box".
    X_WC = RigidTransform(p=[0, 0.0, drop_height], rpy=RollPitchYaw([math.pi/2.0, 0, 0]))
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName("box2"), X_WC)

    return builder, plant

from pydrake.visualization import ApplyVisualizationConfig, VisualizationConfig

def add_viz(builder, plant):
    ApplyVisualizationConfig(
        config=VisualizationConfig(
                   publish_period = 0.01,
                   publish_contacts = False),
        builder=builder, meshcat=meshcat)
    
    return builder, plant


from pydrake.systems.analysis import Simulator

# Test creation of the diagram by simulating for 0 second.
# For now, use only the DiagramBuilder from the first return value and
# ignore the other return value. We will use it later.
clear_meshcat()
builder, plant = add_scene()
add_viz(builder, plant)
simulator = Simulator(builder.Build())
simulator.AdvanceTo(0)

from pydrake.common.value import Value
from pydrake.multibody.plant import ContactResults
from pydrake.systems.framework import LeafSystem

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
            representation = contact_surface.representation()
            print(f"total_area(): area of contact surface in m^2 = {total_area}")
            print(f"num_faces(): number of polygons or triangles = {num_faces}")
            print(f"centroid(): centroid (in World frame) = {centroid}")  
            print(f"contact surface representation = {representation}")
        
        print()

def add_contact_report(builder, plant):   
    contact_reporter = builder.AddSystem(ContactReporter())    
    builder.Connect(plant.get_contact_results_output_port(),
                    contact_reporter.get_input_port(0))
        
    return builder, plant

from pydrake.multibody.meshcat import ContactVisualizer, ContactVisualizerParams


def add_contact_viz(builder, plant):
    contact_viz = ContactVisualizer.AddToBuilder(
        builder, plant, meshcat,
        ContactVisualizerParams(
            publish_period= 0.01,
            newtons_per_meter= 2e1,
            newton_meters_per_meter= 1e-1))

    return builder, plant

from pydrake.systems.analysis import Simulator

def run_simulation_with_contact_report_and_viz(sim_time, time_step=1e-2):
    clear_meshcat()
    
    builder, plant = add_scene(time_step)
    add_viz(builder, plant)
    add_contact_report(builder, plant)
    add_contact_viz(builder, plant)
    
    diagram = builder.Build()
    
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(0.5)
    
    meshcat.StartRecording(frames_per_second=100.0)
    simulator.AdvanceTo(sim_time)
    meshcat.StopRecording()

    # Numerically report contact results at the end of simulation.
    diagram.ForcedPublish(simulator.get_context())


run_simulation_with_contact_report_and_viz(sim_time=2, time_step=time_step) # Gemini wants to run at 10 ms

# In the current version, we can playback contact forces and torques;
# however, contact surfaces are not recorded properly.
# For now, we delete contact surfaces to prevent confusion.
# See issue https://github.com/RobotLocomotion/drake/issues/19142
meshcat.Delete("/drake/contact_forces/hydroelastic/box+rigid_box_link/contact_surface")

meshcat.PublishRecording()








