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


