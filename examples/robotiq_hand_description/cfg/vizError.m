% vizualizer closes
% error not present if 'package://robotiq_hand_description' removed

plant = RigidBodyManipulator('s-model_articulated.urdf');
viz = plant.constructVisualizer();