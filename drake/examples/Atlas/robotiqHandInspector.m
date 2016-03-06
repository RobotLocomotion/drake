function robotiqHandInspector

r = RigidBodyManipulator('urdf/robotiq.urdf');

actuated_dof = getActuatedJoints(r);
v = r.constructVisualizer();
v.inspector([],actuated_dof);
