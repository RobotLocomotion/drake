function drawPR2

options.terrain = RigidBodyFlatTerrain();
r = RigidBodyManipulator('pr2.urdf',options);
v = r.constructVisualizer();
