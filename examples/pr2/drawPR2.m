function drawPR2

r = RigidBodyManipulator('pr2.urdf');
v = r.constructVisualizer();


% NOTEST (build server can't handle vrml)