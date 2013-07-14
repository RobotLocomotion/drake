function drawPR2

r = RigidBodyManipulator('pr2.urdf');

if checkDependency('vrml')
  v = r.constructVisualizer();
end
