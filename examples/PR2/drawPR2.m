function drawPR2
% NOTEST
r = RigidBodyManipulator('pr2.urdf');

if checkDependency('vrml_enabled')
  v = r.constructVisualizer();
end
