function bullet_collision_test

r = RigidBodyManipulator();

for i=1:2
  r = addRobotFromURDF(r,'FallingBrick.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
end

v = r.constructVisualizer();

for i=1:20
  q = randn(getNumDOF(r),1);
  v.draw(0,[q;0*q]);
  
  kinsol = doKinematics(r,q);
  
  b = pairwiseContactTest(r,2,3)
  pause;
end