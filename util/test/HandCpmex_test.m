function HandCpmex_test

m = PlanarRigidBodyModel('../../examples/Acrobot/Acrobot.urdf');
%m = PlanarRigidBodyModel('/Users/russt/locomotion/drc/ros_workspace/atlas_description/urdf/atlas_robot.urdf');

checkDependency('eigen3_enabled');
ptr=HandCpmex(struct(m),m.gravity);

for i=1:1000
  q = randn(m.featherstone.NB,1);
  qd = randn(m.featherstone.NB,1);
  
  [H1,C1] = HandCp(m.featherstone,q,qd,[],m.gravity);
  [H2,C2] = HandCpmex(ptr,q,qd);
  
  valuecheck(H1,H2);
  valuecheck(C1,C2);
end