function testFlipLeftRight()

options.twoD = true;
options.floating = true;
s = 'urdf/atlas_simple_spring_ankle.urdf';
dt = 0.001;
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
r = TimeSteppingRigidBodyManipulator(s,dt,options);

load data/atlas_passiveankle_traj_lqr_082914_2.mat

xtraj2 = xtraj;
utraj2 = utraj;
Btraj2 = Btraj;
Straj_full2 = Straj_full;

[xtraj2,utraj2,Btraj2,Straj_full2] = flipLeftRight(r,xtraj2,utraj2,Btraj2,Straj_full2);
[xtraj2,utraj2,Btraj2,Straj_full2] = flipLeftRight(r,xtraj2,utraj2,Btraj2,Straj_full2);

for i=1:10
  t = rand();
  valuecheck(xtraj2.eval(t),xtraj.eval(t));
  valuecheck(utraj2.eval(t),utraj.eval(t));
  valuecheck(Straj_full2{1}.eval(t),Straj_full{1}.eval(t));
  valuecheck(Btraj2{1}.eval(t),Btraj{1}.eval(t));
end

end

