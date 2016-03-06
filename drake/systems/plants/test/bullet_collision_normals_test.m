function bullet_collision_normals_test

r = RigidBodyManipulator();
r = addRobotFromURDF(r,'brick_point_contact.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
r = addRobotFromURDF(r,'ground_plane.urdf',zeros(3,1),zeros(3,1),struct('floating',false));
r = TimeSteppingRigidBodyManipulator(r,0.005);

rsim = RigidBodyManipulator();
rsim = addRobotFromURDF(rsim,'brick_point_contact.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
rsim = TimeSteppingRigidBodyManipulator(rsim,0.005);

x0 = zeros(12,1);
x0([3,4,5,6]) = [5; randn(3,1)];

v = rsim.constructVisualizer();

xtraj = rsim.simulate([0 3],x0);
v.playback(xtraj);


ground = r.findLinkId('world+ground_plane');
brick = r.findLinkId('brick');

cpts = 1:size(getBodyContacts(r,brick),2);

for t=xtraj.getBreaks()
  x=xtraj.eval(t);
  [phi,n] = r.pairwiseContactConstraints(x(1:6),brick,ground,cpts);
  [phi2,n2] = r.pairwiseContactConstraints(x(1:6),brick,-1,cpts); % check point against world 
  valuecheck(phi,phi2);
  valuecheck(n,n2);

  [~,n2] = r.contactConstraints(x(1:6),brick,cpts);
  valuecheck(phi(phi<0.01),phi2(phi<0.01),1e-4);
  valuecheck(n(phi<0.01,:),n2(phi<0.01,:),1e-4);

end

% NOTEST  