function bullet_collision_normals_test

r = RigidBodyManipulator();
r = addRobotFromURDF(r,'FallingBrick.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
r = addRobotFromURDF(r,'ground_plane.urdf',zeros(3,1),zeros(3,1),struct('floating',false));
r = TimeSteppingRigidBodyManipulator(r,0.005);

x0 = zeros(12,1);
x0([3,4,5,6]) = [5; randn(3,1)];

v = r.constructVisualizer();

xtraj = r.simulate([0 4],x0);
v.playback(xtraj);


ground = r.findLinkInd('world+ground_plane');
brick = r.findLinkInd('brick');


for t=xtraj.getBreaks()
  x=xtraj.eval(t);
  [phi,n] = r.pairwiseContactConstraints(x(1:6),brick,ground);
  if phi ~= 1
    [phi2,n2] = r.contactConstraints(x(1:6),brick);

    % this isn't perfect yet... need to do some book keeping to make sure
    % points are in the right order
    
    valuecheck(n,n2(phi2<1e-8,:),1e-5);
  end

end
