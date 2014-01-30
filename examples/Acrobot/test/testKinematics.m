function testKinematics

w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
m = PlanarRigidBodyManipulator('../Acrobot.urdf');
warning(w);
b = getBody(m,1);
b.contact_pts = randn(3,4);
m = setBody(m,1,b);
b = getBody(m,2);
b.contact_pts = randn(3,4);
m = setBody(m,2,b);
m = compile(m);
options.grad_method = {'user','taylorvar'};

lower_link_id = findLinkInd(m,'lower_link');
hand_id = findFrameId(m,'hand');

for i=1:100
  q = randn(2,1); 
  [x,J] = geval(1,@contactPositions,m,q,options);
  
  kinsol = doKinematics(m,q,true,true);
  hand_frame_xyz = [0;0;-2.1];  % note: this must match the frame specified in the urdf (and rpy must be zero)
  valuecheck(forwardKin(m,kinsol,lower_link_id,hand_frame_xyz),forwardKin(m,kinsol,hand_id,zeros(3,1)));
  valuecheck(forwardKin(m,kinsol,lower_link_id,hand_frame_xyz,1),forwardKin(m,kinsol,hand_id,zeros(3,1),1));
  valuecheck(forwardKin(m,kinsol,lower_link_id,hand_frame_xyz,2),forwardKin(m,kinsol,hand_id,zeros(3,1),2));
end

end