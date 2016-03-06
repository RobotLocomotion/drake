function RelativeGazeTargetConstraintTest()
% Andres would update the RelativeGazeTargetConstraint
  w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
  warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:BodyHasZeroInertia');
  warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
  r = RigidBodyManipulator(fullfile(getDrakePath(),'examples','PR2','pr2.urdf'));
  warning(w);
  q_nom = getZeroConfiguration(r);
  constraintTester('RelativeGazeTargetConstraintTest', r, @makeCon, @(r) q_nom, @(r) q_nom, [],[],[],[],false);
end

function con = makeCon(r)
  body_a_idx = findLinkId(r,'head_tilt_link');
  body_b_idx = findLinkId(r,'l_gripper_palm_link');
  target = 0.2*rand(3,1) - 0.1;
  gaze_origin = [0;0;0];
  %gaze_origin = 0.1*rand(3,1) - 0.05;
  %[x,y,z] = sph2cart(pi*rand-pi/2,pi*rand-pi/2,1);
  x = 1; y = 0; z = 0;
  ax = [x;y;z];
  con = RelativeGazeTargetConstraint(r,body_a_idx,body_b_idx,ax,target,gaze_origin,pi/32,[0 1]);
end
