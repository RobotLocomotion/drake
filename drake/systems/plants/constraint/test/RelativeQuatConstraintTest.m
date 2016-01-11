function RelativeQuatConstraintTest(varargin)
% Andres would update RelativeQuatConstraint
  w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
  warning('off','Drake:RigidBodyManipulator:BodyHasZeroInertia');
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  r = RigidBodyManipulator(strcat(getDrakePath(),'/examples/PR2/pr2.urdf'));
  warning(w);
  q_nom = getZeroConfiguration(r);
  constraintTester('RelativeOrientConstraintTest', r, @makeCon, @(r) q_nom, @(r) q_nom, 10, varargin{:});
end

function con = makeCon(r)
  w = warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
  bodyA_idx = findLinkId(r,'r_gripper_palm_link');
  bodyB_idx = findLinkId(r,'l_gripper_palm_link');
  warning(w);
  rpy = 2*pi*rand(3,1) - pi;
  quat = rpy2quat(rpy);
  con = RelativeQuatConstraint(r,bodyA_idx,bodyB_idx,quat,0,[0,1]);
end
