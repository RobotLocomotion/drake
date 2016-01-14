function RelativePositionConstraintTest(varargin)
  w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
  warning('off','Drake:RigidBodyManipulator:BodyHasZeroInertia');
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  r = RigidBodyManipulator(strcat(getDrakePath(),'/examples/PR2/pr2.urdf'));
  warning(w);
  q_nom = getZeroConfiguration(r);
  constraintTester('RelativePositionConstraintTest', r, @makeCon, @(r) q_nom, @(r) q_nom, 10, varargin{:});
end

function con = makeCon(r)
  %n_pts = 4;
  bodyA_idx = findLinkId(r,'r_gripper_palm_link');
  bodyB_idx = findLinkId(r,'l_gripper_palm_link');
  rpy = 2*pi*rand(3,1) - pi;
  xyz = [0.2;0.2;0.2].*rand(3,1) - [0;0.1;0.1];
  lb = [-0.1;-0.1;-0.1];
  ub = [0.1;0.1;0.1];
  %pts = bsxfun(@minus,bsxfun(@times,(ub-lb),rand(3,n_pts)),(ub-lb)/2);
  %pts = 0.8*[[0;1;1],[0;-1;1],[0;-1;-1],[0;1;-1]].*repmat((ub-lb)/2,1,4);
  pts = 0.4*rand(3,1) - 0.2;

  T = [xyz;rpy2quat(rpy)];
%   T = [rpy2rotmat(rpy),xyz;zeros(1,3),1];

  con = RelativePositionConstraint(r,pts,lb,ub,bodyA_idx,bodyB_idx,T,[0 1]);
end
