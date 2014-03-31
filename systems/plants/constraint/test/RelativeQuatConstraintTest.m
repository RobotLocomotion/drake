function RelativeQuatConstraintTest(varargin)
% Andres would update RelativeQuatConstraint
  r = RigidBodyManipulator(strcat(getDrakePath(),'/examples/PR2/pr2.urdf'));
  q_nom = zeros(r.getNumDOF(),1);
  constraintTester('RelativeOrientConstraintTest', r, @makeCon, @(r) q_nom, @(r) q_nom, 10, varargin{:});
end

function con = makeCon(r)
  bodyA_idx = findLinkInd(r,'r_gripper_palm_link');
  bodyB_idx = findLinkInd(r,'l_gripper_palm_link');
  rpy = 2*pi*rand(3,1) - pi;
  quat = rpy2quat(rpy);
  con = RelativeQuatConstraint(r,bodyA_idx,bodyB_idx,quat,0,[0,1]);
end
