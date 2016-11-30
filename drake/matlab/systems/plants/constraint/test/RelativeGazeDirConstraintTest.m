function RelativeGazeDirConstraintTest(varargin)
% NOTEST
% Andres would update the RelativeGazeDirConstraint
  r = RigidBodyManipulator(strcat(getDrakePath(),'/examples/PR2/pr2.urdf'));
  q_nom = getZeroConfiguration(r);
  constraintTester('RelativeGazeDirConstraintTest', r, @makeCon, @(r) q_nom, @(r) q_nom, varargin{:});
end

function con = makeCon(r)
  bodyA_idx = findLinkId(r,'r_gripper_palm_link');
  bodyB_idx = findLinkId(r,'l_gripper_palm_link');
  direction = [1;0;0];
  ax = rand(3,1)-[0;0.5;0.5];
  %x = 1; y = 0; z = 0;
  ax = ax/norm(ax);
  con = RelativeGazeDirConstraint(r,bodyA_idx,bodyB_idx,ax,direction,pi/12,[0 1]);
end
