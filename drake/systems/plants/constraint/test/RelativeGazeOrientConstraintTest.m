function RelativeGazeOrientConstraintTest(varargin)
%NOTEST
% Andres would update the GazeOrientConstraint
  r = RigidBodyManipulator(strcat(getDrakePath(),'/examples/PR2/pr2.urdf'));
  q_nom = zeros(r.getNumPositions(),1);
  constraintTester('RelativeGazeOrientConstraintTest', r, @makeCon, @(r) q_nom, @(r) q_nom, varargin{:});
end

function con = makeCon(r)
  bodyA_idx = findLinkId(r,'r_gripper_palm_link');
  bodyB_idx = findLinkId(r,'l_gripper_palm_link');
  %rpy = [pi/2;pi/2;0];
  %rpy = 2*pi*rand(3,1) - pi;
  rpy = [2*pi;pi;0].*rand(3,1) - [pi;pi/2;0];
  quat = rpy2quat(rpy);
  [x,y,z] = sph2cart(pi*rand-pi/2,pi*rand-pi/2,1);
  %x = 1; y = 0; z = 0;
  ax = [x;y;z]
  %con = RelativeGazeOrientConstraint(r,[0 1],bodyA_idx,bodyB_idx,ax,quat,0,0)
  con = RelativeGazeOrientConstraint(r,bodyA_idx,bodyB_idx,ax,quat,pi,pi/12,[0 1])
end
