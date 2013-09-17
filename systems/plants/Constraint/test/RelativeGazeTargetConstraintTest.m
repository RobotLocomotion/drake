function RelativeGazeTargetConstraintTest(varargin)
  r = RigidBodyManipulator(strcat(getDrakePath(),'/examples/PR2/pr2.urdf'));
  q_nom = zeros(r.getNumDOF(),1);
  constraintTester('RelativeGazeTargetConstraintTest', r, @makeCon, @(r) q_nom, @(r) q_nom, varargin{:});
end

function con = makeCon(r)
  body_a_idx = findLinkInd(r,'head_tilt_link');
  body_b_idx = findLinkInd(r,'l_gripper_palm_link');
  target = 0.2*rand(3,1) - 0.1;
  gaze_origin = [0;0;0];
  %gaze_origin = 0.1*rand(3,1) - 0.05;
  %[x,y,z] = sph2cart(pi*rand-pi/2,pi*rand-pi/2,1);
  x = 1; y = 0; z = 0;
  ax = [x;y;z];
  con = RelativeGazeTargetConstraint(r,[0 1],body_a_idx,body_b_idx,ax,target,gaze_origin,0,pi/32);
end

