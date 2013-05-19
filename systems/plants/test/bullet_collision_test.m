function bullet_collision_test

r = RigidBodyManipulator();

for i=1:2
  r = addRobotFromURDF(r,'FallingBrick.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
end
%v = r.constructVisualizer();

v = BotVisualizer('FallingBrick.urdf',struct('floating',true));

lcmgl = bot_lcmgl_init('contact_points');

for i=1:20
  q = randn(getNumDOF(r),1);
  v.draw(0,[q;0*q]);
  
  kinsol = doKinematics(r,q);
  
  b = pairwiseContactTest(r,2,3)
  
  pts = contactPositions(r,kinsol);
  
  bot_lcmgl_color3f(lcmgl,0,0,1); 
  for j=1:size(pts,2)
    bot_lcmgl_sphere(lcmgl,pts(:,j),.05,36,36);
  end
  bot_lcmgl_color3f(lcmgl,.7,.7,.7); 
  
  bot_lcmgl_switch_buffer(lcmgl);
  pause;
end
