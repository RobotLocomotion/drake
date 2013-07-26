function bullet_collision_closest_distance_test(varargin)

checkDependency('bullet');

if (nargin > 0)
  typecheck(varargin{1},'double');
  draw_pause = varargin{1};
else
  draw_pause = 0.05;
end
    
r = RigidBodyManipulator();
lcmgl = BotLCMGLClient('bullet_collision_closest_points_test');

for i=1:2
  r = addRobotFromURDF(r,'FallingBrick.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
end

v = r.constructVisualizer();

q = zeros(getNumDOF(r),1);

kinsol = doKinematics(r,q);
pts = contactPositions(r,kinsol);
bnd.xmin=min(pts(1,:));
bnd.xmax=max(pts(1,:));
bnd.ymin=min(pts(2,:));
bnd.ymax=max(pts(2,:));
bnd.zmin=min(pts(3,:));
bnd.zmax=max(pts(3,:));
tol = 1e-6;

for x=linspace(-2*(bnd.xmax-bnd.xmin),-(bnd.xmax-bnd.xmin),50);
  q(1)=x;
  
  kinsol = doKinematics(r,q);
  
  [ptA,ptB,normal,JA,JB,dJA,dJB] = pairwiseClosestPoints(r,kinsol,2,3);

  dist_ref = abs(abs(x) - (bnd.xmax-bnd.xmin));
  debugLCMGL(r,v,kinsol,ptA,ptB,dist_ref,tol);

  valuecheck(norm(ptB-ptA), dist_ref,tol);
end

q = zeros(getNumDOF(r),1);
for y=linspace(-2*(bnd.ymax-bnd.ymin),-(bnd.ymax-bnd.ymin),50);
  q(2)=y;
  
  kinsol = doKinematics(r,q);
  
  [ptA,ptB,normal,JA,JB,dJA,dJB] = pairwiseClosestPoints(r,kinsol,2,3);

  dist_ref = abs(abs(y) - (bnd.ymax-bnd.ymin));
  debugLCMGL(r,v,kinsol,ptA,ptB,dist_ref,tol);

  valuecheck(norm(ptB-ptA), dist_ref,tol);
end

q = zeros(getNumDOF(r),1);
for z=linspace(-2*(bnd.zmax-bnd.zmin),-(bnd.zmax-bnd.zmin),50);
  q(3)=z;
  
  kinsol = doKinematics(r,q);
  
  [ptA,ptB,normal,JA,JB,dJA,dJB] = pairwiseClosestPoints(r,kinsol,2,3);

  dist_ref = abs(abs(z) - (bnd.zmax-bnd.zmin));
  debugLCMGL(r,v,kinsol,ptA,ptB,dist_ref,tol);

  valuecheck(norm(ptB-ptA), dist_ref,tol);
end

function debugLCMGL(r,v,kinsol,ptA,ptB,dist_ref,tol)
  
    v.draw(0,[q;0*q]);

    bot_lcmgl_color3f(lcmgl,1,0,0); % red
    bot_lcmgl_sphere(lcmgl,ptA,.05,20,20);

    bot_lcmgl_color3f(lcmgl,0,1,0); % green
    bot_lcmgl_sphere(lcmgl,ptB,.05,20,20);

    dist = norm(ptB-ptA);
    if (abs(dist_ref-dist)<tol)  
      bot_lcmgl_color3f(lcmgl,0,0,0); % black
    else
      bot_lcmgl_color3f(lcmgl,1,0,0); % red
    end
    bot_lcmgl_text_ex(lcmgl,(ptB+ptA)/2+bnd.zmax*[0;0;1],num2str(dist),0,0);
    
    bot_lcmgl_color3f(lcmgl,.7,.7,.7); % gray
    
    bot_lcmgl_switch_buffer(lcmgl);

    pause(draw_pause);
end
end %bullet_collision_closest_distance_jac_test
