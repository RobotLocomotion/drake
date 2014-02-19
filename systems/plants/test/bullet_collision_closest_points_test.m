function bullet_collision_closest_distance_test(varargin)

checkDependency('bullet');
checkDependency('lcmgl');

if (nargin > 0)
  typecheck(varargin{1},'double');
  draw_pause = varargin{1};
else
  draw_pause = 0.05;
end
    
r = RigidBodyManipulator();
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'bullet_collision_closest_points_test');

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

    lcmgl.glColor3f(1,0,0); % red
    lcmgl.sphere(ptA,.05,20,20);

    lcmgl.glColor3f(0,1,0); % green
    lcmgl.sphere(ptB,.05,20,20);

    dist = norm(ptB-ptA);
    if (abs(dist_ref-dist)<tol)  
      lcmgl.glColor3f(0,0,0); % black
    else
      lcmgl.glColor3f(1,0,0); % red
    end
    lcmgl.text((ptB+ptA)/2+bnd.zmax*[0;0;1],num2str(dist),0,0);
    
    lcmgl.glColor3f(.7,.7,.7); % gray
    
    lcmgl.switchBuffers();

    pause(draw_pause);
end
end %bullet_collision_closest_distance_jac_test
