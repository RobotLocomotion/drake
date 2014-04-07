function collisionDetectTest(urdf_filename,varargin)
  %NOTEST

checkDependency('bullet');
checkDependency('lcmgl');

if (nargin > 1)
  typecheck(varargin{1},'double');
  draw_pause = varargin{1};
else
  draw_pause = 0.05;
end

if nargin > 2
  throw_error = varargin{2};
else
  throw_error = true;
end
    
options.floating = true;
options.terrain = [];
tol = 1e-6;

r = RigidBodyManipulator([],options);
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'bullet_collision_closest_points_test');

for i=1:2
  r = addRobotFromURDF(r,urdf_filename,zeros(3,1),zeros(3,1),options);
end

v = r.constructVisualizer();

q = zeros(getNumDOF(r),1);
q(1:3) = q(1:3) + tol*(rand(3,1)-0.5);

kinsol = doKinematics(r,q);
pts = r.body(2).contact_shapes{1}.getPoints();
bnd.xmin=min(pts(1,:));
bnd.xmax=max(pts(1,:));
bnd.ymin=min(pts(2,:));
bnd.ymax=max(pts(2,:));
bnd.zmin=min(pts(3,:));
bnd.zmax=max(pts(3,:));
%q(2) = bnd.ymin;
%q(3) = bnd.zmax;
for x=linspace(-2*(bnd.xmax-bnd.xmin),2*(bnd.xmax-bnd.xmin),50);
  q(1)=x;
  
  kinsol = doKinematics(r,q);
  
  [phi,normal,xA,xB,idxA,idxB] = collisionDetect(r,kinsol);
  sizecheck(xA,[3,1]);

  dist_ref = max(abs(x) - (bnd.xmax-bnd.xmin),bnd.ymin-bnd.ymax);
  debugLCMGL(r,v,kinsol,phi,normal,xA,xB,idxA,idxB,dist_ref,tol);

  if throw_error
    valuecheck(phi, dist_ref,tol);
  end
end

q = zeros(getNumDOF(r),1);
q(1:3) = q(1:3) + tol*(rand(3,1)-0.5);
for y=linspace(-2*(bnd.ymax-bnd.ymin),2*(bnd.ymax-bnd.ymin),50);
  q(2)=y;
  
  kinsol = doKinematics(r,q);
  
  [phi,normal,xA,xB,idxA,idxB] = collisionDetect(r,kinsol);

  dist_ref = max(abs(y) - (bnd.ymax-bnd.ymin),bnd.zmin-bnd.zmax);
  debugLCMGL(r,v,kinsol,phi,normal,xA,xB,idxA,idxB,dist_ref,tol);

  if throw_error
    valuecheck(phi, dist_ref,tol);
  end
end

q = zeros(getNumDOF(r),1);
q(1:3) = q(1:3) + tol*(rand(3,1)-0.5);
for z=linspace(-2*(bnd.zmax-bnd.zmin),2*(bnd.zmax-bnd.zmin),50);
  q(3)=z;
  
  kinsol = doKinematics(r,q);
  
  [phi,normal,xA,xB,idxA,idxB] = collisionDetect(r,kinsol);

  dist_ref = max(abs(z) - (bnd.zmax-bnd.zmin),bnd.ymin-bnd.ymax);
  debugLCMGL(r,v,kinsol,phi,normal,xA,xB,idxA,idxB,dist_ref,tol);

  if throw_error
    valuecheck(phi, dist_ref,tol);
  end
end

function debugLCMGL(r,v,kinsol,phi,normal,xA,xB,idxA,idxB,dist_ref,tol)

    xA_in_world = forwardKin(r,kinsol,idxA,xA);
    xB_in_world = forwardKin(r,kinsol,idxB,xB);
  
    v.draw(0,[q;0*q]);

    lcmgl.glColor3f(1,0,0); % red
    lcmgl.sphere(xA_in_world,.02,20,20);

    lcmgl.glColor3f(0,1,0); % green
    lcmgl.sphere(xB_in_world,.02,20,20);

    lcmgl.glColor3f(0,0,0); % black
    lcmgl.glLineWidth(5); % black
    lcmgl.line3(xB_in_world(1),xB_in_world(2),xB_in_world(3), ...
                xB_in_world(1)+normal(1),xB_in_world(2)+normal(2), ...
                xB_in_world(3)+normal(3));

    dist = norm(xB_in_world-xA_in_world);
    if (abs(dist_ref-phi)<tol) && (abs(abs(phi)-dist)<tol)  
      lcmgl.glColor3f(0,0,0); % black
    else
      lcmgl.glColor3f(1,0,0); % red
    end
    lcmgl.text((xB_in_world+xA_in_world)/2+bnd.zmax*[0;0;1],num2str(phi),0,0);
    
    lcmgl.glColor3f(.7,.7,.7); % gray
    
    lcmgl.switchBuffers();

    pause(draw_pause);
end
end %bullet_collision_closest_distance_jac_test

