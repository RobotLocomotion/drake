function bullet_collision_test

r = RigidBodyManipulator();

for i=1:2
  r = addRobotFromURDF(r,'FallingBrick.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
end
%v = r.constructVisualizer();

%v = BotVisualizer('FallingBrick.urdf',struct('floating',true));
% lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'contact_points');

%q = randn(getNumPositions(r),1);
q = zeros(getNumPositions(r),1);

kinsol = doKinematics(r,q);
pts = terrainContactPositions(r,kinsol);
bnd.xmin=min(pts(1,:));
bnd.xmax=max(pts(1,:));
bnd.ymin=min(pts(2,:));
bnd.ymax=max(pts(2,:));
bnd.zmin=min(pts(3,:));
bnd.zmax=max(pts(3,:));

for x=linspace(-2*(bnd.xmax-bnd.xmin),0,20);
  q(1)=x;
  
  kinsol = doKinematics(r,q);
  [ptsA,ptsB] = pairwiseContactTest(r,kinsol,2,3);
  
  if (0) debugLCMGL(r,v,kinsol,ptsA,ptsB); end % use botvis
  if (1)  debugPlot(r,kinsol,ptsA,ptsB); end % use matlab plotting
  
  should_have_contact = (bnd.xmax+x >= bnd.xmin);
  assert(should_have_contact==~isempty(ptsA),'collision error');
end
q(1)=0;

for y=linspace(-2*(bnd.ymax-bnd.ymin),0,20);
  q(2)=y;
  
  kinsol = doKinematics(r,q);
  [ptsA,ptsB] = pairwiseContactTest(r,kinsol,2,3);
  
  if (0) debugLCMGL(r,v,kinsol,ptsA,ptsB); end % use botvis
  if (1)  debugPlot(r,kinsol,ptsA,ptsB); end % use matlab plotting
  
  should_have_contact = (bnd.ymax+y >= bnd.ymin);
  assert(should_have_contact==~isempty(ptsA),'collision error');
end
q(2)=0;

for z=linspace(-2*(bnd.zmax-bnd.zmin),0,20);
  q(3)=z;
  
  kinsol = doKinematics(r,q);
  [ptsA,ptsB] = pairwiseContactTest(r,kinsol,2,3);
  
  if (0) debugLCMGL(r,v,kinsol,ptsA,ptsB); end % use botvis
  if (1)  debugPlot(r,kinsol,ptsA,ptsB); end % use matlab plotting
  
  should_have_contact = (bnd.zmax+z >= bnd.zmin);
  assert(should_have_contact==~isempty(ptsA),'collision error');
end
q(3)=0;

q(6)=randn;
for x=linspace(-2*(bnd.xmax-bnd.xmin),0,20);
  q(1)=x;
  kinsol = doKinematics(r,q);
  [ptsA,ptsB] = pairwiseContactTest(r,kinsol,2,3);
  
  if (0) debugLCMGL(r,v,kinsol,ptsA,ptsB); end % use botvis
  if (1)  debugPlot(r,kinsol,ptsA,ptsB); end % use matlab plotting
  
  pause(0.01);
end
q(1)=0;

q(4:6) = randn(3,1);
q(6+(4:6)) = randn(3,1);
for x=linspace(-2*(bnd.xmax-bnd.xmin),0,20);
  q(1)=x;
  kinsol = doKinematics(r,q);
  [ptsA,ptsB] = pairwiseContactTest(r,kinsol,2,3);
  
  if (0) debugLCMGL(r,v,kinsol,ptsA,ptsB); end % use botvis
  if (1)  debugPlot(r,kinsol,ptsA,ptsB); end % use matlab plotting
  
  pause(.1);
end


end

function debugLCMGL(r,v,kinsol,ptsA,ptsB)
    pts = terrainContactPositions(r,kinsol);
  
    v.draw(0,[q;0*q]);
    obj.lcmgl.glColor3f(0,0,1); % blue
    for j=1:size(pts,2)
      obj.lcmgl.sphere(pts(:,j),.05,20,20);
    end
    
    for j=1:size(ptsA,2)
      obj.lcmgl.glColor3f(1,0,0); % red
      obj.lcmgl.sphere(ptsA(:,j),.05,20,20);
      obj.lcmgl.glColor3f(0,1,0); % green
      obj.lcmgl.sphere(ptsB(:,j),.05,20,20);
    end
    
    obj.lcmgl.glColor3f(.7,.7,.7); % gray
    
    obj.lcmgl.switchBuffers();
end

function debugPlot(r,kinsol,ptsA,ptsB)
    pts = terrainContactPositions(r,kinsol);
  
    figure(1); clf; hold on;
    plotBox(pts(:,1:8));
    plotBox(pts(:,9:end));
    plot3(pts(1,:),pts(2,:),pts(3,:),'b.');
    plot3(ptsA(1,:),ptsA(2,:),ptsA(3,:),'r*');
    plot3(ptsB(1,:),ptsB(2,:),ptsB(3,:),'g*');
    axis equal
    drawnow;
end

function plotBox(pts)
  x=pts(1,:); y=pts(2,:); z=pts(3,:);
  ind=[1,2;2,4;4,3;3,1;5,6;6,8;8,7;7,5;1,5;2,6;3,7;4,8]';
  line(x(ind),y(ind),z(ind),'Color','b');
end


% NOTEST