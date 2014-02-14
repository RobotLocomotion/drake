function testClosestDistance()
% NOTEST
% Check distance from a box to a box
xyz = [2;0;0];
rpy = [0.05;0.05;0.06];
robot = RigidBodyManipulator('brick1.urdf',struct('floating',true));
robot = addRobotFromURDF(robot,'brick2.urdf',[2;0;0],[0.05;0.05;0.05],struct('floating',true));
q1 = zeros(6,1);
q2 = [xyz;rpy];
q = [q1;q2];
kinsol = robot.doKinematics(q);
[ptsA,ptsB,normal,dist,idxA,idxB,JA,JB,Jd,vertsA,vertsB] = collisionmex(robot.getMexModelPtr,5);
Jd_numeric = numerical_gradient(robot,q,1e-4);
valuecheck(Jd,Jd_numeric,1e-2);
v = robot.constructVisualizer();
v.draw(0,[q;0*q]);

% Check distance from a sphere to a sphere
xyz = [2;3;4];
rpy = randn(3,1);
robot = RigidBodyManipulator('ball.urdf',struct('floating',true));
robot = robot.addRobotFromURDF('ball.urdf',xyz,rpy,struct('floating',true));
q = [zeros(6,1);xyz;rpy];
kinsol = robot.doKinematics(q);
[ptsA,ptsB,normal,dist,idxA,idxB,JA,JB,Jd,vertsA,vertsB] = collisionmex(robot.getMexModelPtr,5);

% Check the atlas case  
robot = RigidBodyManipulator([getDrakePath,'/systems/plants/constraint/test/model_simple_visuals.urdf'],struct('floating',true));
nomdata = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
nq = robot.getNumDOF();
q = nomdata.xstar(1:nq)+1e-2*randn(nq,1);
kinsol = robot.doKinematics(q);
[ptsA,ptsB,normal,dist,idxA,idxB,JA,JB,Jd,vertsA,vertsB] = collisionmex(robot.getMexModelPtr,5);


% check the edge-surface case 
v = robot.constructVisualizer();
v.draw(0,[q;0*q]);
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,'atlas');
num_dist = length(dist);

for i = 1:num_dist
  type = 0; % 0 for normal, 1 for edge-surface
  if(size(vertsA{i},2) == 2 && (size(vertsB{i},2) == 3 || size(vertsB{i},2) == 4))
    pts_fewer = ptsA(:,i);
    pts_more = ptsB(:,i);
    verts_fewer = vertsA{i};
    verts_more = vertsB{i};
    idx_more = idxB(i);
    idx_fewer = idxA(i);
    type = 1;
  elseif(size(vertsB{i},2) == 2 && (size(vertsA{i},2) == 3 || size(vertsA{i},2) == 4))
    pts_fewer = ptsB(:,i);
    pts_more = ptsA(:,i);
    verts_fewer = vertsB{i};
    verts_more = vertsA{i};
    idx_more = idxA(i);
    idx_fewer = idxB(i);
    type = 1;
  end
  if(type == 1)
    if(~isCoPlane(verts_more))
      warning('surface vertices not on the same plane');
    end
    lcmgl.glColor3f(1,0,0);% red
    lcmgl.text(pts_fewer,'Bullet point fewer',0,0);
    lcmgl.text(pts_more,'Bullet point more',0,0);
    lcmgl.sphere(pts_fewer,0.01,20,20);
    lcmgl.sphere(pts_more,0.01,20,20);
    lcmgl.glColor3f(0.7,0.7,0.7); % gray
    for j = 1:size(verts_fewer,2)
      lcmgl.glColor3f(0,1,0);
      lcmgl.text(verts_fewer(:,j),'edge',0,0);
      lcmgl.sphere(verts_fewer(:,j),0.01,20,20);
    end
    for j = 1:size(verts_more,2)
      lcmgl.glColor3f(0,1,0);
      lcmgl.text(verts_more(:,j),'surface',0,0);
      lcmgl.sphere(verts_more(:,j),0.01,20,20);
    end
    lcmgl.glColor3f(0.7,0.7,0.7);
    lcmgl.switchBuffers();
    figure
    plot3(verts_fewer(1,:),verts_fewer(2,:),verts_fewer(3,:));
    hold on
    fill3(verts_more(1,:),verts_more(2,:),verts_more(3,:),'b');
    plot3(pts_fewer(1),pts_fewer(2),pts_fewer(3),'r.');
    plot3(pts_more(1),pts_more(2),pts_more(3),'r.');
    hold off;
  end
end

end

function dist = collision_userfun(robot,q)
robot.doKinematics(q);
[~,~,~,dist] = collisionmex(robot.getMexModelPtr,5);
end

function gradient = numerical_gradient(robot,q,dq)
nq = robot.getNumDOF();
dist = collision_userfun(robot,q);
gradient = zeros(length(dist),nq);
for i = 1:nq
  q_err = zeros(nq,1);
  q_err(i) = dq;
  dist2 = collision_userfun(robot,q+q_err);
  gradient(:,i) = (dist2-dist)/dq;
end
end

function flag = isCoPlane(verts)
d = diff(verts,1,2);
d = d./bsxfun(@times,sqrt(sum(d.*d,1)),ones(3,1));
normal = cross(d(:,2),d(:,1));
normal = normal/norm(normal);
if(any(abs(normal'*d)>1e-3))
  flag = false;
else
  flag = true;
end
end

function d = pointLineDist(y11,y21,y22)
a = y11-y21;
b = y22-y21;
c = y22-y11;
s = (a+b+c)/2;
d = sqrt(s*(s-a)*(s-b)*(s-c))*2/b;
end

function [d,x1,x2] = lineLineDist(y11,y12,y21,y22)
% d is the distance
% x1 is in line adjoining y11 and y12
% x2 is in line adjoining y21 and y22
% dist(x2,x1) = d;
a = y12-y11;
b = y22-y21;
c = y21-y11;
d = abs(c'*cross(a,b))/norm(cross(a,b));
st = -([a';b']*[a b])\[a b]'*c;
x1 = y11+a*st(1);
x2 = y21+b*st(2);
end

function [d,x] = pointPlaneDist(y11,y21,y22,y23)
% x is on the plane of y21,y22,y23
% dist(y11,x) = d;
a = y22-y21;
b = y23-y21;
v = cross(a,b);
n = v/norm(v);
d = abs(n'*(y11-y21));
k = -n'*y21;
x = [a';b';v']\[[a';b']*y21;k];
end

function [x1,x2,w1,w2,d] = polygon2polygonDist(verts1,verts2)
n_pts1 = size(verts1,2);
n_pts2 = size(verts2,2);
H = [verts1 -verts2]'*[verts1 -verts2];
Aeq = [ones(1,n_pts1) zeros(1,n_pts2);zeros(1,n_pts2) ones(1,n_pts1)];
beq = [1;1];
lb = zeros(n_pts1+n_pts2,1);
ub = ones(n_pts1+n_pts2,1);
[w,fval,exitflag] = quadprog(H,[],[],[],Aeq,beq,lb,ub);
w1 = w(1:n_pts1);
w2 = w(n_pts1+(1:n_pts2));
x1 = verts1*w1;
x2 = verts2*w2;
d = sqrt(2*fval);
end