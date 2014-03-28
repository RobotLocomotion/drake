function testCollisionGrad
options.floating = true;
r = RigidBodyManipulator([getenv('DRC_BASE'),'/software/models/mit_gazebo_models/mit_robot_drake/model.urdf'],options);
%r = RigidBodyManipulator([getDrakePath(), '/examples/Pendulum/test/PendulumWithFriction.urdf']);
%r = RigidBodyManipulator();
%r = r.addRobotFromURDF('FallingBrick.urdf',[],[],options);
%r = r.addRobotFromURDF('FallingBrick.urdf',[],[],options);
v = r.constructVisualizer();
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'testCollisionGrad');
nq = getNumPositions(r);
%q0 = 10*rand(nq,1);
x0 = r.getInitialState();
load /home/avalenzu/drc/software/control/matlab/data/atlas_fp.mat
while(true)
  q0 = xstar(1:nq)+5e-1*(2*rand(nq,1)-1);
  q1 = q0;
  q2 = q1; q2(6) = pi/2;
  [f_user,df_user] = geval(@(q)closestDistanceAllBodies(r,q),q0,struct('grad_method','user'));
  [f_num,df_num] = geval(@(q)closestDistanceAllBodies(r,q),q0,struct('grad_method','numerical'));
  max(abs(df_user(:)-df_num(:)))
  %[ptsA,ptsB,~,~,idxA,idxB] = closestPointsAllBodies(r,q0);
  %[ptsA1,ptsB1,~,~,idxA1,idxB1] = closestPointsAllBodies(r,q1);
  %[ptsA2,ptsB2,~,~,idxA2,idxB2] = closestPointsAllBodies(r,q2);
  %da = logspace(-8,0,5e1);
  [ptsA_in_world,ptsB_in_world,ptsA,ptsB,idxA,idxB] = closestPoints(r,q0,v,lcmgl);
  lcmgl.switchBuffers();
  rows_bad = find(any(abs(df_user-df_num)>1e-6,2));
  for i = 1:length(rows_bad);
    fprintf('Gradient mismatch for %s and %s\n',r.getLinkName(idxA(rows_bad(i))),r.getLinkName(idxB(rows_bad(i))));
  end
  keyboard
end
end

% Remove this when the RBM method is implemented
function n_pos = getNumPositions(r)
  n_pos = r.getNumDOF();
end

function [ptsA_in_world, ptsB_in_world, ptsA, ptsB, idxA, idxB] = closestPoints(r,q,v,lcmgl)
  kinsol = doKinematics(r,q);
  [ptsA,ptsB,~,~,idxA,idxB] = closestPointsAllBodies(r,kinsol);
  ptsA_in_world = zeros(size(ptsA));
  ptsB_in_world = zeros(size(ptsB));
  for i = 1:length(idxA)
    ptsA_in_world(:,i) = forwardKin(r,kinsol,idxA(i),ptsA(:,i));
    ptsB_in_world(:,i) = forwardKin(r,kinsol,idxB(i),ptsB(:,i));
  end
  plotPointsLCMGL(lcmgl,ptsA_in_world,[1,0,0])
  plotPointsLCMGL(lcmgl,ptsB_in_world,[0,0,1])
  v.draw(0,[q;0*q]);
end

function df_array = sweepNumGradStepSize(r,da,q0,row_idx,col_idx,diff_type)
  nq = length(q0);
  n_da = length(da);
  df_array = zeros(n_da,nq);
  for i = 1:n_da
    fprintf('da = %6.4e\n',da(i));
    dq = zeros(nq,1);
    dq(col_idx) = da(i);
    [~,df] = geval(@(q)closestDistanceAllBodies(r,q),q0+dq,struct('grad_method','numerical','da',da(i),'diff_type',diff_type));
    df_array(i,:) = df(row_idx,:);
  end
end

function plotPointsLCMGL(lcmgl,pts,c)
  if nargin < 3, c = zeros(1,3); end;
  for pt = pts
    lcmgl.glColor3f(c(1),c(2),c(3));lcmgl.sphere(pt,0.01,20,20);
  end
end
