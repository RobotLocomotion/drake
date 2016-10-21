function collisionDetectGradTest(visualize,n_debris)
  % Setting a fixed seed to avoid stochastic failures
  rng(1);

  checkDependency('lcmgl');
  if nargin < 1
    visualize = false;
  end
  if nargin < 2
    n_debris = 5;
  end
  options.floating = true;
  S = warning('OFF','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('OFF','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  r = RigidBodyManipulator([getDrakePath(),'/examples/Atlas/urdf/atlas_convex_hull.urdf'],options);
  for i = 1:n_debris
    r = r.addRobotFromURDF('FallingBrick.urdf',3*(2*rand(3,1)-1),2*pi*rand(3,1));
    r = r.addRobotFromURDF('ball.urdf',3*(2*rand(3,1)-1),2*pi*rand(3,1));
    r = r.addRobotFromURDF('Capsule.urdf',3*(2*rand(3,1)-1),2*pi*rand(3,1));
  end
  warning(S);

  if visualize
    v = r.constructVisualizer(struct('use_collision_geometry',true));
    lcmgl = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'testCollisionGrad');
  end
  nq = getNumPositions(r);
  load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
  for j = 1:1e1
    q0 = xstar(1:nq)+5e-1*(2*rand(nq,1)-1);
    q0(1:3) = 2*(2*rand(3,1)-1);
    [f_user,df_user] = geval(@(q)contactConstraintsTest(r,q),q0,struct('grad_method','user'));
    [~,df_num] = geval(@(q)contactConstraintsTest(r,q),q0,struct('grad_method','numerical','da',1.1e-8, 'diff_type','central'));
    rows_bad = find(any(abs(df_user-df_num)>1e-6,2));
    if visualize
      v.draw(0,[q0;0*q0]);
      drawClosestPoints(r,q0,v,lcmgl);
      lcmgl.glColor3f(0,0,0);
      %lcmgl.text(zeros(3,1),sprintf('Min. signed distance: %6.4f\nMax.  difference between user and numerical gradients: %6.4e',min(f_user),max(abs(df_user(:)-df_num(:)))),0,0);
      fprintf('Min. signed distance: %6.4f\nMax.  difference between user and numerical gradients: %6.4e\n',min(f_user),max(abs(df_user(:)-df_num(:))));
      lcmgl.switchBuffers();
    end
    if ~isempty(rows_bad)
      [~,~,~,~,idxA,idxB] = collisionDetect(r,q0);
      for i = 1:length(rows_bad);
        fprintf('Gradient mismatch for %s and %s\n',r.getLinkName(idxA(rows_bad(i))),r.getLinkName(idxB(rows_bad(i))));
      end
      error('Numerical and user gradients do not match');
    end
  end
end

% Remove this when the RBM method is implemented
function n_pos = getNumPositions(r)
  n_pos = r.getNumPositions();
end

function [ptsA_in_world, ptsB_in_world, ptsA, ptsB, idxA, idxB] = drawClosestPoints(r,q,v,lcmgl)
  kinsol = doKinematics(r,q);
  [~,~,ptsA,ptsB,idxA,idxB] = collisionDetect(r,kinsol);
  ptsA_in_world = ptsA;
  ptsB_in_world = ptsB;
  for i = 1:length(idxA)
    ptsA_in_world(:,i) = forwardKin(r,kinsol,idxA(i),ptsA(:,i));
    ptsB_in_world(:,i) = forwardKin(r,kinsol,idxB(i),ptsB(:,i));
  end
  plotPointsLCMGL(lcmgl,ptsA_in_world,[1,0,0])
  plotPointsLCMGL(lcmgl,ptsB_in_world,[0,0,1])
  v.draw(0,[q;0*q]);
end

function plotPointsLCMGL(lcmgl,pts,c)
  if nargin < 3, c = zeros(1,3); end;
  for pt = pts
    lcmgl.glColor3f(c(1),c(2),c(3));lcmgl.sphere(pt,0.01,20,20);
  end
end

function [f,df] = contactConstraintsTest(r,q)
  [f,~,~,~,~,~,~,~,df] = contactConstraints(r,q);
end
