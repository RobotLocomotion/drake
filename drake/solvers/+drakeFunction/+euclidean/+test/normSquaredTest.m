function [x,F,info] = normSquaredTest(N)
  import drakeFunction.*
  if nargin < 1, N = 3; end
  lcmgl = LCMGLClient('normSquaredTest');
  r_inds = reshape(1:3*N,3,N);
  radius = 0.2;
  dim = 3;

  % Test Non-positive-defintite Q detection
  try
    drakeFunction.euclidean.NormSquared(dim, zeros(dim));
  catch ex
    valuecheck(ex.identifier,'Drake:drakeFunction:euclidean:NormSquared:NonPositiveDefiniteQ');
  end

  norm_squared_fcn = drakeFunction.euclidean.NormSquared(3);
  squared_dist_between_pts_fcn = norm_squared_fcn(Difference(3));
  distance_constraint = DrakeFunctionConstraint((2*radius)^2,inf,squared_dist_between_pts_fcn);

  prog = NonlinearProgram(3*N);
  prog = prog.addDisplayFunction(@(r)displayCallback(lcmgl,radius,r),r_inds);
  for i = 1:N
    for j = i+1:N
      prog = prog.addCost(distance_constraint,r_inds(:,[i,j]));
      prog = prog.addConstraint(distance_constraint,r_inds(:,[i,j]));
    end
    %prog = prog.addCost(distance_cost,r_inds(:,i));
  end

  [x,F,info] = solve(prog,0.1*rand(3*N,1));
end

function displayCallback(lcmgl,radius,r)
  r = reshape(r,3,[]);
  lcmgl.glColor4f(0,0,1,0.5);
  for pt = r
    lcmgl.sphere(pt,radius,20,20);
  end
  lcmgl.switchBuffers();
end
