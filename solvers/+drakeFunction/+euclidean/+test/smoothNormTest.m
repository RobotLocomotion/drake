function [x,F,info] = smoothNormTest(N)
  import drakeFunction.*
  if nargin < 1, N = 3; end
  lcmgl = LCMGLClient('smoothNormTest');
  r_inds = reshape(1:3*N,3,N);
  dim = 3;
  radius = 0.2;
  smoothing_factor = 1e-4;
  smooth_norm_fcn = drakeFunction.euclidean.SmoothNorm(dim,smoothing_factor);
  smooth_dist_between_pts_fcn = smooth_norm_fcn(Difference(dim));
  distance_constraint = DrakeFunctionConstraint((2*radius),inf,smooth_dist_between_pts_fcn);

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
