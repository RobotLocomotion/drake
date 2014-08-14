function [x,F,info] = normSquaredTest(N)
  import expression.*
  if nargin < 1, N = 3; end
  lcmgl = LCMGLClient('normSquaredTest');
  r_inds = reshape(1:3*N,3,N);
  R3 = expression.frames.R(3);
  radius = 0.2;
  distance_from_origin_expr = expression.euclidean.NormSquared(R3);
  squared_dist_between_pts_expr = ...
    compose(expression.euclidean.NormSquared(R3), ...
            expression.Identity(R3)-expression.Identity(R3));
  distance_cost = DrakeFunctionConstraint(0,1,distance_from_origin_expr);
  distance_constraint = DrakeFunctionConstraint((2*radius)^2,inf,squared_dist_between_pts_expr);

  prog = NonlinearProgramWConstraintObjects(3*N);
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
