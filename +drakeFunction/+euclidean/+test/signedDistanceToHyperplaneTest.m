function [x,F,info] = signedDistanceToHyperplaneTest(N)
  import drakeFunction.*
  if nargin < 1, N = 3; end
  lcmgl = LCMGLClient('normSquaredTest');
  r_inds = reshape(1:3*N,3,N);
  R3 = drakeFunction.frames.R(3);
  radius = 0.2;
  origin = [1;0;0];
  distance_from_origin_expr = drakeFunction.euclidean.NormSquared(R3);
  squared_dist_between_pts_expr = ...
    compose(drakeFunction.euclidean.NormSquared(R3), ...
            drakeFunction.Identity(R3)-drakeFunction.Identity(R3));
  hyperplane_distance1 = ...
    drakeFunction.euclidean.SignedDistanceToHyperplane(Point(R3,origin), ...
                                                    Point(R3,[1;1;1]));
  hyperplane_distance2 = ...
    drakeFunction.euclidean.SignedDistanceToHyperplane(Point(R3,origin), ...
                                                    Point(R3,[0;0;-1]));

  tmp = repmat({hyperplane_distance1},1,N);
  hyperplane_distance1_N = Concatenated(tmp);
  tmp = repmat({hyperplane_distance2},1,N);
  hyperplane_distance2_N = Concatenated(tmp);
  %hyperplane_distance2_N = hyperplane_distance2_N.setInputFrame(hyperplane_distance1_N.getInputFrame());

  distance_cost = DrakeFunctionConstraint(0,1,distance_from_origin_expr);
  distance_constraint = DrakeFunctionConstraint((2*radius)^2,inf,squared_dist_between_pts_expr);
  hyperplane_constraint = DrakeFunctionConstraint(zeros(2*N,1),Inf(2*N,1),Concatenated({hyperplane_distance1_N,hyperplane_distance2_N},true));

  prog = NonlinearProgramWConstraintObjects(3*N);
  prog = prog.addDisplayFunction(@(r)displayCallback(lcmgl,radius,origin,r),r_inds);
  prog = prog.addConstraint(hyperplane_constraint,r_inds);
  for i = 1:N
    for j = i+1:N
      prog = prog.addConstraint(distance_constraint,r_inds(:,[i,j]));
    end
    prog = prog.addCost(distance_cost,r_inds(:,i));
  end

  [x,F,info] = solve(prog,0.1*rand(3*N,1));
end

function displayCallback(lcmgl,radius,origin,r)
  r = reshape(r,3,[]);
  lcmgl.glDrawAxes();
  lcmgl.glColor4f(1,0,0,1);
  lcmgl.sphere(origin,0.01,20,20);
  lcmgl.glColor4f(0,0,1,0.5);
  for pt = r
    lcmgl.sphere(pt,radius,20,20);
  end
  lcmgl.switchBuffers();
end

