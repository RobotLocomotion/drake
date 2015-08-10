function [x,F,info] = signedDistanceToHyperplaneTest(N)
  import drakeFunction.*
  if nargin < 1, N = 3; end

  % Setup visualization
  lcmgl = LCMGLClient('normSquaredTest');

  % Convenient constants
  dim = 3;
  r_inds = reshape(1:3*N,3,N);
  radius = 0.2;
  origin = [1;0;0];

  % Create functions for norm squared and squared distance
  norm_squared_fcn = drakeFunction.euclidean.NormSquared(dim);
  squared_dist_between_pts_fcn = norm_squared_fcn(Difference(dim));

  % Create expressions for signed distances to two planes
  hyperplane_distance1 = ...
    drakeFunction.euclidean.SignedDistanceToHyperplane(origin, [1;1;1]);
  hyperplane_distance2 = ...
    drakeFunction.euclidean.SignedDistanceToHyperplane(origin, [0;0;-1]);

  % Duplicate these for N points and concatenate with shared inputs
  hyperplane_distances = Concatenated({duplicate(hyperplane_distance1,N), ...
                                       duplicate(hyperplane_distance2,N)}, ...
                                      true);

  % Create constraints 
  distance_cost = DrakeFunctionConstraint(0,1,norm_squared_fcn);
  distance_constraint = DrakeFunctionConstraint((2*radius)^2,Inf,squared_dist_between_pts_fcn);
  hyperplane_constraint = DrakeFunctionConstraint(zeros(2*N,1),Inf(2*N,1),hyperplane_distances);

  % Create nonlinear program
  prog = NonlinearProgram(3*N);
  prog = prog.addDisplayFunction(@(r)displayCallback(lcmgl,radius,origin,r),r_inds);

  % Add constraints
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

