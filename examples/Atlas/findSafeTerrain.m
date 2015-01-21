function safe_regions = findSafeTerrain(terrain, initial_pose, navgoal, varargin)

p = inputParser();
p.addRequired('terrain', @(x) typecheck(x,'RigidBodyTerrain'));
p.addRequired('initial_pose', @(x) sizecheck(x, 6));
p.addRequired('navgoal', @(x) sizecheck(x, 6));
p.addParameter('resolution', 0.025, @isnumeric);
p.addParameter('forward_dist', 3, @isnumeric);
p.addParameter('width', 2, @isnumeric);
p.addParameter('backward_dist', 0.5, @isnumeric);
p.addParameter('max_slope_angle', 40 * pi/180, @isnumeric);
p.addParameter('plane_dist_tol', 0.05, @isnumeric);
p.addParameter('normal_angle_tol', 10 * pi/180, @isnumeric);
p.addParameter('xy_bounds', iris.Polytope(zeros(0,2),zeros(0,1)));
p.addParameter('seeds', []);
p.parse(terrain, initial_pose, navgoal, varargin{:});
options = p.Results;

original_foot_shape = [-0.12, -0.12, 0.13, 0.13;
              0.04, -0.04, 0.04, -0.04];

 
collision_boxes = struct('z', [0, 0.05, 0.35, 0.75, 1.15],...
  'boxes', {{[],...
     [-0.17, -0.17, 0.17, 0.17; 0.07, -0.07, 0.07, -0.07],...
     [-0.17, -0.17, 0.25, 0.25; .25, -.25, .25, -.25],...
     [-0.2, -0.2, 0.25, 0.25; .4, -.4, .4, -.4],...
     [-0.4, -0.4, 0.3, 0.3; .45, -.45, .45, -.45]}});


checkDependency('mosek');
checkDependency('iris');

FOOT_LENGTH = 0.2;

dist_to_goal = norm(navgoal(1:2) - initial_pose(1:2));

xprime = -options.backward_dist:options.resolution:min(dist_to_goal, options.forward_dist);
yprime = -options.width/2:options.resolution:options.width/2;

safe_regions = iris.TerrainRegion.empty();

[Xp, Yp] = meshgrid(xprime, yprime);
s = size(Xp);
Xp = reshape(Xp, 1, []);
Yp = reshape(Yp, 1, []);
XY = bsxfun(@plus, rotmat(atan2(navgoal(2) - initial_pose(2), navgoal(1) - initial_pose(1))) * [Xp; Yp], initial_pose(1:2));
X = XY(1,:);
Y = XY(2,:);

[Z, normal] = terrain.getHeight([X; Y]);
normal = bsxfun(@rdivide, normal, sqrt(sum(normal.^2, 1)));
slope_angle = atan2(sqrt(sum(normal(1:2,:).^2, 1)), normal(3,:));

X = reshape(X, s);
Y = reshape(Y, s);
Z = reshape(Z, s);
slope_angle = reshape(slope_angle, s);
potential_safe_grid = slope_angle < options.max_slope_angle;

within_bounds = all(bsxfun(@le, options.xy_bounds.A * [reshape(X, 1, []); reshape(Y, 1, [])], options.xy_bounds.b), 1);
potential_safe_grid = potential_safe_grid & reshape(within_bounds,s);

for dx = [-1,1]
  for dy = [-1, 1]
    dZ = Z(2:end-1,2:end-1) - Z((2+dx):(end-1+dx),(2+dy):(end-1+dy));
    potential_safe_grid(2:end-1,2:end-1) = potential_safe_grid(2:end-1,2:end-1) & abs(dZ) < 0.05;
  end
end

in_existing_regions_mask = true(s);

lcmgl = LCMGLClient('planar_region');
lcmgl.glColor3f(.2,.2,.9);

figure(5)
clf
hold on

seed_ind = 1;
while true
  
%   figure(3)
%   clf
%   imshow(potential_safe_grid, 'InitialMagnification', 'fit');
  
  if seed_ind <= size(options.seeds, 2)
    start = options.seeds([1,2,6],seed_ind);
    seed_ind = seed_ind + 1;
    [~, m0] = min(abs(Y(:,1) - start(2)));
    [~, n0] = min(abs(X(1,:) - start(1)));
    i0 = sub2ind(s, m0, n0);
  else
    obs_dists = iris.terrain_grid.obs_dist(potential_safe_grid);
    [max_dist, i0] = max(obs_dists(:));
    if max_dist < 0.5 * FOOT_LENGTH / options.resolution
      break;
    end
  end
  
  x0 = X(i0);
  y0 = Y(i0);
  z0 = Z(i0);
  p0 = [x0;y0;z0];
  n0 = normal(:,i0);
  % Reorient the foot onto the terrain plane
  c = cross([0;0;1], n0);
  if norm(c) >= 1e-6
    ax = c / norm(c);
    angle = asin(norm(c));
    Rplane = axis2rotmat([ax; angle]);
  else
    Rplane = eye(3);
  end

  dist_to_plane = abs(n0'*[reshape(X, 1, []); reshape(Y, 1, []); reshape(Z, 1, [])] - n0'*[x0;y0;z0]);
  dist_to_plane = reshape(dist_to_plane, s);
  dist_mask = abs(dist_to_plane) < options.plane_dist_tol;


  normal_product = n0' * normal;
  normal_product = reshape(normal_product, s);
  normal_angle_mask = normal_product > cos(options.normal_angle_tol);
  
%   for j = 1:length(region_obstacles)
%     in_existing_regions_mask = in_existing_regions_mask | all(bsxfun(@le, region_obstacles(j).A(:,1:2) * [reshape(X, 1, []); reshape(Y, 1, [])], region_obstacles(j).b), 1);
%   end
%   in_existing_regions_mask = reshape(in_existing_regions_mask, s);
%   in_existing_regions_mask = ~in_existing_regions_mask;

  plane_mask = dist_mask & normal_angle_mask & in_existing_regions_mask;
  
  if ~plane_mask(i0)
    break
  end

  boundary_mask = logical(iris.terrain_grid.component_boundary(plane_mask, i0));
%   
%   figure(4)
%   clf
%   subplot(321)
%   imshow(dist_mask, 'InitialMagnification', 'fit');
%   title('dist')
%   subplot(323)
%   imshow(normal_angle_mask, 'InitialMagnification', 'fit');
%   title('normal');
%   subplot(325)
%   imshow(in_existing_regions_mask, 'InitialMagnification', 'fit');
%   title('existing');
%   subplot(322)
%   imshow(plane_mask, 'InitialMagnification', 'fit');
%   title('combined');
%   subplot(324)
%   imshow(~boundary_mask, 'InitialMagnification', 'fit');
%   title('boundary');
  
%   figure(6)
%   clf
%   hold on
  
  obs_x = X(boundary_mask);
  obs_y = Y(boundary_mask);
  obstacle_pts = reshape([reshape(obs_x, 1, []); reshape(obs_y, 1, [])], 2, 1, []);
%   plot(obstacle_pts(1,:), obstacle_pts(2,:), 'k.');
  theta_steps = initial_pose(6) + (-pi:pi/4:pi);
  
  c_obs = iris.cspace.cspace3(obstacle_pts, original_foot_shape, theta_steps);
  
    
  % compute distance from the heightmap Z to the current plane
  % n' * [x;y;z] = n' * point
  % z = (n'*point - n(1:2)'*[x;y])/n(3)

  dZ = Z - reshape((n0'*p0 - n0(1:2)'*[reshape(X,1,[]);reshape(Y,1,[])])/n0(3), size(Z));
%   figure(7)
%   clf
%   hold on
%   mesh(X, Y, dZ)
%   plot3(x0, y0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r')
  for j = 1:length(collision_boxes.z)
    if isempty(collision_boxes.boxes{j})
      continue
    end
    zmin = collision_boxes.z(j);
    if j < length(collision_boxes.z)
      zmax = collision_boxes.z(j+1);
    else
      zmax = inf;
    end
    z_range_mask = dZ >= zmin & dZ <= zmax;
    boundary_mask = logical(iris.terrain_grid.component_boundary(~z_range_mask, i0));
%     figure(9)
%     clf
%     subplot 211
%     imshow(z_range_mask, 'InitialMagnification', 'fit');
%     subplot 212
%     imshow(boundary_mask, 'InitialMagnification', 'fit');
    obs_x = X(boundary_mask);
    obs_y = Y(boundary_mask);
    if ~isempty(obs_x)
      obstacle_pts = reshape([reshape(obs_x, 1, []); reshape(obs_y, 1, [])], 2, 1, []);
%       figure(6)
%       plot(obstacle_pts(1,:), obstacle_pts(2,:), 'k.');
      c_obs = cat(3, c_obs, iris.cspace.cspace3(obstacle_pts, collision_boxes.boxes{j}, theta_steps));
    end
  end
  

  [A_bounds, b_bounds] = poly2lincon([X(end,1), X(end,end), X(1,end), X(1,1)], [Y(end,1), Y(end,end), Y(1,end), Y(1,1)]);
  A_bounds = [A_bounds; options.xy_bounds.A];
  b_bounds = [b_bounds; options.xy_bounds.b];
  A_bounds = [A_bounds, zeros(size(A_bounds, 1), 1); zeros(2, size(A_bounds, 2)), [-1;1]];
  b_bounds = [b_bounds; -theta_steps(1); theta_steps(end)];
  try
    [A, b, C, d, results] = iris.inflate_region(c_obs, A_bounds, b_bounds, [x0; y0; initial_pose(6)], struct('require_containment', true, 'error_on_infeas_start', true));
  catch e
    if strcmp(e.identifier, 'IRIS:InfeasibleStart')     
      in_existing_regions_mask(i0) = false;
      potential_safe_grid(i0) = false;
      continue
    else
      rethrow(e)
    end
  end
%   iris.drawing.animate_results(results);

%   figure(11)
%   clf
%   hold on
%   for j = 1:size(c_obs, 3)
%   iris.drawing.drawPolyFromVertices(c_obs(:,:,j), 'k');
%   end
%   iris.drawing.drawPolyFromVertices(iris.thirdParty.polytopes.lcon2vert(A,b)', 'r')

%   figure(12)
%   clf
%   iris.drawing.drawPolyFromVertices(iris.thirdParty.polytopes.lcon2vert(A,b)', 'r')
  
  p = iris.TerrainRegion(A, b, C, d, p0, n0);
  safe_regions(end+1) = p;
  poly = iris.Polytope(A(:,1:2), b).normalize();
%   figure(5)
%   poly.plotVertices('r.-');
  poly.b = poly.b - FOOT_LENGTH;
%   region_obstacles(end+1) = poly;
  in_existing_regions_mask(i0) = false;
  potential_safe_grid(i0) = false;
  
  inpoly = all(bsxfun(@minus, A * [reshape(X, 1, []); reshape(Y, 1, []); initial_pose(6) + zeros(1,numel(X))], b) <= FOOT_LENGTH / 2, 1);
  inpoly = reshape(inpoly, s);
  potential_safe_grid = potential_safe_grid & ~inpoly;
  
%     xs = iris.sample_convex_polytope(safe_regions(end).A, safe_regions(end).b, 100);
%     lcmgl = LCMGLClient('sampled_foot_poses');
%     lcmgl.glColor3f(.2,.9,.2);
%     for j = 1:size(xs, 2)
%       xyyaw = xs(:,j);
%       % n1 * x + n2 * y + n3 * z = n*p
%       % z = (n*p - n1x - n2y) / n3
%       z = 0.05 + (n0' * p0 - n0(1:2)' * xyyaw(1:2)) / n0(3); 
%       foot_shape = bsxfun(@plus, [xyyaw(1:2); z], Rplane * rpy2rotmat([0;0;xyyaw(3)]) * [original_foot_shape; zeros(1, size(original_foot_shape, 2))]);
%       lcmgl.glBegin(lcmgl.LCMGL_LINES);
%       k = convhull(foot_shape(1,:), foot_shape(2,:));
%       for i = 1:length(k)-1
%         lcmgl.glVertex3d(foot_shape(1,k(i)), foot_shape(2,k(i)), foot_shape(3,k(i)));
%         lcmgl.glVertex3d(foot_shape(1,k(i+1)), foot_shape(2,k(i+1)), foot_shape(3,k(i+1)));
%       end
%       lcmgl.glEnd();
%     end 
%     lcmgl.switchBuffers();
%     disp('here');

end
lcmgl.switchBuffers();

disp('done')
