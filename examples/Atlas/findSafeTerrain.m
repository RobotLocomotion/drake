function safe_regions = findSafeTerrain(terrain, initial_pose, navgoal, varargin)

p = inputParser();
p.addRequired('terrain', @(x) typecheck(x,'RigidBodyTerrain'));
p.addRequired('initial_pose', @(x) sizecheck(x, 6));
p.addRequired('navgoal', @(x) sizecheck(x, 6));
p.addParameter('resolution', 0.05, @isnumeric);
p.addParameter('forward_dist', 3, @isnumeric);
p.addParameter('width', 2, @isnumeric);
p.addParameter('backward_dist', 0.5, @isnumeric);
p.addParameter('max_slope_angle', 40 * pi/180, @isnumeric);
p.addParameter('plane_dist_tol', 0.05, @isnumeric);
p.addParameter('normal_angle_tol', 10 * pi/180, @isnumeric);
p.parse(terrain, initial_pose, navgoal, varargin{:});
options = p.Results;

foot_shape = [-0.13, -0.13, 0.13, 0.13;
              0.0562, -0.0562, 0.0562, -0.0562];
collision_boxes = struct('z', [0, 0.05, 0.35, 0.75, 1.15],...
  'boxes', {[],...
     [-0.13, -0.13, 0.13, 0.13; 0.0562, -0.0562, 0.0562, -0.0562],...
     [-0.13, -0.13, 0.25, 0.25; .25, -.25, .25, -.25],...
     [-0.2, -0.2, 0.25, 0.25; .4, -.4, .4, -.4],...
     [-0.35, -0.35, 0.25, 0.25; .4, -.4, .4, -.4]});

checkDependency('mosek');
checkDependency('iris');

dist_to_goal = norm(navgoal(1:2) - initial_pose(1:2));

xprime = -options.backward_dist:options.resolution:min(dist_to_goal, options.forward_dist);
yprime = -options.width/2:options.resolution:options.width/2;

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

figure(1)
clf
hold on
% quiver3(X, Y, Z, normal(1,:), normal(2,:), normal(3,:));

X = reshape(X, s);
Y = reshape(Y, s);
Z = reshape(Z, s);
slope_angle = reshape(slope_angle, s);
% mesh(X, Y, Z);
contourf(X, Y, slope_angle);
axis equal

figure(2)
clf
potential_safe_grid = slope_angle < options.max_slope_angle;

while true
  imshow(potential_safe_grid, 'InitialMagnification', 'fit');

  obs_dists = iris.terrain_grid.obs_dist(potential_safe_grid);

  figure(3)
  clf
  mesh(X, Y, obs_dists);

  [~, i0] = max(obs_dists(:));
  x0 = X(i0);
  y0 = Y(i0);
  z0 = Z(i0);
  n0 = normal(:,i0);

  dist_to_plane = abs(n0'*[reshape(X, 1, []); reshape(Y, 1, []); reshape(Z, 1, [])] - n0'*[x0;y0;z0]);
  dist_to_plane = reshape(dist_to_plane, s);
  dist_mask = abs(dist_to_plane) < options.plane_dist_tol;

  figure(4)
  clf
  mesh(X, Y, dist_to_plane);

  normal_product = n0' * normal;
  normal_product = reshape(normal_product, s);
  normal_product = medfilt2(normal_product, [5, 5], 'symmetric');
  normal_angle_mask = normal_product > cos(options.normal_angle_tol);

  figure(5)
  clf
  subplot(211)
  imshow(dist_mask, 'InitialMagnification', 'fit');
  subplot(212)
  imshow(normal_angle_mask, 'InitialMagnification', 'fit');

  plane_mask = dist_mask & normal_angle_mask;

  boundary_mask = logical(iris.terrain_grid.component_boundary(plane_mask, i0));
  figure(7)
  imshow(boundary_mask, 'InitialMagnification', 'fit');

  figure(6)
  clf
  hold on
  obs_x = X(boundary_mask);
  obs_y = Y(boundary_mask);
  obstacle_pts = reshape([reshape(obs_x, 1, []); reshape(obs_y, 1, [])], 2, 1, []);
  obstacle_pts = bsxfun(@plus, obstacle_pts, 0.5 * options.resolution * [-1, 1, 1, -1; -1, -1, 1, 1]);
  size(obstacle_pts)

  [A_bounds, b_bounds] = poly2lincon([X(end,1), X(end,end), X(1,end), X(1,1)], [Y(end,1), Y(end,end), Y(1,end), Y(1,1)]);
  [A_planar, b_planar, ~, ~, results] = iris.inflate_region(obstacle_pts, A_bounds, b_bounds, [x0; y0]);
  iris.drawing.animate_results(results);
  V = iris.thirdParty.polytopes.lcon2vert(A_planar, b_planar)';
  k = convhull(V(1,:), V(2,:));

  plot(V(1,k), V(2,k), 'r.-');
  lcmgl = LCMGLClient('planar_region');
  lcmgl.glBegin(lcmgl.LCMGL_LINES);
  for j = 1:length(k)-1
    lcmgl.glVertex3d(V(1,k(j)), V(2,k(j)), 0.2);
    lcmgl.glVertex3d(V(1,k(j+1)), V(2,k(j+1)), 0.2);
  end
  lcmgl.glEnd();
  lcmgl.switchBuffers();

  inpoly = all(bsxfun(@minus, A_planar * [reshape(X, 1, []); reshape(Y, 1, [])], b_planar) <= 0, 1);
  inpoly = reshape(inpoly, s);
  potential_safe_grid = potential_safe_grid & ~inpoly;
end

bounds.x = [min(min(X(plane_mask))), max(max(X(plane_mask)))];
bounds.y = [min(min(Y(plane_mask))), max(max(Y(plane_mask)))];

lcmgl = LCMGLClient('terrain_segmentation');
lcmgl.sphere([bounds.x(1), bounds.y(1), 0.2], 0.02, 20, 20);
lcmgl.sphere([bounds.x(2), bounds.y(2), 0.2], 0.02, 20, 20);
lcmgl.sphere([bounds.x(1), bounds.y(2), 0.2], 0.02, 20, 20);
lcmgl.sphere([bounds.x(2), bounds.y(1), 0.2], 0.02, 20, 20);
lcmgl.switchBuffers();

% 
% obstacles = iris.terrain_grid.segment_grid(~plane_mask);
% for j = 1:length(obstacles)
%   plot(obstacles{j}(1,:), obstacles{j}(2,:), 'r.-');
% end






safe_regions = [];
