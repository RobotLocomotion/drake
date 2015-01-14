function safe_regions = findSafeTerrain(terrain, initial_pose, navgoal, varargin)

p = inputParser();
p.addRequired('terrain', @(x) typecheck(x,'RigidBodyTerrain'));
p.addRequired('initial_pose', @(x) sizecheck(x, 6));
p.addRequired('navgoal', @(x) sizecheck(x, 6));
p.addParameter('resolution', 0.05, @isnumeric);
p.addParameter('forward_dist', 3, @isnumeric);
p.addParameter('width', 2, @isnumeric);
p.addParameter('backward_dist', 1, @isnumeric);
p.addParameter('max_slope_angle', 40 * pi/180, @isnumeric);
p.addParameter('plane_dist_tol', 0.05, @isnumeric);
p.addParameter('normal_angle_tol', 10 * pi/180, @isnumeric);
p.parse(terrain, initial_pose, navgoal, varargin{:});
options = p.Results;

checkDependency('mosek');
checkDependency('iris');

FOOT_LENGTH = 0.2;

planar_regions = iris.TerrainRegion.empty();
region_obstacles = iris.Polytope.empty();

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

X = reshape(X, s);
Y = reshape(Y, s);
Z = reshape(Z, s);
slope_angle = reshape(slope_angle, s);
potential_safe_grid = slope_angle < options.max_slope_angle;

lcmgl = LCMGLClient('planar_region');
while true
%   
%   figure(3)
%   clf
%   imshow(potential_safe_grid, 'InitialMagnification', 'fit');
  

  obs_dists = iris.terrain_grid.obs_dist(potential_safe_grid);

  [max_dist, i0] = max(obs_dists(:));
  if max_dist < 0.5 * FOOT_LENGTH / options.resolution
    break;
  end
  x0 = X(i0);
  y0 = Y(i0);
  z0 = Z(i0);
  n0 = normal(:,i0);

  dist_to_plane = abs(n0'*[reshape(X, 1, []); reshape(Y, 1, []); reshape(Z, 1, [])] - n0'*[x0;y0;z0]);
  dist_to_plane = reshape(dist_to_plane, s);
  dist_mask = abs(dist_to_plane) < options.plane_dist_tol;


  normal_product = n0' * normal;
  normal_product = reshape(normal_product, s);
%   normal_product = medfilt2(normal_product, [3,3], 'symmetric');
%   normal_product = imfilter(normal_product, ones(3), 'symmetric');
  normal_angle_mask = normal_product > cos(options.normal_angle_tol);
  
  in_existing_regions_mask = false(1, numel(X));
  for j = 1:length(region_obstacles)
    in_existing_regions_mask = in_existing_regions_mask | all(bsxfun(@le, region_obstacles(j).A(:,1:2) * [reshape(X, 1, []); reshape(Y, 1, [])], region_obstacles(j).b), 1);
  end
  in_existing_regions_mask = reshape(in_existing_regions_mask, s);
  in_existing_regions_mask = ~in_existing_regions_mask;

  plane_mask = dist_mask & normal_angle_mask & in_existing_regions_mask;
  
  if ~plane_mask(i0)
    break
  end

  boundary_mask = logical(iris.terrain_grid.component_boundary(plane_mask, i0));
  
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
  
  obs_x = X(boundary_mask);
  obs_y = Y(boundary_mask);
  obstacle_pts = reshape([reshape(obs_x, 1, []); reshape(obs_y, 1, [])], 2, 1, []);
%   obstacle_pts = bsxfun(@plus, obstacle_pts, 0.5 * options.resolution * [-1, 1, 1, -1; -1, -1, 1, 1]);
  obstacle_pts = bsxfun(@plus, obstacle_pts, 0.25 * options.resolution * [-1, 1, 1, -1; -1, -1, 1, 1]);

  [A_bounds, b_bounds] = poly2lincon([X(end,1), X(end,end), X(1,end), X(1,1)], [Y(end,1), Y(end,end), Y(1,end), Y(1,1)]);
  [A, b, C, d, results] = iris.inflate_region(obstacle_pts, A_bounds, b_bounds, [x0; y0], struct('require_containment', true));
  
  p = iris.TerrainRegion([A, zeros(size(A, 1), 1)], b, C, d, [x0;y0;z0], n0);
  planar_regions(end+1) = p;
  poly = p.getXYZPolytope().normalize();
  poly.b = poly.b - FOOT_LENGTH;
  region_obstacles(end+1) = poly;
  
  lcmgl.glColor3f(.2,.2,.9);
  planar_regions(end).getXYZPolytope().drawLCMGL(lcmgl);
  lcmgl.switchBuffers();
  
  
  inpoly = all(bsxfun(@minus, A * [reshape(X, 1, []); reshape(Y, 1, [])], b) <= options.resolution / 2, 1);
  inpoly = reshape(inpoly, s);
  potential_safe_grid = potential_safe_grid & ~inpoly;
end




safe_regions = iris.TerrainRegion.empty();
for j = 1:length(planar_regions)
  lcmgl.glColor3f(.2,.2,.9);
  planar_regions(j).getXYZPolytope().drawLCMGL(lcmgl);
  lcmgl.switchBuffers();
  
  safe_region = bodyCSpaceRegion(planar_regions(j), initial_pose(6), X, Y, Z);
  if ~isempty(safe_region)
    safe_regions(end+1) = safe_region;
  end
  
end
lcmgl.switchBuffers();

disp('done')
