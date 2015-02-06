function slices = collisionSlices(obj, q, slice_heights, varargin)
% Compute bounding boxes of the robot's geometry at a given configuration
% @param q the configuration vector
% @param slice_heights vector of z values (m) providing the limits of the bounding boxes

p = inputParser();
p.addRequired('q');
p.addRequired('slice_heights', @isnumeric);
p.addParamValue('scan_radius', 3, @isscalar);
p.addParamValue('margin', 0.05, @isnumeric);
p.parse(q, slice_heights, varargin{:});
options = p.Results;

if length(options.margin) == 1
  options.margin = repmat(options.margin, 1, length(slice_heights)-1);
end

DEBUG = true;

q(6) = 0; % set yaw to zero

% Remove collision groups for the left leg (which will presumably be
% swinging).
leg_link_names = {'l_foot', 'l_talus', 'l_lleg', 'l_uleg', 'l_lglut', 'l_uglut'};
leg_body_ids = zeros(1, length(leg_link_names));
for j = 1:length(leg_link_names)
  leg_body_ids(j) = obj.findLinkId(leg_link_names{j});
end
obj = obj.removeCollisionGroupsExcept({},1,leg_body_ids);
obj = obj.compile();

scan_center = q(1:2);

kinsol = doKinematics(obj, q);

slices = struct('z', slice_heights(1:end-1),...
                'xy', zeros([2, 4, length(slice_heights)]));
              

ths = linspace(-pi, pi);

if DEBUG
  all_returns = zeros(3, 0);
end
for j = 1:length(slice_heights)-1
  zmin = slice_heights(j);
  zmax = slice_heights(j+1);
  zs = linspace(zmin, zmax, 10);
  slice_returns = zeros(3, length(zs) * length(ths));


  offset = 0;

  for th_ind = 1:length(ths)
    th = ths(th_ind);
    xy = scan_center + options.scan_radius * [cos(th); sin(th)];
    origins = [repmat(xy, 1, length(zs)); zs];
    targets = [repmat(scan_center, 1, length(zs)); zs];
    distances = collisionRaycast(obj, kinsol, origins, targets, false);
    distances(distances <= 0) = nan;

    normals = bsxfun(@rdivide, targets - origins, sqrt(sum((targets - origins).^2, 1)));
    returns = origins + bsxfun(@times, normals, distances');
    slice_returns(:,offset+(1:length(zs))) = returns;
    offset = offset + length(zs);
  end

  slice_returns = slice_returns(:,~any(isnan(slice_returns), 1));
  
  lb = min(slice_returns(1:2,:), [], 2);
  ub = max(slice_returns(1:2,:), [], 2);
  lb = lb - options.margin(j);
  ub = ub + options.margin(j);
  if j > 1
    % The planner assumes that the slices are monotonically growing in all
    % dimensions as z increases
    lb = min([lb, slices.xy(:,:,j-1)], [], 2);
    ub = max([ub, slices.xy(:,:,j-1)], [], 2);
  end
    
  slices.xy(:,:,j) = [lb(1), ub(1), lb(1), ub(1);
                      lb(2), lb(2), ub(2), ub(2)];
  
  if DEBUG
    all_returns = [all_returns, slice_returns];
  end
end
  
if DEBUG
  figure(105)
  clf
  hold on
  plot3(all_returns(1,:), all_returns(2,:), all_returns(3,:), 'b.');
  axis equal
  for j = 1:length(slice_heights)-1
    verts = [slices.xy(:,:,j), slices.xy(:,:,j); repmat(slice_heights(j), 1, 4), repmat(slice_heights(j+1), 1, 4)];
    iris.drawing.drawPolyFromVertices(verts, 'r');
  end
end
