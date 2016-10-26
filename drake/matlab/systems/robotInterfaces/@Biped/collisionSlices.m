function slices = collisionSlices(obj, q, slice_heights, varargin)
%NOTEST
% Compute bounding boxes of the robot's geometry at a given configuration
% @param q the configuration vector
% @param slice_heights vector of z values (m) providing the limits of the bounding boxes

p = inputParser();
p.addRequired('q');
p.addRequired('slice_heights', @isnumeric);
p.addParamValue('scan_radius', 3, @isscalar);
p.addParamValue('margin', 0.05, @isnumeric);
p.addParamValue('debug', false);
p.parse(q, slice_heights, varargin{:});
options = p.Results;

if length(options.margin) == 1
  options.margin = repmat(options.margin, 1, length(slice_heights)-1);
end

% ensure that the right foot is at 0 0 0
feet_pose = obj.feetPosition(q);
q([1,2,3,6]) = q([1,2,3,6]) - feet_pose.right([1,2,3,6]);

scan_center = q(1:2);

kinsol = doKinematics(obj, q);

slices = struct('z', slice_heights(1:end-1),...
                'xy', zeros([2, 4, length(slice_heights)]));
nslices = length(slice_heights)-1;
              
num_ths = 50;
ths = linspace(-pi, pi, num_ths);
num_zs_per_slice = 10;
zs = zeros(1, num_zs_per_slice * nslices);
for j = 1:nslices
  zs((j-1)*num_zs_per_slice+(1:num_zs_per_slice)) = linspace(slice_heights(j), slice_heights(j+1), num_zs_per_slice);
end
num_returns_per_slice = num_ths * num_zs_per_slice;

[Z, TH] = meshgrid(zs, ths);
origin_X = scan_center(1) + options.scan_radius * cos(TH);
origin_Y = scan_center(2) + options.scan_radius * sin(TH);
target_X = scan_center(1) + zeros(size(TH));
target_Y = scan_center(2) + zeros(size(TH));
origins = [reshape(origin_X, 1, []); reshape(origin_Y, 1, []); reshape(Z, 1, [])];
targets = [reshape(target_X, 1, []); reshape(target_Y, 1, []); reshape(Z, 1, [])];
distances = collisionRaycast(obj, kinsol, origins, targets, false);
distances(distances <= 0) = nan;

normals = bsxfun(@rdivide, targets - origins, sqrt(sum((targets - origins).^2, 1)));
returns = origins + bsxfun(@times, normals, distances');


if options.debug
  all_returns = zeros(3, 0);
end
for j = 1:nslices
  slice_returns = returns(:,(j-1)*num_returns_per_slice+(1:num_returns_per_slice));

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
  
  if options.debug
    all_returns = [all_returns, slice_returns];
  end
end
  
if options.debug
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
