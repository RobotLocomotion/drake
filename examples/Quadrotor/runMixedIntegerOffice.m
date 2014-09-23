function [r, xtraj, utraj, prog] = runMixedIntegerOffice

javaaddpath([getenv('DRC_BASE'), '/software/build/share/java/lcmtypes_eigen-utils.jar']);
checkDependency('lcmgl');
r = Quadrotor();
lc = lcm.lcm.LCM.getSingleton();

degree = 3;
n_segments = 7;
% start = [5;-1;1.5];
start = [-2;0;1];
% goal = [0;0;.5];
goal = [5;-1;1.5];
% goal = [6;-1;1.5];

wall_height = 3.0;


%room walls
r = addBox(r, [10.0,.1,wall_height], [0;3.15],0);
r = addBox(r, [10.0,.1,wall_height], [0;-3.15],0);
r = addBox(r, [.1,8.0,1.0], [4.0;0],0);
r = addBox(r, [.1,8.0,wall_height], [-4.0;0],0);

%room internal wall
r = addBox(r, [.1,3.7,wall_height], [2.5;-1.65],0);
r = addBox(r, [.1,1.5,wall_height], [2.5;2.5],0);

%room window
r = addBox(r, [.1,4.0,wall_height], [4.0;1.5],0);
r = addBox(r, [.1,1.75,wall_height], [4.0;-2.5],0);
r = addFloatingBox(r, [.28,1.5,wall_height-2], [4.0,-.95,2.5], 0, [83,53,10]/255);

%roof
%r = addBox(r, [10,8,2.0], [0;0],0);

%cabinet
r = addFloatingBox(r, [.8,1.7,1.7],[-3.5,-1.8,0.8],0);

r = addFloatingBox(r, [.6,1.5,.4],[-3.3,-1.8,1.3],0);
r = addFloatingBox(r, [.6,1.5,.4],[-3.3,-1.8,0.8],0);
r = addFloatingBox(r, [.6,1.5,.4],[-3.3,-1.8,0.3],0);

r = addTable(r, [.2,.2,1.0], [-1,-1,.5],2);


  v = constructVisualizer(r);%,struct('use_contact_shapes',true));
  %v.draw(0,double(x0));


% bot_bounding_box = RigidBodyBox([.6;.6;.6], [0;0;0], [0;0;0]).getPoints();

b = r.getBody(1);
obstacles = cell(1, length(b.getContactShapes()));
for j = 1:length(b.getContactShapes())
  % obstacles{j} = iris.cspace.minkowski_sum(bot_bounding_box, b.getContactShapes{j}.getPoints());
  obstacles{j} = b.getContactShapes{j}.getPoints();
end
padded = iris.pad_obstacle_points(obstacles);
obstacle_pts = cell2mat(reshape(padded, size(padded, 1), [], length(obstacles)));


lb = [-5;-3.5;0.01];
ub = [10;3.5;3];
A_bounds = [eye(3);-eye(3)];
b_bounds = [ub; -lb];




figure(1);
clf
hold on

seeds = [...
         start'; 
         goal';
         % [1, 0, .5];
         % [2, 0, .5];
         [3, 0, 1];
         [2.5, .75, 1];
         [4, -1, 1.5];
         % [0, 0, 2];
         % [-2, 0, 1];
         ]';
obs_regions = struct('A', cell(1, size(obstacle_pts, 3)), 'b', cell(1, size(obstacle_pts, 3)));
for j = 1:size(obstacle_pts, 3)
  [obs_regions(j).A, obs_regions(j).b] = iris.thirdParty.polytopes.vert2lcon(obstacle_pts(:,:,j)');
end
safe_regions = struct('A', {}, 'b', {});
for j = 1:size(seeds, 2)
  seed = seeds(:,j);
  [A, b] = iris.inflate_region(obstacle_pts, A_bounds, b_bounds, seed, struct('require_containment', true, 'error_on_infeas_start', true));
  safe_regions(end+1) = struct('A', A, 'b', b);
  V = iris.thirdParty.polytopes.lcon2vert(A,b)';
  msg = eigen_utils.eigen_matrixxd_t();
  msg.rows = size(V, 1);
  msg.cols = size(V, 2);
  msg.num_data = msg.rows * msg.cols;
  msg.data = reshape(V, [], 1);
  lc.publish('DRAW_POLYTOPE', msg);
end

num_cells = 10;
step_size = (ub - lb) ./ num_cells;

while length(safe_regions) < 7
  dists = iris.util.obstacle_distance_matrix([obs_regions, safe_regions], lb, ub, num_cells);
  [~, idx] = max(dists(:));
  [i,j,k] = ind2sub(size(dists), idx);
  seed = ([i;j;k] - 1) .* step_size + lb;
  [A, b] = iris.inflate_region(obstacle_pts, A_bounds, b_bounds, seed, struct('require_containment', true, 'error_on_infeas_start', true));
  safe_regions(end+1) = struct('A', A, 'b', b);
  V = iris.thirdParty.polytopes.lcon2vert(A,b)';
  msg = eigen_utils.eigen_matrixxd_t();
  msg.rows = size(V, 1);
  msg.cols = size(V, 2);
  msg.num_data = msg.rows * msg.cols;
  msg.data = reshape(V, [], 1);
  lc.publish('DRAW_POLYTOPE', msg);
end


for j = 1:length(safe_regions)
  A = safe_regions(j).A;
  b = safe_regions(j).b;
  V = iris.thirdParty.polytopes.lcon2vert(A,b)';
  iris.drawing.drawPolyFromVertices(V, 'r');
end
drawnow();

ytraj = SOSTrajectory(start, goal, {safe_regions}, degree, n_segments, 0.6);
% save('ytraj.mat', 'ytraj');
% load('ytraj.mat', 'ytraj');
ytraj = ytraj.setOutputFrame(DifferentiallyFlatOutputFrame);
xtraj_from_flat = invertFlatOutputs(r,ytraj);
v.playback(xtraj_from_flat, struct('slider', true));

