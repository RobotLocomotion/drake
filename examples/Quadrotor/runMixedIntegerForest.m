function [r, xtraj, utraj, prog] = runMixedIntegerOffice

javaaddpath([getenv('DRC_BASE'), '/software/build/share/java/lcmtypes_eigen-utils.jar']);
checkDependency('lcmgl');
r = Quadrotor();
lc = lcm.lcm.LCM.getSingleton();

r = Quadrotor();
% The important trees to create swerving path
r = addTree(r, [.8,.45,1.25], [.20;2.5], pi/4);
r = addTree(r, [.5,.35,1.65], [-.25;5], -pi/6);
r = addTree(r, [.55,.65,1.5], [.25;7.5], pi/4);
r = addTree(r, [.55,.85,1.6], [-1.35;8.5], pi/3.7);
r = addTree(r, [.85,.95,1.65], [-1.85;5.2], -pi/3.7);
r = addTree(r, [.75,.9,1.75], [2;4.4], -pi/5);
% Random trees to make forest bigger and more dense
r = addTrees(r, 25);

start = [0;-1.5;.5];
goal = [0; 11; 0.5];


  v = constructVisualizer(r);%,struct('use_contact_shapes',true));
  %v.draw(0,double(x0));


bot_bounding_box = RigidBodyBox([.6;.6;.6], [0;0;0], [0;0;0]).getPoints();

b = r.getBody(1);
obstacles = cell(1, length(b.getContactShapes()));
for j = 1:length(b.getContactShapes())
  obstacles{j} = iris.cspace.minkowski_sum(bot_bounding_box, b.getContactShapes{j}.getPoints());
end
padded = iris.pad_obstacle_points(obstacles);
obstacle_pts = cell2mat(reshape(padded, size(padded, 1), [], length(obstacles)));

lb = [-5;-2;0];
ub = [5;13;3];
A_bounds = [eye(3);-eye(3)];
b_bounds = [ub; -lb];

figure(1);
clf
hold on

seeds = [...
         start';
         goal';
         [0,5,.5];
         ]';
safe_regions = struct('A', {}, 'b', {});
for j = 1:size(seeds, 2)
  seed = seeds(:,j);
  [A, b] = iris.inflate_region(obstacle_pts, A_bounds, b_bounds, seed, struct('require_containment', true, 'error_on_infeas_start', false));
  V = iris.thirdParty.polytopes.lcon2vert(A,b)';
  iris.drawing.drawPolyFromVertices(V, 'r');

  msg = eigen_utils.eigen_matrixxd_t();
  msg.rows = size(V, 1);
  msg.cols = size(V, 2);
  msg.num_data = msg.rows * msg.cols;
  msg.data = reshape(V, [], 1);
  lc.publish('DRAW_POLYTOPE', msg);
  safe_regions(end+1) = struct('A', A, 'b', b);
end
drawnow();

degree = 3;
n_segments = 7;

ytraj = SOSTrajectory(start, goal, {safe_regions}, degree, n_segments);
% save('ytraj.mat', 'ytraj');
% load('ytraj.mat', 'ytraj');
ytraj = ytraj.setOutputFrame(DifferentiallyFlatOutputFrame);
xtraj_from_flat = invertFlatOutputs(r,ytraj);
v.playback(xtraj_from_flat, struct('slider', true));

