function runMixedIntegerEnvironment(r, start, goal, lb, ub, seeds, degree, n_segments, n_regions)

bot_radius = 0.3;

lc = lcm.lcm.LCM.getSingleton();
lcmgl = LCMGLClient('quad_goal');
lcmgl.glColor3f(.2,.8,.2);
lcmgl.sphere(goal, .1, 20, 20);
lcmgl.switchBuffers();

v = constructVisualizer(r);%,struct('use_contact_shapes',true));
x0 = double(Point(getStateFrame(r)));  % initial conditions: all-zeros
x0(1:3) = start;
v.draw(0,double(x0));

b = r.getBody(1);
obstacles = cell(1, length(b.getContactShapes()));
for j = 1:length(b.getContactShapes())
  obstacles{j} = b.getContactShapes{j}.getPoints();
end
padded = iris.pad_obstacle_points(obstacles);
obstacle_pts = cell2mat(reshape(padded, size(padded, 1), [], length(obstacles)));

A_bounds = [eye(3);-eye(3)];
b_bounds = [ub; -lb];

figure(1);
clf
hold on

obs_regions = struct('A', cell(1, size(obstacle_pts, 3)), 'b', cell(1, size(obstacle_pts, 3)));
for j = 1:size(obstacle_pts, 3)
  [obs_regions(j).A, obs_regions(j).b] = iris.thirdParty.polytopes.vert2lcon(obstacle_pts(:,:,j)');
end
safe_regions = struct('A', {}, 'b', {});

% Clear the displayed polytopes
sendLCMPolytope(0, 0, 0, 0, lc);
pause

% Draw the bounding box
sendLCMPolytope(A_bounds, b_bounds, 100, true, lc);
pause

% Clear the displayed polytopes
sendLCMPolytope(0, 0, 0, 0, lc);

lcmgl = LCMGLClient('region_seed');
for j = 1:size(seeds, 2)
  seed = seeds(:,j);
  [A, b] = iris.inflate_region(obstacle_pts, A_bounds, b_bounds, seed, struct('require_containment', true, 'error_on_infeas_start', true));
  safe_regions(end+1) = struct('A', A, 'b', b);
  sendLCMPolytope(A, b, j, false, lc);
  lcmgl.glColor3f(.8,.8,.2);
  lcmgl.sphere(seed, 0.06, 20, 20);
  lcmgl.switchBuffers();
  pause
end

num_cells = 10;
step_size = (ub - lb) ./ num_cells;

while length(safe_regions) < n_regions
  dists = iris.util.obstacle_distance_matrix([obs_regions, safe_regions], lb, ub, num_cells);
  [~, idx] = max(dists(:));
  [i,j,k] = ind2sub(size(dists), idx);
  seed = ([i;j;k] - 1) .* step_size + lb;
  [A, b] = iris.inflate_region(obstacle_pts, A_bounds, b_bounds, seed, struct('require_containment', true, 'error_on_infeas_start', true));
  safe_regions(end+1) = struct('A', A, 'b', b);
  sendLCMPolytope(A, b, length(safe_regions), false, lc);
end


for j = 1:length(safe_regions)
  A = safe_regions(j).A;
  b = safe_regions(j).b;
  V = iris.thirdParty.polytopes.lcon2vert(A,b)';
  iris.drawing.drawPolyFromVertices(V, 'r');
end
drawnow();

ytraj = SOSTrajectory(start, goal, {safe_regions}, degree, n_segments, bot_radius);
folder_name = ['~/locomotion/papers/icra-2015-uav-miqp/data/', datestr(now,'yyyy-mm-dd_HH.MM.SS')];
system(sprintf('mkdir -p %s', folder_name));
% load('ytraj.mat', 'ytraj');
ytraj = ytraj.setOutputFrame(DifferentiallyFlatOutputFrame);
xtraj = invertFlatOutputs(r,ytraj);
save([folder_name, '/results.mat'], 'ytraj', 'r', 'v', 'xtraj');
v.playback(xtraj, struct('slider', true));

