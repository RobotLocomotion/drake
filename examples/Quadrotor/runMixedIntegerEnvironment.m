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
figure(1);
clf
hold on


% Clear the displayed polytopes
drawLCMPolytope(0, 0, 0, 0, lc);
% pause

% Draw the bounding box
dim = length(lb);
A_bounds = [eye(dim);-eye(dim)];
b_bounds = [ub; -lb];
drawLCMPolytope(A_bounds, b_bounds, 100, true, lc);
% pause

% Clear the displayed polytopes
drawLCMPolytope(0, 0, 0, 0, lc);

region_idx = 1;
function done = drawRegion(r, seed)
  drawLCMPolytope(r.A, r.b, region_idx, false, lc);
  region_idx = region_idx + 1;
  lcmgl.glColor3f(.8,.8,.2);
  lcmgl.sphere(seed, 0.06, 20, 20);
  lcmgl.switchBuffers();
  done = false;
end

num_steps = 10;
safe_regions = iris.util.auto_seed_regions(obstacles, lb, ub, seeds, n_regions, num_steps, @drawRegion);

drawnow();

ytraj = findSOSTrajectory(start, goal, {safe_regions}, degree, n_segments, bot_radius);
folder_name = ['~/locomotion/papers/icra-2015-uav-miqp/data/', datestr(now,'yyyy-mm-dd_HH.MM.SS')];
system(sprintf('mkdir -p %s', folder_name));
% load('ytraj.mat', 'ytraj');
ytraj = ytraj.setOutputFrame(DifferentiallyFlatOutputFrame);
xtraj = invertFlatOutputs(r,ytraj);
save([folder_name, '/results.mat'], 'ytraj', 'r', 'v', 'xtraj');
v.playback(xtraj, struct('slider', true));

lc = lcm.lcm.LCM.getSingleton();
lcmgl = drake.util.BotLCMGLClient(lc, 'quad_trajectory');
lcmgl.glBegin(lcmgl.LCMGL_LINES);
lcmgl.glColor3f(0.0,0.0,1.0);

breaks = ytraj.getBreaks();
ts = linspace(breaks(1), breaks(end));
Y = ytraj.eval(ts);
Ybreaks = ytraj.eval(breaks);
plot3(Y(1,:), Y(2,:), Y(3,:), 'b-');
plot3(Ybreaks(1,:), Ybreaks(2,:), Ybreaks(3,:), 'bo')
for i = 1:size(Y, 2)-1
  lcmgl.glVertex3f(Y(1,i), Y(2,i), Y(3,i));
  lcmgl.glVertex3f(Y(1,i+1), Y(2,i+1), Y(3,i+1));
end

lcmgl.glEnd();
lcmgl.switchBuffers();
end

