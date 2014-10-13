function testMISOSTrajectory()
% Test the Mixed-integer trajectory planner described in MISOSTrajectoryProblem.m on a very simple environment.
% Uses IRIS to find safe regions if it's available, or just computes them by hand if it's not available.

path_handle = addpathTemporary(fullfile(getDrakePath,'examples','Quadrotor'));
checkDependency('yalmip');

x0 = [.5; .1];
xf = [.1; .5];
lb = [0;0]; 
ub = [1;1];
dim = length(lb);
traj_degree = 3;
num_segments = 4;
num_iris_regions = 2;
                  
for h = [1,2]
  figure(h)
  clf
  hold on
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate obtsacles
obstacles = {[.2,.8,.8,.2; .2,.2,.8,.8]};

for h = [1,2]
  figure(h)
  for j = 1:length(obstacles)  
    obs = obstacles{j};
    patch(obs(1,:), obs(2,:), [.2,.2,.2]);
  end
  for j = 1:length(obstacles)  
    obs = obstacles{j};
    plot(obs(1,[1:end,1]), obs(2,[1:end,1]), 'k', 'LineWidth', 2);
  end
end

figure(1)
if checkDependency('iris')
  % Generate safe regions with IRIS
  iris_regions = iris.util.auto_seed_regions(obstacles, lb, ub, [x0,xf], num_iris_regions, 10, []);
  for j = 1:length(iris_regions)
    V = iris.thirdParty.polytopes.lcon2vert(iris_regions(j).A, iris_regions(j).b)';
    k = convhull(V(1,:), V(2,:), 'simplify', true);
    patch(V(1,k), V(2,k), [1,1,150/255], 'LineWidth', 3)
  end
else
  % Create safe regions manually (without IRIS)
  iris_regions = struct('A', {}, 'b', {});
  V = [0,.2,.2,0; 0, 0, 1, 1];
  [A, b] = poly2lincon(V(1,:), V(2,:));
  patch(V(1,:), V(2,:), [1,1,150/255], 'LineWidth', 3)
  iris_regions(end+1) = struct('A', A, 'b', b);

  V = [0,1,1,0; 0,0,.2,.2];
  [A, b] = poly2lincon(V(1,:), V(2,:));
  patch(V(1,:), V(2,:), [1,1,150/255], 'LineWidth', 3)
  iris_regions(end+1) = struct('A', A, 'b', b);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate safe regions from obstacle faces
non_iris_regions = {};
figure(2)
for j = 1:length(obstacles)
  obs = obstacles{j};
  non_iris_regions{j} = struct('A', {}, 'b', {});
  for k = 1:size(obs, 2)
    if k < size(obs,2)
      kn = k+1;
    else
      kn = 1;
    end
    ai = [0, -1; 1, 0] * [obs(1,kn)-obs(1,k); obs(2,kn) - obs(2,k)];
    ai = ai / norm(ai);
    bi = ai' * obs(:,k);
    non_iris_regions{j}(end+1) = struct('A', ai', 'b', bi);
    R = [0,-1;1,0];
    edge = [obs(:,k) - 2*R*ai, obs(:,k) + 2*R*ai];
    plot(edge(1,:), edge(2,:), 'k:', 'LineWidth', 2)
  end
end
A_bounds = [eye(dim); -eye(dim)];
b_bounds = [ub; -lb];
non_iris_regions{end+1} = struct('A', A_bounds, 'b', b_bounds);

for h = [1,2]
  figure(h)
  plot(x0(1), x0(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
  plot(xf(1), xf(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
end
    
drawnow();

prob = MISOSTrajectoryProblem();
prob.traj_degree = traj_degree;
prob.num_traj_segments = num_segments;
start = [x0, [0;0]];
goal = [xf, [0;0]];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Solve with IRIS regions
ytraj_iris = prob.solveTrajectory(start, goal, iris_regions);

figure(1)
breaks = ytraj_iris.getBreaks();
ts = linspace(breaks(1), breaks(end));
Y = ytraj_iris.eval(ts);
Ybreaks = ytraj_iris.eval(breaks);
plot(Y(1,:), Y(2,:), 'b-', 'LineWidth', 3);
plot(Ybreaks(1,:), Ybreaks(2,:), 'bo', 'MarkerSize', 7, 'MarkerFaceColor', 'b')
axis equal
xlim([lb(1),ub(1)]);
ylim([lb(2),ub(2)]);
set(gca, 'XTick', []);
set(gca, 'YTick', []);
box on
drawnow();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Solve without IRIS
ytraj_non_iris = prob.solveTrajectory(start, goal, non_iris_regions);

figure(2)
breaks = ytraj_non_iris.getBreaks();
ts = linspace(breaks(1), breaks(end));
Y = ytraj_non_iris.eval(ts);
Ybreaks = ytraj_non_iris.eval(breaks);
plot(Y(1,:), Y(2,:), 'b-', 'LineWidth', 3);
plot(Ybreaks(1,:), Ybreaks(2,:), 'bo', 'MarkerSize', 7, 'MarkerFaceColor', 'b')
axis equal
xlim([lb(1),ub(1)]);
ylim([lb(2),ub(2)]);
set(gca, 'XTick', []);
set(gca, 'YTick', []);
box on
drawnow();
