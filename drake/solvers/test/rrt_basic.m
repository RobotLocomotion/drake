function rrt_basic

figure(1); clf; hold on;
x_start = rand(2,1);
x_goal = rand(2,1);
plot(x_start(1),x_start(2),'bx',x_goal(1),x_goal(2),'gx','MarkerSize',20,'LineWidth',3);

T = CartesianMotionPlanningTree(2);
T.max_edge_length = .1;
options.visualize = true;
[T, path_ids, info] = T.rrt(x_start, x_goal, options);
T.drawPath(path_ids);
end
