function rrt_bugtrap(planning_mode)

if nargin < 1, planning_mode = 'rrt_connect'; end
bugtrap = [ .35 .8; .49 .65; .49 .60; .35 .75; .25 .75; .25 .25; ...
  .75 .25; .75 .75; .65 .75; .51 .60; .51 .65; .65 .8; .8 .8; .8 .2; ...
  .2 .2; .2 .8; .35 .8]';

figure(1); clf; hold on;
patch(bugtrap(1,:),bugtrap(2,:),'r');
axis equal; 
axis([0 1 0 1]);
x_goal = [.1;.1];
x_start = [.3;.3];
plot(x_start(1),x_start(2),'bx',x_goal(1),x_goal(2),'gx','MarkerSize',20,'LineWidth',3);

prob = CartesianMotionPlanningTree(2);
prob = addConstraint(prob,FunctionHandleConstraint(0,0,2,@(x)inpolygon(x(1),x(2),bugtrap(1,:),bugtrap(2,:)),-2));

options = struct();
prob.max_edge_length = .1;
prob.max_length_between_constraint_checks = .005;
switch planning_mode
  case 'rrt'
    [TA, path_ids_A, info] = prob.rrtNew(x_start, x_goal, options);
  case 'rrt_connect'
    [TA, TB,  path_ids_A, path_ids_B, info] = prob.rrtConnect(x_start, x_goal, [], options);
    if info == 1
      [TA, id_last] = TA.addPath(TB, fliplr(path_ids_B(1:end-1)), path_ids_A(end));
      path_ids_A = TA.getPathToVertex(id_last);
    end
end

TA.drawPath(path_ids_A);

end


function xs = uniformSamples

xs = rand(2,1);

end
