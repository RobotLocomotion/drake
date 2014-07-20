function rrt_bugtrap

close all;
bugtrap = [ .35 .8; .49 .65; .49 .60; .35 .75; .25 .75; .25 .25; ...
  .75 .25; .75 .75; .65 .75; .51 .60; .51 .65; .65 .8; .8 .8; .8 .2; ...
  .2 .2; .2 .8; .35 .8]';

patch(bugtrap(1,:),bugtrap(2,:),'r');
axis equal; 
axis([0 1 0 1]);
x_goal = [.1;.1];
x_start = [.3;.3];

prob = MotionPlanningProblem(2);
prob = addConstraint(prob,FunctionHandleConstraint(0,0,2,@(x)inpolygon(x(1),x(2),bugtrap(1,:),bugtrap(2,:))));

options.figure_num = gcf;
options.max_edge_length = .1;
options.max_length_between_constraint_checks = .01;
prob.rrt(x_start,x_goal,.05,@uniformSamples,options);

end


function xs = uniformSamples

xs = rand(2,1);

end
