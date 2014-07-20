function rrt_basic

close all;

prob = MotionPlanningProblem(2);

options.max_edge_length = .1;
prob.rrt(rand(2,1),rand(2,1),.05,@uniformSamples,options);

end

function xs = uniformSamples

xs = rand(2,1);

end
