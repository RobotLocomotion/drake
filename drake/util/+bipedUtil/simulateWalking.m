function [ytraj, com, rms_com] = simulateWalking(r, walking_plan_data, options)
%NOTEST
if nargin < 3
  options = struct();
end
options = applyDefaults(options, struct('gui_control_interface', true));

typecheck(r, 'Biped');

if ~isfield(options, 'v') || isempty(options.v)
  v = r.constructVisualizer;
  v.display_dt = 0.05;
else
  v = options.v;
end

x0 = r.getInitialState();

% Build our controller and plan eval objects
control = bipedControllers.InstantaneousQPController(r, []);
planeval = bipedControllers.BipedPlanEval(r, walking_plan_data);
plancontroller = bipedControllers.BipedPlanEvalAndControlSystem(r, control, planeval);
sys = feedback(r, plancontroller);

% Add a visualizer
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);

% Simulate and draw the result
T = walking_plan_data.duration + 1;
ytraj = simulate(sys, [0, T], x0, options);
[com, rms_com] = bipedUtil.plotWalkingTraj(r, ytraj, walking_plan_data);

end
