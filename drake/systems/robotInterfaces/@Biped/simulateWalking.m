function [ytraj, com, rms_com] = simulateWalking(obj, walking_plan_data, options)
if nargin < 3
  options = struct();
end
options = applyDefaults(options, struct('gui_control_interface', true,...
                                        'urdf_modifications_file', ''));

if ~isfield(options, 'v') || isempty(options.v)
  v = obj.constructVisualizer;
  v.display_dt = 0.05;
else
  v = options.v;
end

x0 = obj.getInitialState();

% Build our controller and plan eval objects
control = bipedControllers.InstantaneousQPController(obj.getManipulator().urdf{1}, obj.control_config_file, options.urdf_modifications_file);
planeval = bipedControllers.BipedPlanEval(obj, walking_plan_data);
plancontroller = bipedControllers.BipedPlanEvalAndControlSystem(obj, control, planeval);
sys = feedback(obj, plancontroller);

% Add a visualizer
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);

% Simulate and draw the result
T = walking_plan_data.duration + 1;
ytraj = simulate(sys, [0, T], x0, options);
[com, rms_com] = obj.plotWalkingTraj(ytraj, walking_plan_data);

end
