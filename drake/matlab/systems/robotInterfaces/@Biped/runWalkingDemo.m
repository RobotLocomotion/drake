function runWalkingDemo(obj, walking_options)
% Run the new split QP controller, which consists of separate PlanEval
% and InstantaneousQPController objects. The controller will also
% automatically transition to standing when it reaches the end of its walking
% plan.

checkDependency('gurobi');
checkDependency('lcmgl');

if nargin < 2; walking_options = struct(); end;

walking_options = applyDefaults(walking_options, struct('initial_pose', [],...
                                                        'navgoal', [1.5;0;0;0;0;0],...
                                                        'max_num_steps', 6,...
                                                        'rms_com_tolerance', 0.0051,...
                                                        'urdf_modifications_file', ''));
walking_options = applyDefaults(walking_options, obj.default_footstep_params);
walking_options = applyDefaults(walking_options, obj.default_walking_params);

% set initial state to fixed point
load(obj.fixed_point_file, 'xstar');
if ~isempty(walking_options.initial_pose), xstar(1:6) = walking_options.initial_pose; end
xstar = obj.resolveConstraints(xstar);
obj = obj.setInitialState(xstar);

v = obj.constructVisualizer;
v.display_dt = 0.01;

nq = getNumPositions(obj);

x0 = xstar;

% Find the initial positions of the feet
R=rotz(walking_options.navgoal(6));

rfoot_navgoal = walking_options.navgoal;
lfoot_navgoal = walking_options.navgoal;

rfoot_navgoal(1:3) = rfoot_navgoal(1:3) + R*[0;-0.13;0];
lfoot_navgoal(1:3) = lfoot_navgoal(1:3) + R*[0;0.13;0];

% Plan footsteps to the goal
goal_pos = struct('right', rfoot_navgoal, 'left', lfoot_navgoal);
footstep_plan = obj.planFootsteps(x0(1:nq), goal_pos, [], struct('step_params', walking_options));
for j = 1:length(footstep_plan.footsteps)
  footstep_plan.footsteps(j).walking_params = walking_options;
end

% Generate a dynamic walking plan
walking_plan_data = obj.planWalkingZMP(x0(1:obj.getNumPositions()), footstep_plan);

[ytraj, com, rms_com] = obj.simulateWalking(walking_plan_data, walking_options);

v.playback(ytraj, struct('slider', true));

if ~rangecheck(rms_com, 0, walking_options.rms_com_tolerance);
  error('Drake:runWalkingDemo:BadCoMTracking', 'Center-of-mass during execution differs substantially from the plan.');
end


