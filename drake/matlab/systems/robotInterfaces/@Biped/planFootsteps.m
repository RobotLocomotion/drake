function [plan, solvertime] = planFootsteps(obj, start_pos_or_q, goal_pos, safe_regions, options)
% planFootsteps: find a set of reachable foot positions from the start to
% the goal.
% @param start_pos_or_q a struct with fields 'right' and 'left' OR a configuration vector
%                  start_pos.right is the 6 DOF initial pose
%                  of the right foot sole and start_pos.left
%                  is the 6 DOF pose of the left sole.
% @param goal_pos a struct with fields 'right' and 'left'.
%                 goal_pos.right is the desired 6 DOF pose
%                 of the right foot, and likewise for
%                 goal_pos.left
% @params safe_regions a list of planar polytopic regions into which the footstep
%                      locations must be placed. Can be empty. If
%                      safe_regions == [], then the footstep locations are
%                      not constrained in this way. Otherwise, each
%                      footstpe must fall within at least one of the
%                      defined safe_regions. safe_regions should be a list
%                      of objects or structures which have fields 'A',
%                      'b', 'point', and 'normal' which define a region in
%                      v = [x,y,z,yaw] as
%                      A*v <= b AND normal'*v(1:3) == normal'*point
% @param options a struct of options
%
% @option method_handle (default: @footstepPlanner.alternatingMIQP) the footstep planning
%                method to use, expressed as a function handle
% @option step_params (default: struct()) specific parameters for footstep
%                     planning. Attributes set here overload those in
%                     obj.default_footstep_params

if nargin < 5; options = struct(); end
if nargin < 4; safe_regions = []; end
if ~isfield(options, 'method_handle'); options.method_handle = @footstepPlanner.alternatingMIQP; end
if ~isfield(options, 'step_params'); options.step_params = struct(); end
options.step_params = obj.applyDefaultFootstepParams(options.step_params);

if isnumeric(start_pos_or_q)
  start_pos = obj.feetPosition(start_pos_or_q);
else
  typecheck(start_pos_or_q, 'struct');
  start_pos = start_pos_or_q;
end
sizecheck(start_pos.right, [6,1]);
sizecheck(start_pos.left, [6,1]);
sizecheck(goal_pos.right, [6,1]);
sizecheck(goal_pos.left, [6,1]);

if isempty(safe_regions)
  terrain = obj.getTerrain();
  if isempty(terrain)
    % Use the plane of the feet to define the terrain
    n = rpy2rotmat(start_pos.right(4:6)) * [0;0;1];
    pt = start_pos.right(1:3);
  else
    [z, n] = terrain.getHeight(start_pos.right(1:2));
    pt = [start_pos.right(1:2); z];
  end
  safe_regions = [struct('A', zeros(0,3), 'b', zeros(0,1), 'point', pt, 'normal', n)];
end
for j = 1:length(safe_regions)
  sizecheck(safe_regions(j).A, [NaN, 3]);
  sizecheck(safe_regions(j).b, [size(safe_regions(j).A, 1), 1]);
  sizecheck(safe_regions(j).point, [3,1]);
  sizecheck(safe_regions(j).normal, [3,1]);
  for k = 1:size(safe_regions(j).A, 1)
    n = norm(safe_regions(j).A(k,:));
    safe_regions(j).A(k,:) = safe_regions(j).A(k,:) / n;
    safe_regions(j).b(k) = safe_regions(j).b(k) / n;
  end
end

if options.step_params.leading_foot == 0 % lead left
  foot1 = 'left';
  foot2 = 'right';
elseif options.step_params.leading_foot == 1 % lead right
  foot1 = 'right';
  foot2 = 'left';
elseif options.step_params.leading_foot == -1 % lead auto
  obj.warning_manager.warnOnce('Drake:planFootsteps:LeadAutoNotImplemented', 'AUTO leading foot selection is not implemented; will lead with RIGHT foot always');
  foot1 = 'right';
  foot2 = 'left';
else
  error('Drake:planFootsteps:InvalidLeadingFoot', 'Invalid value of options.step_params.leading_foot');
end
plan = FootstepPlan.blank_plan(obj, options.step_params.max_num_steps + 2, [obj.foot_frame_id.(foot1), obj.foot_frame_id.(foot2)], options.step_params, safe_regions);
plan.footsteps(1).pos = start_pos.(foot1);
plan.footsteps(2).pos = start_pos.(foot2);


start_pos.right(4:5) = 0;
start_pos.left(4:5) = 0;

weights = getFootstepOptimizationWeights(obj);

% Shift the problem to always start from x,y,z,yaw = 0;
T = makehgtform('zrotate', -start_pos.right(6), 'translate', -start_pos.right(1:3));
plan = plan.applyTransform(T);
for f = fieldnames(goal_pos)'
  field = f{1};
  G = poseRPY2tform(goal_pos.(field));
  goal_pos.(field) = tform2poseRPY(T * G);
end

try
  [plan, solvertime] = options.method_handle(obj, plan, weights, goal_pos);
catch e
  if strcmp(e.identifier, 'Drake:MixedIntegerConvexProgram:InfeasibleProblem')
    warning('The footstep planning problem is infeasible. Returning just the initial footstep poses');
    plan = plan.slice(1:2);
    solvertime = 0;
  else
    rethrow(e);
  end
end

plan = plan.applyTransform(inv(T));

end

