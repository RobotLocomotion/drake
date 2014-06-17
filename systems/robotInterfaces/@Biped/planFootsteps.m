function plan = planFootsteps(obj, start_pos, goal_pos, safe_regions, options)
% planFootsteps: find a set of reachable foot positions from the start to
% the goal. 
% @param start_pos a struct with fields 'right' and 'left'.
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
% @option method (default: @footstepAlternatingMIQP) the footstep planning 
%                method to use, expressed as a function handle
% @option step_params (default: struct()) specific parameters for footstep
%                     planning. Attributes set here overload those in
%                     obj.default_footstep_params

if nargin < 5; options = struct(); end
if ~isfield(options, 'method'); options.method = @footstepAlternatingMIQP; end
if ~isfield(options, 'step_params'); options.step_params = struct(); end
options.step_params = obj.applyDefaultFootstepParams(options.step_params);
sizecheck(start_pos.right, [6,1]);
sizecheck(start_pos.left, [6,1]);
sizecheck(goal_pos.right, [6,1]);
sizecheck(goal_pos.left, [6,1]);

if isempty(safe_regions)
  n = rpy2rotmat(start_pos.right(4:6)) * [0;0;1];
  pt = start_pos.right(1:3);
  safe_regions = [struct('A', zeros(0,4), 'b', zeros(0,1), 'point', pt, 'normal', n)];
end
for j = 1:length(safe_regions)
  sizecheck(safe_regions(j).A, [NaN, 4]);
  sizecheck(safe_regions(j).b, [size(safe_regions(j).A, 1), 1]);
  sizecheck(safe_regions(j).point, [3,1]);
  sizecheck(safe_regions(j).normal, [3,1]);
end

% Currently we always lead with the right foot, but this should change soon
plan = FootstepPlan.blank_plan(options.step_params.max_num_steps + 2, [obj.foot_frame_id.right, obj.foot_frame_id.left], options.step_params, safe_regions);
plan.footsteps(1).pos = start_pos.right;
plan.footsteps(2).pos = start_pos.left;


start_pos.right(4:5) = 0;
start_pos.left(4:5) = 0;

weights = getFootstepOptimizationWeights(obj);

plan = options.method(obj, plan, weights, goal_pos);

end

