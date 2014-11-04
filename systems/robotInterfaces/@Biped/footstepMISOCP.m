function [plan, seed, solvertime] = footstepMISOCP(biped, seed_plan, weights, goal_pos)
% This planner is the first prototype of the mixed-integer planner
% described in "Footstep Planning on Uneven Terrain with Mixed-Integer 
% Convex Optimization" by Robin Deits and Russ Tedrake. 
% 
% Note: this is now obsolete: please use footstepMIQCQP.m instead.
% 
% This implementation uses a mixed-integer SOCP to plan the number of footsteps to take,
% the position and yaw of those steps, and the assignments of footsteps to
% convex regions of obstacle-free terrain. 
%
% This planner should be used by passing the 'method_handle', @footstepMISOCP 
% option to planFootsteps.m.
% 
% @param seed_plan a blank footstep plan, provinding the structure of the
%                  desired plan. Probably generated with
%                  FootstepPlan.blank_plan()
% @param weights a struct with fields 'goal', 'relative', and
%                'relative_final' describing the various contributions to
%                the cost function. These are described in detail in
%                Biped.getFootstepOptimizationWeights()
% @param goal_pos a struct with fields 'right' and 'left'.
%                 goal_pos.right is the desired 6 DOF pose
%                 of the right foot sole, and likewise for
%                 goal_pos.left

checkDependency('yalmip');
checkDependency('gurobi');
seed_plan.sanity_check();
rangecheck(seed_plan.footsteps(1).pos(6), -pi, pi);
rangecheck(seed_plan.footsteps(2).pos(6), -pi, pi);

nsteps = length(seed_plan.footsteps);

max_num_steps = seed_plan.params.max_num_steps + 2;
min_num_steps = max([seed_plan.params.min_num_steps + 2, 3]);

x = sdpvar(4, nsteps, 'full');
cos_yaw = sdpvar(1, nsteps, 'full');
sin_yaw = sdpvar(1, nsteps, 'full');
yaw = x(4,:);

seed_steps = [seed_plan.footsteps.pos];

min_yaw = pi * floor(seed_steps(6,1) / pi - 1);
max_yaw = pi * ceil(seed_steps(6,1) / pi + 1);

Constraints = [x(:,1) == seed_steps([1,2,3,6],1),...
               x(:,2) == seed_steps([1,2,3,6],2),...
               min_yaw <= yaw <= max_yaw,...
               x(1:3,:) >= -100 + repmat(seed_steps(1:3,1), 1, nsteps),...
               x(1:3,:) <= 100 + repmat(seed_steps(1:3,1), 1, nsteps)...
               ];

if strcmp(seed_plan.params.rot_mode, 'sincos_linear')
  [sincos_constraint, sin_sector, cos_sector] = sinCosPiecewiseLinear(sin_yaw, cos_yaw, yaw, min_yaw, max_yaw);
  Constraints = [Constraints, sincos_constraint, binary(sin_sector), binary(cos_sector)];
  for j = 3:nsteps
    % Ensure that the foot doesn't yaw too much per step
    if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.left
      Constraints = [Constraints, ...
                     0 <= yaw(j) - yaw(j-1) <= pi/8];
      for k = 1:size(cos_sector, 1) - 1
        Constraints = [Constraints, sum(cos_sector(k:k+1,j)) >= cos_sector(k,j-1),...
                                    sum(sin_sector(k:k+1,j)) >= sin_sector(k,j-1)];
      end
    else
      Constraints = [Constraints, ...
                     -pi/8 <= yaw(j) - yaw(j-1) <= 0];
      for k = 2:size(cos_sector, 1)
        Constraints = [Constraints, sum(cos_sector(k-1:k,j)) >= cos_sector(k,j-1),...
                                    sum(sin_sector(k-1:k,j)) >= sin_sector(k,j-1)];
      end
    end
  end
elseif strcmp(seed_plan.params.rot_mode, 'circle_linear_eq')
  [sincos_constraint, sector] = sinCosUnitCircleLinearEquality(sin_yaw, cos_yaw, yaw, min_yaw, max_yaw, 8);
  Constraints = [Constraints, sincos_constraint, binary(sector)];
  for j = 3:nsteps
    % Ensure that the foot doesn't yaw too much per step
    if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.left
      Constraints = [Constraints, ...
                     0 <= yaw(j) - yaw(j-1) <= pi/8];
      for k = 1:size(sector, 1) - 1
        Constraints = [Constraints, sum(sector(k:k+1,j)) >= sector(k,j-1)];
      end
    else
      Constraints = [Constraints, ...
                     -pi/8 <= yaw(j) - yaw(j-1) <= 0];
      for k = 2:size(sector, 1)
        Constraints = [Constraints, sum(sector(k-1:k,j)) >= sector(k,j-1)];
      end
    end
  end
else
  error('bad rot mode');
end


[trim_constr, trim] = finalTrimConstraint(x(:,3:end));
Constraints = [Constraints, trim_constr, binary(trim),...
               sum(trim) <= nsteps - (min_num_steps - 2)];

for j = 3:nsteps
  % Enforce relative step reachability
  [rel_foci, radii] = biped.getReachabilityCircles(seed_plan.params, seed_plan.footsteps(j-1).frame_id);
  for k = 1:size(rel_foci, 2)
    Constraints = [Constraints, ...
      cone(x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * rel_foci(:,k) - x(1:2,j), radii(k)),...
      abs(x(3,j) - x(3,j-1)) <= seed_plan.params.nom_upward_step];

  end
end

% Enforce membership in safe regions
[reg_const, region] = terrainRegionConstraint(seed_plan.safe_regions, x(:,3:end));
Constraints = [Constraints, reg_const, binary(region)];

w_goal = diag(weights.goal([1,1,3,6]));
w_rel = diag(weights.relative([1,1,3,6]));
w_trim = weights.relative(1) * (seed_plan.params.nom_forward_step^2);
%        + (seed_steps([1,2,3,6], 2) - seed_steps([1,2,3,6], 1))' * w_rel * (seed_steps([1,2,3,6], 2) - seed_steps([1,2,3,6], 1));

if seed_plan.footsteps(end).frame_id == biped.foot_frame_id.right
  goal1 = goal_pos.left([1,2,3,6]);
  goal2 = goal_pos.right([1,2,3,6]);
else
  goal1 = goal_pos.right([1,2,3,6]);
  goal2 = goal_pos.left([1,2,3,6]);
end
Objective = (x(:,nsteps-1) - goal1)' * w_goal * (x(:,nsteps-1) - goal1) ...
          + (x(:,nsteps) - goal2)' * w_goal * (x(:,nsteps) - goal2);
for j = 3:nsteps
  if j == nsteps
    w_rel = diag(weights.relative_final([1,1,3,6]));
  end
  if seed_plan.footsteps(j-1).frame_id == biped.foot_frame_id.right
    nom = [0; seed_plan.params.nom_step_width];
  else
    nom = [0; -seed_plan.params.nom_step_width];
  end
  err = x(:,j) - [(x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * nom); x(3:4,j-1)];
  Objective = Objective + err' * w_rel * err;

  if j < nsteps-1
    Objective = Objective + -1 * w_trim * trim(j-2);
  end
end

diagnostics = solvesdp(Constraints, Objective, sdpsettings('solver', 'gurobi', 'gurobi.MIPGap', 1e-4));
solvertime = diagnostics.solvertime;

x = double(x);
yaw = double(yaw);
cos_yaw = double(cos_yaw);
sin_yaw = double(sin_yaw);

figure(7)
clf
hold on
plot(cos_yaw, sin_yaw, 'bo');
plot(cos(linspace(0, 2*pi, 100)), sin(linspace(0, 2*pi, 100)), 'k-')
axis equal


trim = logical(round(double(trim)));
region = logical(round(double(region)));
steps = zeros(6,nsteps);
steps([1,2,3,6],:) = x;

plan = seed_plan;
region_order = nan(1, size(region, 2));
for j = 1:length(region_order)
  i = find(region(:,j));
  if ~isempty(i)
    region_order(j) = i;
  end
end
assert(length(region_order) == size(region, 2));
region_order = [1, 1, region_order];
plan.region_order = region_order;

for j = 1:nsteps
  plan.footsteps(j).pos = steps(:,j);
end

% Remove unnecessary footsteps
trim = [0, 0, trim, 1, 1];
trim(find(trim, 2, 'first')) = 0;
plan = plan.slice(~trim);

seed = [];

plan.sanity_check();

