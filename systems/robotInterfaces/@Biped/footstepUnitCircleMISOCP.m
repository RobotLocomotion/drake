function [plan, sin_yaw, cos_yaw] = footstepUnitCircleMISOCP(biped, seed_plan, weights, goal_pos)
% This planner is derived from the one presented in the Humanoids 2014 paper, but it performs
% the piecewise linear approximation on the unit circle of sin and cos, rather than performing
% separate approximations for each function of theta.
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

t0 = tic;
checkDependency('yalmip');
checkDependency('gurobi');
seed_plan.sanity_check();
rangecheck(seed_plan.footsteps(1).pos(6), -pi, pi);
rangecheck(seed_plan.footsteps(2).pos(6), -pi, pi);

nsteps = length(seed_plan.footsteps);
num_precise_steps = nsteps;
min_num_steps = max([seed_plan.params.min_num_steps + 2, 3]);

x = sdpvar(4, nsteps, 'full');
cos_yaw = sdpvar(1, num_precise_steps, 'full');
sin_yaw = sdpvar(1, num_precise_steps, 'full');
yaw = x(4,:);


trim = binvar(1, nsteps, 'full');

region = binvar(length(seed_plan.safe_regions), nsteps, 'full');

seed_steps = [seed_plan.footsteps.pos];

min_yaw = pi * floor(seed_steps(6,1) / pi - 1);
max_yaw = pi * ceil(seed_steps(6,1) / pi + 1);

angle_boundaries = (-pi-pi/16):(pi/8):(pi-pi/16);
sector = binvar(length(angle_boundaries) - 1, num_precise_steps, 'full');

Constraints = [x(:,1) == seed_steps([1,2,3,6],1),...
               x(:,2) == seed_steps([1,2,3,6],2),...
               min_yaw <= yaw <= max_yaw,...
               x(1:3,:) >= -100 + repmat(seed_steps(1:3,1), 1, nsteps),...
               x(1:3,:) <= 100 + repmat(seed_steps(1:3,1), 1, nsteps)...
               -1 <= cos_yaw <= 1,...
               -1 <= sin_yaw <= 1,...
               sum(region, 1) + trim == 1,...
               sum(sector, 1) == 1,...
               trim(1:2) == 1,...
               trim(1:end-1) >= trim(2:end),...
               ];

% Enforce min number of steps
Constraints = [Constraints, ...
               sum(trim) <= nsteps - (min_num_steps - 2)];

for j = 1:num_precise_steps
  for s = 1:length(angle_boundaries) - 1
    th0 = angle_boundaries(s);
    th1 = angle_boundaries(s+1);

    th = (th0 + th1)/2;
    ct = cos(th);
    st = sin(th);
    k = tan((th1 - th0)/2) / ((th1 - th0) / 2);
    Constraints = [Constraints,...
                   implies(sector(s,j), th0 <= yaw(j) <= th1)...
                   implies(sector(s,j), [cos_yaw(j); sin_yaw(j)] == [ct; st] + (yaw(j) - th) * k * [-st; ct])];
  end
end

for j = 3:num_precise_steps
  % Ensure that the foot doesn't yaw too much per step
  if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.left
    Constraints = [Constraints, ...
                   implies(~trim(j), 0 <= yaw(j) - yaw(j-1) <= pi/8)];
    for k = 1:size(sector, 1) - 1
      Constraints = [Constraints, sum(sector(k:k+1,j)) >= sector(k,j-1)];
    end
  else
    Constraints = [Constraints, ...
                   implies(~trim(j), -pi/8 <= yaw(j) - yaw(j-1) <= 0)];
    for k = 2:size(sector, 1)
      Constraints = [Constraints, sum(sector(k-1:k,j)) >= sector(k,j-1)];
    end
  end

  % Enforce relative step reachability
  [centers, radii] = biped.getReachabilityCircles(seed_plan.params, seed_plan.footsteps(j-1).frame_id);
  for k = 1:size(centers, 2)
    Constraints = [Constraints, ...
      polycone(x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * centers(:,k) - x(1:2,j), radii(k) + seed_plan.params.max_step_width * trim(j), 17),...
      abs(x(3,j) - x(3,j-1)) <= seed_plan.params.nom_upward_step];

  end

end

for j = num_precise_steps+1:nsteps
  % Ensure that the foot doesn't yaw too much per step
  if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.left
    Constraints = [Constraints, implies(~trim(j), 0 <= yaw(j) - yaw(j-1) <= pi/8)];
  else
    Constraints = [Constraints, implies(~trim(j), -pi/8 <= yaw(j) - yaw(j-1) <= 0)];
  end

  % Enforce approximate step reachability
  Constraints = [Constraints, ...
                 pcone(x(1:2,j) - x(1:2,j-1), seed_plan.params.nom_forward_step, 8),...
                 abs(x(3,j) - x(3,j-1)) <= seed_plan.params.nom_upward_step];
end

% Enforce membership in safe regions
for j = 3:nsteps
  for r = 1:length(seed_plan.safe_regions)
    Ar = [seed_plan.safe_regions(r).A(:,1:2), zeros(size(seed_plan.safe_regions(r).A, 1), 1), seed_plan.safe_regions(r).A(:,3)];
    Constraints = [Constraints, ...
       implies(region(r,j), Ar * x(:,j) <= seed_plan.safe_regions(r).b),...
       implies(region(r,j), seed_plan.safe_regions(r).normal' * x(1:3,j) == seed_plan.safe_regions(r).normal' * seed_plan.safe_regions(r).point)];
  end
end

% trim(j) fixes step j to the initial pose of that foot (so we can trim it out of the plan later)
for j = 3:(nsteps)
  if seed_plan.footsteps(j).frame_id == seed_plan.footsteps(1).frame_id
    Constraints = [Constraints, implies(trim(j), x(:,j) == seed_steps([1,2,3,6],1))];
  else
    Constraints = [Constraints, implies(trim(j), x(:,j) == seed_steps([1,2,3,6], 2))];
  end
end

w_goal = diag(weights.goal([1,1,3,6]));
w_rel = diag(weights.relative([1,1,3,6]));
w_trim = w_rel(1) * (seed_plan.params.nom_forward_step^2 + seed_plan.params.nom_step_width^2);
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
for j = 2:nsteps
  if j == nsteps
    w_rel = diag(weights.relative_final([1,1,3,6]));
  end
  Objective = Objective + (x(:,j) - x(:,j-1))' * w_rel * (x(:,j) - x(:,j-1)) + -1 * w_trim * trim(j);
end

fprintf(1, 'setup: %f\n', toc(t0));
t0 = tic;
solvesdp(Constraints, Objective, sdpsettings('solver', 'gurobi'));
fprintf(1, 'solve: %f\n', toc(t0));

x = double(x);
cos_yaw = double(cos_yaw);
sin_yaw = double(sin_yaw);
trim = double(trim);
region = double(region);
steps = zeros(6,nsteps);
steps([1,2,3,6],:) = x;

plan = seed_plan;
region_order = nan(1, size(region, 2));
for j = 1:length(region_order)
  i = find(round(region(:,j)));
  if ~isempty(i)
    region_order(j) = i;
  end
end
assert(length(region_order) == size(region, 2));
plan.region_order = region_order;

for j = 1:nsteps
  plan.footsteps(j).pos = steps(:,j);
end

% Remove unnecessary footsteps
trim = round(trim);
trim(find(trim, 2, 'last')) = 0;
plan = plan.slice(~trim);
sin_yaw = sin_yaw(~trim);
cos_yaw = cos_yaw(~trim);

plan.sanity_check();

end

function constraint = pcone(v, r, num_pieces)
  % Polynomial approximation of a conic constraint norm(v) <= r
  A = zeros(num_pieces, 2);
  b = repmat(r, num_pieces, 1);
  ths = linspace(0, 2*pi, num_pieces);
  for j = 1:num_pieces
    th = ths(j);
    c = cos(th);
    s = sin(th);
    R = [c, -s; s, c];
    A(j,:) = (R * [1;0])';
  end
  constraint = A * v <= b;
end
