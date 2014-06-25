function plan = footstepMISOCP(biped, seed_plan, weights, goal_pos)
% New, experimental footstep planner based on a Mixed-Integer Second-Order Cone Program.
% This planner allows footstep rotation with a piecewise approximation of sine and cosine.

checkDependency('yalmip');
checkDependency('gurobi');
seed_plan.sanity_check();
rangecheck(seed_plan.footsteps(1).pos(6), -pi, pi);
rangecheck(seed_plan.footsteps(2).pos(6), -pi, pi);

nsteps = length(seed_plan.footsteps);
num_precise_steps = 10;
max_num_steps = seed_plan.params.max_num_steps + 2;
min_num_steps = max([seed_plan.params.min_num_steps + 2, 3]);

x = sdpvar(4, nsteps, 'full');
cos_yaw = sdpvar(1, num_precise_steps, 'full');
sin_yaw = sdpvar(1, num_precise_steps, 'full');
yaw = x(4,:);


trim = binvar(1, nsteps, 'full');
region = binvar(length(seed_plan.safe_regions), nsteps, 'full');


foci = [[0.05; 0.1], [0.05; -0.6]];
ellipse_l = 0.45;

seed_steps = [seed_plan.footsteps.pos];

min_yaw = pi * floor(seed_steps(6,1) / pi - 1);
max_yaw = pi * ceil(seed_steps(6,1) / pi + 1);

cos_boundaries = reshape(bsxfun(@plus, [min_yaw:pi:max_yaw; min_yaw:pi:max_yaw], [-(pi/2-1); (pi/2-1)]), 1, []);
sin_boundaries = reshape(bsxfun(@plus, [min_yaw:pi:max_yaw; min_yaw:pi:max_yaw], [-1; 1]), 1, []);

cos_sector = binvar(length(cos_boundaries) - 1, num_precise_steps, 'full');
sin_sector = binvar(length(sin_boundaries) - 1, num_precise_steps, 'full');
Constraints = [x(:,1) == seed_steps([1,2,3,6],1),...
               x(:,2) == seed_steps([1,2,3,6],2),...
               min_yaw <= yaw <= max_yaw,...
               x(1:3,:) >= -100 + repmat(seed_steps(1:3,1), 1, nsteps),...
               x(1:3,:) <= 100 + repmat(seed_steps(1:3,1), 1, nsteps)...
               -1 <= cos_yaw <= 1,...
               -1 <= sin_yaw <= 1,...
               region(:,1:2) == [1, 1; zeros(size(region, 1)-1, 2)],...
               sum(region, 1) >= 1,...
               sum(sin_sector, 1) == 1,...
               sum(cos_sector, 1) == 1,...
               trim(end-1:end) == 1,...
               trim(1:end-1) <= trim(2:end),...
               ];

% Enforce min number of steps
Constraints = [Constraints, ...
               sum(trim) <= nsteps - (min_num_steps - 2)];

for j = 1:num_precise_steps
  for s = 1:length(cos_boundaries) - 1
    th0 = cos_boundaries(s);
    th1 = cos_boundaries(s+1);

    th = (th0 + th1)/2;
    cos_slope = -sin(th);
    cos_intercept = cos(th) - (cos_slope * th);
    Constraints = [Constraints,...
                   implies(cos_sector(s, j), th0 <= yaw(j) <= th1),...
                   implies(cos_sector(s, j), cos_yaw(j) == cos_slope * yaw(j) + cos_intercept)];
  end
end

for j = 1:num_precise_steps
  for s = 1:length(sin_boundaries) - 1
    th0 = sin_boundaries(s);
    th1 = sin_boundaries(s+1);

    th = (th0 + th1)/2;
    sin_slope = cos(th);
    sin_intercept = sin(th) - (sin_slope * th);
    Constraints = [Constraints,...
                   implies(sin_sector(s, j), th0 <= yaw(j) <= th1),...
                   implies(sin_sector(s, j), sin_yaw(j) == sin_slope * yaw(j) + sin_intercept)];
  end
end

for j = 1:num_precise_steps
  for k = 1:size(sin_sector, 1)
    Constraints = [Constraints,...
                   sum(sin_sector(max(1,k-1):min(k+1,size(sin_sector,1)),j)) >= cos_sector(k,j),...
                   sum(cos_sector(max(1,k-1):min(k+1,size(cos_sector,1)),j)) >= sin_sector(k,j)];
  end
end

for j = 3:num_precise_steps
  % Ensure that the foot doesn't yaw too much per step
  if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.left
    rel_foci = [foci(1,:); -foci(2,:)];
    Constraints = [Constraints, 0 <= yaw(j) - yaw(j-1) <= pi/8];
    for k = 1:size(cos_sector, 1) - 1
      Constraints = [Constraints, sum(cos_sector(k:k+1,j)) >= cos_sector(k,j-1),...
                                  sum(sin_sector(k:k+1,j)) >= sin_sector(k,j-1)];
    end
  else
    rel_foci = foci;
    Constraints = [Constraints, -pi/8 <= yaw(j) - yaw(j-1) <= 0];
    for k = 2:size(cos_sector, 1)
      Constraints = [Constraints, sum(cos_sector(k-1:k,j)) >= cos_sector(k,j-1),...
                                  sum(sin_sector(k-1:k,j)) >= sin_sector(k,j-1)];
    end
  end

  % Enforce relative step reachability
  for k = 1:size(rel_foci, 2)
    Constraints = [Constraints, ...
      cone(x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * rel_foci(:,k) - x(1:2,j), ellipse_l),...
%       cone(x(1:3,j) - x(1:3,j-1) .* xyz_ellipse_weights, 1),...
      abs(x(3,j) - x(3,j-1)) <= seed_plan.params.nom_upward_step,...
      ];

  end

end

for j = num_precise_steps+1:nsteps
  % Ensure that the foot doesn't yaw too much per step
  if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.left
    Constraints = [Constraints, 0 <= yaw(j) - yaw(j-1) <= pi/8];
  else
    Constraints = [Constraints, -pi/8 <= yaw(j) - yaw(j-1) <= 0];
  end

  % Enforce approximate step reachability
  Constraints = [Constraints, ...
                 cone(x(1:2,j) - x(1:2,j-1), seed_plan.params.nom_forward_step),...
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

% trim(j) fixes step j to the final pose of that foot (so we can trim it out of the plan later)
for j = 3:(nsteps-2)
  if seed_plan.footsteps(j).frame_id == seed_plan.footsteps(end).frame_id
    Constraints = [Constraints, implies(trim(j), x(:,j) == x(:,end))];
  else
    Constraints = [Constraints, implies(trim(j), x(:,j) == x(:,end-1))];
  end
end

w_goal = diag(weights.goal([1,2,3,6]));
w_rel = diag(weights.relative([1,2,3,6]));
% w_trim = w_rel(1) * (seed_plan.params.nom_forward_step^2 + seed_plan.params.nom_step_width^2);
w_trim = 0;

if seed_plan.footsteps(end).frame_id == biped.foot_frame_id.right
  goal = goal_pos.right([1,2,3,6]);
else
  goal = goal_pos.left([1,2,3,6]);
end
Objective = (x(:,nsteps) - goal)' * w_goal * (x(:,nsteps) - goal);
for j = 2:nsteps
  if j == nsteps
    w_rel = diag(weights.relative_final([1,2,3,6]));
  end
  Objective = Objective + (x(:,j) - x(:,j-1))' * w_rel * (x(:,j) - x(:,j-1)) + -1 * w_trim * trim(j);
end

solvesdp(Constraints, Objective, sdpsettings('solver', 'gurobi'));

x = double(x);
yaw = double(yaw);
cos_yaw = double(cos_yaw);
sin_yaw = double(sin_yaw);
trim = double(trim);
region = double(region);
steps = zeros(6,nsteps);
steps([1,2,3,6],:) = x;

plan = seed_plan;
[region_order, ~] = find(abs(region - 1) < 1e-2);
assert(length(region_order) == size(region, 2));

plan = seed_plan;
for j = 1:nsteps
  plan.footsteps(j).pos = steps(:,j);
end
plan.region_order = region_order;

% Remove unnecessary footsteps
% trim(1:2) = 0;
trim(find(trim, 2, 'first')) = 0;
plan = plan.slice(~trim);

% Fix the order of the first two steps as necessary
if plan.footsteps(3).frame_id == plan.footsteps(2).frame_id
  plan = plan.slice([2,1,3:length(plan.footsteps)]);
end

