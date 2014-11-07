function [plan, solvertime] = relaxedMISOCP(biped, seed_plan, weights, goal_pos, ~)
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

SECTOR_WIDTH = pi/8;
POSE_INDICES = [1,2,3,6]; % which indices of xyzrpy we are searching over
MAX_DISTANCE = 30;

TRIM_THRESHOLD = [0.02; 0.02; inf; inf; inf; pi/16];

POLYCONE_APPROX_LEVEL = 0;

REACHABILITY_METHOD = 'ellipse';

nsteps = length(seed_plan.footsteps);
min_num_steps = max([seed_plan.params.min_num_steps + 2, 3]);

x = sdpvar(4, nsteps, 'full');
cos_yaw = sdpvar(1, nsteps, 'full');
sin_yaw = sdpvar(1, nsteps, 'full');
yaw = x(4,:);

seed_steps = [seed_plan.footsteps.pos];

min_yaw = pi * floor(seed_steps(6,1) / pi - 1);
max_yaw = pi * ceil(seed_steps(6,1) / pi + 1);

Constraints = [x(:,1) == seed_steps(POSE_INDICES,1),...
               x(:,2) == seed_steps(POSE_INDICES,2),...
               cos_yaw(1:2) == cos(seed_steps(6,1:2)),...
               sin_yaw(1:2) == sin(seed_steps(6,1:2)),...
               min_yaw <= yaw <= max_yaw,...
               -MAX_DISTANCE + seed_steps(1,1) <= x(1,:) <= MAX_DISTANCE + seed_steps(1,1),...
               -MAX_DISTANCE + seed_steps(2,1) <= x(2,:) <= MAX_DISTANCE + seed_steps(2,1),...
               -MAX_DISTANCE + seed_steps(3,1) <= x(3,:) <= MAX_DISTANCE + seed_steps(3,1),...
               -1 <= cos_yaw <= 1,...
               -1 <= sin_yaw <= 1,...
               ];
Objective = 0;

%% Enforce reachability
if strcmp(REACHABILITY_METHOD, 'ellipse')
  for j = 3:nsteps
    [foci, l] = biped.getReachabilityEllipse(seed_plan.params, seed_plan.footsteps(j-1).frame_id);
    % expr = 0;
    dists = sdpvar(1, size(foci, 2), 'full');
    for k = 1:size(foci, 2)
      Constraints = [Constraints,...
        coneOrPolyCone(x(1:2,j) - (x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * foci(:,k)), dists(k), POLYCONE_APPROX_LEVEL),...
        ];
      % expr = expr + norm(x(1:2,j) - (x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * foci(:,k)));
    end
    % Constraints = [Constraints, expr <= l];
    Constraints = [Constraints, sum(dists) <= l];
  end
elseif strcmp(REACHABILITY_METHOD, 'circles')
  for j = 3:nsteps
    [centers, radii] = biped.getReachabilityCircles(seed_plan.params, seed_plan.footsteps(j-1).frame_id);
    for k = 1:size(centers, 2)
      Constraints = [Constraints, cone(x(1:2,j) - (x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * centers(:,k)), radii(k))];
    end
  end
else
  error('bad reachability method');
end

%% Enforce z and yaw reachability
for j = 3:nsteps
  Constraints = [Constraints,...
    -seed_plan.params.nom_downward_step <= x(3,j) - x(3,j-1) <= seed_plan.params.nom_upward_step,...
    ];
  if seed_plan.footsteps(j-1).frame_id == biped.foot_frame_id.right
    Constraints = [Constraints,...
      -seed_plan.params.max_inward_angle <= x(4,j) - x(4,j-1) <= seed_plan.params.max_outward_angle];
  elseif seed_plan.footsteps(j-1).frame_id == biped.foot_frame_id.left
    Constraints = [Constraints,...
      -seed_plan.params.max_inward_angle <= x(4,j-1) - x(4,j) <= seed_plan.params.max_outward_angle];
  else
    error('invalid frame ID: %d', seed_plan.footsteps(j-1).frame_id);
  end
end

%% Enforce rotation convex relaxation
for j = 3:nsteps
  Constraints = [Constraints,...
    coneOrPolyCone([cos_yaw(j); sin_yaw(j)], 1, POLYCONE_APPROX_LEVEL)];
end
% for j = 3:nsteps
%   Constraints = [Constraints,...
%     coneOrPolyCone([cos_yaw(j) - cos_yaw(j-1); sin_yaw(j) - sin_yaw(j-1)], pi/8, POLYCONE_APPROX_LEVEL)];
% end

%% Enforce rotation mixed-integer constraints
sector_boundaries = (min_yaw):SECTOR_WIDTH:(max_yaw);
sector = binvar(length(sector_boundaries) - 1, nsteps, 'full');

Constraints = [Constraints, sum(sector, 1) == 1];

for q = 1:length(sector_boundaries)-1
  th0 = sector_boundaries(q);
  th1 = sector_boundaries(q+1);
  c0 = cos(th0);
  s0 = sin(th0);
  c1 = cos(th1);
  s1 = sin(th1);
  v = [c1; s1] - [c0; s0];
  d0 = v' * [c0; s0];
  d1 = v' * [c1; s1];
  u = [0, 1; -1, 0] * v;
  u = u / norm(u);

  Constraints = [Constraints,...
    yaw >= th0 + 2*pi*(sector(q,:)-1),...
    yaw <= th1 + 2*pi*(1-sector(q,:)),...
    ];
  for j = 3:nsteps
    Constraints = [Constraints,...
      % yaw(j) >= th0 - 2*pi + 2*pi*sector(q,j),...
      % yaw(j) <= th1 + 2*pi - 2*pi*sector(q,j),...
      u' * [cos_yaw(j); sin_yaw(j)] >= u' * [c0; s0] - 3 + 3*sector(q,j),...
      implies(sector(q,j), (v' * [cos_yaw(j); sin_yaw(j)] - d0) / (d1 - d0) == (yaw(j) - th0) / (th1 - th0)),...
      ];
  end
end

for j = 3:nsteps
  if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.left
    for k = 1:size(sector, 1) - 1
      Constraints = [Constraints,...
       sum(sector(k:k+1,j)) >= sector(k,j-1), ...
      ];
    end
  else
    for k = 2:size(sector, 1)
      Constraints = [Constraints,...
       sum(sector(k-1:k,j)) >= sector(k,j-1), ...
      ];
    end
  end
end


%% Enforce convex regions of terrain

if length(seed_plan.safe_regions) > 1
  region = binvar(length(seed_plan.safe_regions), nsteps, 'full');
  Constraints = [Constraints, ...
    sum(region, 1) == 1,...
    region(1,1:2) == 1];
else
  region = true(1, nsteps);
end

for j = 3:nsteps
  for r = 1:size(region, 1)
    Ar = [seed_plan.safe_regions(r).A(:,1:2), zeros(size(seed_plan.safe_regions(r).A, 1), 1), seed_plan.safe_regions(r).A(:,3)];
    if isa(region(r,j), 'sdpvar')
      Constraints = [Constraints,...
        implies(region(r,j), Ar * x(:,j) <= seed_plan.safe_regions(r).b),...
        implies(region(r,j), seed_plan.safe_regions(r).normal' * x(1:3,j) == seed_plan.safe_regions(r).normal' * seed_plan.safe_regions(r).point),...
        ];
    elseif region(r,j)
      Constraints = [Constraints,...
        Ar * x(:,j) <= seed_plan.safe_regions(r).b,...
        seed_plan.safe_regions(r).normal' * x(1:3,j) == seed_plan.safe_regions(r).normal' * seed_plan.safe_regions(r).point,...
        ];
    end
  end
end

%% Add the objective on distance to the goal
goal_obj = sdpvar(2, nsteps-2, 'full');
Objective = Objective + [0.1 * ones(1,nsteps-3), 10] * sum(goal_obj,1)';
for j = 3:nsteps
  if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.right
    Constraints = [Constraints,...
      coneOrPolyCone(x(1:2,j) - goal_pos.right(1:2), goal_obj(1,j-2), POLYCONE_APPROX_LEVEL),...
      goal_obj(2,j-2) >= [1, 1] * abs(x(3:4,j) - goal_pos.right(POSE_INDICES(3:4))),...
      ];
  elseif seed_plan.footsteps(j).frame_id == biped.foot_frame_id.left
    Constraints = [Constraints,...
      coneOrPolyCone(x(1:2,j) - goal_pos.left(1:2), goal_obj(1,j-2), POLYCONE_APPROX_LEVEL),...
      goal_obj(2,j-2) >= [1, 1] * abs(x(3:4,j) - goal_pos.left(POSE_INDICES(3:4))),...
      ];
  else
    error('Invalid frame ID: %d', seed_plan.footsteps(j).frame_id);
  end
end

%% Add the objective on relative step displacements
rel_obj = sdpvar(2, nsteps-2, 'full');
Objective = Objective + sum(rel_obj(:));
for j = 3:nsteps
  if seed_plan.footsteps(j-1).frame_id == biped.foot_frame_id.right
    nom = [0; seed_plan.params.nom_step_width];
  else
    nom = [0; -seed_plan.params.nom_step_width];
  end
  err = x(:,j) - [(x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * nom); x(3:4,j-1)];
  if j == nsteps
    Constraints = [Constraints,...
      coneOrPolyCone(20 * err(1:2), rel_obj(1,j-2), POLYCONE_APPROX_LEVEL),...
      rel_obj(2,j-2) >= 20 * [1, .25] * abs(err(3:4)),...
      ];
  else
    scale = 0.1 * (nsteps - j) + 1;
    Constraints = [Constraints,...
      coneOrPolyCone(0.5 * scale * err(1:2), rel_obj(1,j-2), POLYCONE_APPROX_LEVEL),...
      coneOrPolyCone(2 * scale * err(1:2), rel_obj(1,j-2) + scale*((2 - 0.5) * seed_plan.params.nom_forward_step), POLYCONE_APPROX_LEVEL),...
      rel_obj(2,j-2) >= [1, .25] * abs(err(3:4))];
  end
end

%% Add the objective on movement of each foot
foot_obj = sdpvar(2, nsteps-2, 'full');
Objective = Objective + sum(foot_obj(:));
for j = 3:nsteps
  err = x(:,j) - x(:,j-2);
  Constraints = [Constraints,...
    coneOrPolyCone(err(1:2), foot_obj(1,j-2), POLYCONE_APPROX_LEVEL),...
    foot_obj(2,j-2) >= [1, .5] * abs(err(3:4)),...
    ];
end

fprintf(1, 'setup: %f\n', toc(t0));
t0 = tic;
diagnostics = solvesdp(Constraints, Objective, sdpsettings('solver', 'gurobi', 'gurobi.MIPGap', 1e-4));
if ~(diagnostics.problem == 0)
  error('Drake:MixedIntegerConvexProgram:InfeasibleProblem', 'The problem is infeasible');
end
solvertime = diagnostics.solvertime;
fprintf(1, 'solve: %f\n', toc(t0));

x = double(x);
cos_yaw = double(cos_yaw);
sin_yaw = double(sin_yaw);
yaw = double(yaw);
% sector = double(sector)


figure(7);
clf
hold on
plot(cos_yaw, sin_yaw, 'bo');
plot(cos(yaw), sin(yaw), 'ro');
th = linspace(0, 2*pi);
plot(cos(th), sin(th), 'k-');
drawnow()

region = double(region);
steps = zeros(6,nsteps);
steps(POSE_INDICES,:) = x;

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

% plan.relative_step_offsets()

% Remove unnecessary footsteps
at_final_pose = false(1, nsteps);
at_final_pose(end-1:end) = true;
for j = 3:nsteps-2
  if mod(nsteps-j, 2)
    at_final_pose(j) = all(abs(plan.footsteps(j).pos - plan.footsteps(end-1).pos) <= TRIM_THRESHOLD);
  else
    at_final_pose(j) = all(abs(plan.footsteps(j).pos - plan.footsteps(end).pos) <= TRIM_THRESHOLD);
  end
end
trim = at_final_pose; % Cut off any steps that are at the final poses
trim(find(trim, 2, 'first')) = false; % Don't cut off the final poses themselves

plan = plan.slice(~trim);



plan.sanity_check();
% plan.relative_step_offsets()

seed = [];

end

function constraint = coneOrPolyCone(x, v, N)
  if N == 0
    constraint = cone(x, v);
  else
    constraint = polycone(x, v, N);
  end
end
