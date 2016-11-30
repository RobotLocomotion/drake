function [plan, solvertime] = relaxedMISOCPGurobi(biped, seed_plan, weights, goal_pos, v_seed)
t0 = tic();
checkDependency('gurobi');
seed_plan.sanity_check();
if nargin < 5
  v_seed = [];
end

SECTOR_WIDTH = pi/8;
POSE_INDICES = [1,2,3,6]; % which indices of xyzrpy we are searching over
MAX_DISTANCE = 30;

TRIM_THRESHOLD = [0.02; 0.02; inf; inf; inf; pi/16];

POLYCONE_APPROX_LEVEL = 0;

REACHABILITY_METHOD = 'ellipse';

if seed_plan.params.min_num_steps > 2
  biped.warning_manager.warnOnce('Drake:footstepRelaxedMISOCPGurobi:MinStepsNotImplemented', 'Min number of steps not implemented for this solver. You can call a different solver like @footstepAlternatingMIQP if this is important for you now.');
end

nsteps = length(seed_plan.footsteps);
seed_steps = [seed_plan.footsteps.pos];

min_yaw = pi * floor(seed_steps(6,1) / pi - 1);
max_yaw = pi * ceil(seed_steps(6,1) / pi + 1);
sector_boundaries = (min_yaw):SECTOR_WIDTH:(max_yaw);

p = MixedIntegerHelper();

x_lb = [-MAX_DISTANCE + repmat(seed_steps(1:3,1), 1, nsteps); 
          repmat(min_yaw, 1, nsteps)];
x_ub = [MAX_DISTANCE + repmat(seed_steps(1:3,1), 1, nsteps);
          repmat(max_yaw, 1, nsteps)];

x_start = seed_steps([1,2,3,6],:);
p = p.addVariable('x', 'C', [4, nsteps], x_lb, x_ub, x_start);
if isempty(v_seed) || ~isstruct(v_seed)
  p = p.addVariable('cos_yaw', 'C', [1, nsteps], -1, 1);
  p = p.addVariable('sin_yaw', 'C', [1, nsteps], -1, 1);
  p = p.addVariable('region', 'B', [length(seed_plan.safe_regions), nsteps], 0, 1);
  p = p.addVariable('sector', 'B', [length(sector_boundaries)-1, nsteps], 0, 1);
  p = p.addVariable('reach_slack', 'C', [3, nsteps-2, 2], -MAX_DISTANCE, MAX_DISTANCE);
  p = p.addVariable('goal_obj', 'C', [3, nsteps-2], 0, inf);
  p = p.addVariable('goal_slack', 'C', [2, nsteps-2], -inf, inf);
  p = p.addVariable('rel_obj', 'C', [3, nsteps-2], 0, inf);
  p = p.addVariable('rel_slack', 'C', [2, nsteps-2, 2], -inf, inf);
  p = p.addVariable('nom_step_slack', 'C', [1, nsteps-2], -inf, inf);
  p = p.addVariable('swing_obj', 'C', [3, nsteps-2], 0, inf);
  p = p.addVariable('swing_slack', 'C', [2, nsteps-2], -inf, inf);
  p = p.addVariable('sincos_bound', 'C', [1, nsteps-2], 1, 1); % fixed to 1
  p = p.addVariable('sincos_slack', 'C', [2, nsteps-2], -1, 1);
  p = p.addVariable('sincos_slack_bound', 'C', [1, nsteps-2], SECTOR_WIDTH,SECTOR_WIDTH);
else
  p = p.addVariable('cos_yaw', 'C', [1, nsteps], -1, 1, v_seed.cos_yaw.value);
  p = p.addVariable('sin_yaw', 'C', [1, nsteps], -1, 1, v_seed.sin_yaw.value);
  p = p.addVariable('region', 'B', [length(seed_plan.safe_regions), nsteps], 0, 1, v_seed.region.value);
  p = p.addVariable('sector', 'B', [length(sector_boundaries)-1, nsteps], 0, 1, v_seed.sector.value);
  p = p.addVariable('reach_slack', 'C', [3, nsteps-2, 2], -MAX_DISTANCE, MAX_DISTANCE, v_seed.reach_slack.value);
  p = p.addVariable('goal_obj', 'C', [3, nsteps-2], 0, inf);
  p = p.addVariable('goal_slack', 'C', [2, nsteps-2], -inf, inf);
  p = p.addVariable('rel_obj', 'C', [3, nsteps-2], 0, inf, v_seed.rel_obj.value);
  p = p.addVariable('rel_slack', 'C', [2, nsteps-2, 2], -inf, inf, v_seed.rel_slack.value);
  p = p.addVariable('nom_step_slack', 'C', [1, nsteps-2], -inf, inf, v_seed.nom_step_slack.value);
  p = p.addVariable('swing_obj', 'C', [3, nsteps-2], 0, inf, v_seed.swing_obj.value);
  p = p.addVariable('swing_slack', 'C', [2, nsteps-2], -inf, inf, v_seed.swing_slack.value);
  p = p.addVariable('sincos_bound', 'C', [1, nsteps-2], 1, 1, v_seed.sincos_bound.value); % fixed to 1
  p = p.addVariable('sincos_slack', 'C', [2, nsteps-2], -1, 1, v_seed.sincos_slack.value);
  p = p.addVariable('sincos_slack_bound', 'C', [1, nsteps-2], SECTOR_WIDTH,SECTOR_WIDTH, v_seed.sincos_slack_bound.value);
end
p.nv
p = p.freeze();

% Constraints = [x(:,1) == seed_steps(POSE_INDICES,1),...
%                x(:,2) == seed_steps(POSE_INDICES,2),...
p.v.x.lb(:,1:2) = seed_steps(POSE_INDICES, 1:2);
p.v.x.ub(:,1:2) = seed_steps(POSE_INDICES, 1:2);

%                cos_yaw(1:2) == cos(seed_steps(6,1:2)),...
p.v.cos_yaw.lb(1:2) = cos(seed_steps(6,1:2));
p.v.cos_yaw.ub(1:2) = cos(seed_steps(6,1:2));
%                sin_yaw(1:2) == sin(seed_steps(6,1:2)),...
p.v.sin_yaw.lb(1:2) = sin(seed_steps(6,1:2));
p.v.sin_yaw.ub(1:2) = sin(seed_steps(6,1:2));

%% Enforce reachability
nfoci = 2;
slack_i = [p.v.reach_slack.i([3,1,2],:,1), p.v.reach_slack.i([3,1,2],:,2)];
p = p.addConesOrPolyConesByIndex(slack_i, POLYCONE_APPROX_LEVEL);
if strcmp(REACHABILITY_METHOD, 'ellipse')
  Aeq_i = zeros(2*nfoci * (nsteps-2), p.nv);
  beq_i = zeros(size(Aeq_i,1), 1);
  Ai = zeros(nsteps-2, p.nv);
  bi = zeros(size(Ai, 1), 1);
  offset = 0;
  expected_offset = size(Aeq_i, 1);
  for j = 3:nsteps
    [foci, l] = biped.getReachabilityEllipse(seed_plan.params, seed_plan.footsteps(j-1).frame_id);
    assert(size(foci, 2) == nfoci, 'I have hard-coded the number of reachability ellipse foci');
    % expr = 0;
    for k = 1:size(foci, 2)
      % Constraints = [Constraints,...
      %   polycone(x(1:2,j) - (x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * foci(:,k)), dists(k), 16),...
      %   ];
      Aeq_i(offset+1, p.v.x.i(1,j)) = 1;
      Aeq_i(offset+1, p.v.x.i(1,j-1)) = -1;
      Aeq_i(offset+1, p.v.cos_yaw.i(j-1)) = -foci(1,k);
      Aeq_i(offset+1, p.v.sin_yaw.i(j-1)) = foci(2,k);
      Aeq_i(offset+1, p.v.reach_slack.i(1,j-2,k)) = -1;
      beq_i(offset+1) = 0;

      Aeq_i(offset+2, p.v.x.i(2,j)) = 1;
      Aeq_i(offset+2, p.v.x.i(2,j-1)) = -1;
      Aeq_i(offset+2, p.v.sin_yaw.i(j-1)) = -foci(1,k);
      Aeq_i(offset+2, p.v.cos_yaw.i(j-1)) = -foci(2,k);
      Aeq_i(offset+2, p.v.reach_slack.i(2,j-2,k)) = -1;
      beq_i(offset+2) = 0;
      offset = offset + 2;

    end
    % Constraints = [Constraints, sum(dists) <= l];
    Ai(j-2, p.v.reach_slack.i(3,j-2,:)) = 1;
    bi(j-2) = l;
  end
  assert(offset == expected_offset, 'setup check failed');
  p = p.addLinearConstraints(Ai, bi, Aeq_i, beq_i);
elseif strcmp(REACHABILITY_METHOD, 'circles')
  ncenters = 2;
  Aeq_i = zeros(3*ncenters*(nsteps-2), p.nv);
  beq_i = zeros(size(Aeq_i, 1), 1);
  offset = 0;
  expected_offset = size(Aeq_i, 1);
  slack_i = [p.v.reach_slack.i([3,1,2],:,1), p.v.reach_slack.i([3,1,2],:,2)];
  p = p.addConesOrPolyConesByIndex(slack_i, POLYCONE_APPROX_LEVEL);
  for j = 3:nsteps
    [centers, radii] = biped.getReachabilityCircles(seed_plan.params, seed_plan.footsteps(j-1).frame_id);
    assert(size(centers, 2) == ncenters, 'I have hard-coded the number of reachability circle centers');
    for k = 1:size(centers, 2)
      % Constraints = [Constraints, cone(x(1:2,j) - (x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * centers(:,k)), radii(k))];
      Aeq_i(offset+1, p.v.x.i(1,j)) = 1;
      Aeq_i(offset+1, p.v.x.i(1,j-1)) = -1;
      Aeq_i(offset+1, p.v.cos_yaw.i(j-1)) = -centers(1,k);
      Aeq_i(offset+1, p.v.sin_yaw.i(j-1)) = centers(2,k);
      Aeq_i(offset+1, p.v.reach_slack.i(1,j-2,k)) = -1;
      beq_i(offset+1) = 0;

      Aeq_i(offset+2, p.v.x.i(2,j)) = 1;
      Aeq_i(offset+2, p.v.x.i(2,j-1)) = -1;
      Aeq_i(offset+2, p.v.sin_yaw.i(j-1)) = -centers(1,k);
      Aeq_i(offset+2, p.v.cos_yaw.i(j-1)) = -centers(2,k);
      Aeq_i(offset+2, p.v.reach_slack.i(2,j-2,k)) = -1;
      beq_i(offset+2) = 0;

      Aeq_i(offset+3, p.v.reach_slack.i(3,j-2,k)) = 1;
      beq_i(offset+3) = radii(k);
      offset = offset+3;
    end
  end
  assert(offset == expected_offset, 'setup check failed');
  p = p.addLinearConstraints([], [], Aeq_i, beq_i);
else
  error('bad reachability method');
end

% Enforce z and yaw reachability
Ai = zeros((nsteps-2) * 4, p.nv);
bi = zeros(size(Ai, 1), 1);
offset = 0;
expected_offset = size(Ai, 1);
for j = 3:nsteps
  % Constraints = [Constraints,...
  %   -seed_plan.params.nom_downward_step <= x(3,j) - x(3,j-1) <= seed_plan.params.nom_upward_step,...
  %   ];
  Ai(offset+1, p.v.x.i(3,j)) = -1;
  Ai(offset+1, p.v.x.i(3,j-1)) = 1;
  bi(offset+1) = seed_plan.params.nom_downward_step;
  Ai(offset+2, p.v.x.i(3,j)) = 1;
  Ai(offset+2, p.v.x.i(3,j-1)) = -1;
  bi(offset+2) = seed_plan.params.nom_upward_step;
  offset = offset+2;

  if seed_plan.footsteps(j-1).frame_id == biped.foot_frame_id.right
    % Constraints = [Constraints,...
    %   -seed_plan.params.max_inward_angle <= x(4,j) - x(4,j-1) <= seed_plan.params.max_outward_angle];
    Ai(offset+1, p.v.x.i(4,j)) = -1;
    Ai(offset+1, p.v.x.i(4,j-1)) = 1;
    bi(offset+1) = seed_plan.params.max_inward_angle;
    Ai(offset+2, p.v.x.i(4,j)) = 1;
    Ai(offset+2, p.v.x.i(4,j-1)) = -1;
    bi(offset+2) = seed_plan.params.max_outward_angle;
    offset = offset+2;
  elseif seed_plan.footsteps(j-1).frame_id == biped.foot_frame_id.left
    % Constraints = [Constraints,...
    %   -seed_plan.params.max_inward_angle <= x(4,j-1) - x(4,j) <= seed_plan.params.max_outward_angle];
    Ai(offset+1, p.v.x.i(4,j)) = 1;
    Ai(offset+1, p.v.x.i(4,j-1)) = -1;
    bi(offset+1) = seed_plan.params.max_inward_angle;
    Ai(offset+2, p.v.x.i(4,j)) = -1;
    Ai(offset+2, p.v.x.i(4,j-1)) = 1;
    bi(offset+2) = seed_plan.params.max_outward_angle;
    offset = offset+2;
  else
    error('invalid frame ID: %d', seed_plan.footsteps(j-1).frame_id);
  end
end
p = p.addLinearConstraints(Ai, bi, [], []);
assert(offset == expected_offset);

%% Enforce rotation convex relaxation
p = p.addConesOrPolyConesByIndex([p.v.sincos_bound.i; p.v.cos_yaw.i(3:nsteps); p.v.sin_yaw.i(3:nsteps)], POLYCONE_APPROX_LEVEL);

Aeq_i = zeros(2*(nsteps-2), p.nv);
beq_i = zeros(size(Aeq_i, 1), 1);
offset = 0;
expected_offset = size(Aeq_i, 1);
p = p.addConesOrPolyConesByIndex([p.v.sincos_slack_bound.i; p.v.sincos_slack.i], 4);
for j = 3:nsteps
  %   polycone([cos_yaw(j) - cos_yaw(j-1); sin_yaw(j) - sin_yaw(j-1)], pi/8, 16)];
  Aeq_i(offset+1, p.v.cos_yaw.i(j)) = 1;
  Aeq_i(offset+1, p.v.cos_yaw.i(j-1)) = -1;
  Aeq_i(offset+1, p.v.sincos_slack.i(1,j-2)) = -1;
  beq_i(offset+1) = 0;

  Aeq_i(offset+2, p.v.sin_yaw.i(j)) = 1;
  Aeq_i(offset+2, p.v.sin_yaw.i(j-1)) = -1;
  Aeq_i(offset+2, p.v.sincos_slack.i(2,j-2)) = -1;
  beq_i(offset+2) = 0;
  offset = offset + 2;
end
assert(offset == expected_offset);

%% Enforce rotation mixed-integer constraints

% Constraints = [Constraints, sum(sector, 1) == 1];
Aeq_i = zeros(nsteps, p.nv);
beq_i = zeros(size(Aeq_i, 1), 1);
offset = 0;
expected_offset = size(Aeq_i, 1);
for j = 1:nsteps
  Aeq_i(offset+1, p.v.sector.i(:,j)) = 1;
  beq_i(offset+1) = 1;
  offset = offset + 1;
end
p = p.addLinearConstraints([], [], Aeq_i, beq_i);
assert(offset == expected_offset);

Ai = zeros((length(sector_boundaries)-1)*(nsteps-2)*5, p.nv);
bi = zeros(size(Ai, 1), 1);
offset = 0;
expected_offset = size(Ai, 1);
for q = 1:length(sector_boundaries)-1
  th0 = sector_boundaries(q);
  th1 = sector_boundaries(q+1);
  c0 = cos(th0);
  s0 = sin(th0);
  c1 = cos(th1);
  s1 = sin(th1);
  v1 = [c1; s1] - [c0; s0];
  v1 = v1 / norm(v1);
  d0 = v1' * [c0; s0];
  d1 = v1' * [c1; s1];
  u = [0, 1; -1, 0] * v1;
  u = u / norm(u);

  for j = 3:nsteps
    %   yaw(j) >= th0 - 2*pi + 2*pi*sector(q,j),...
    % 2*pi*sector(q,j) - yaw(j) <= -th0 + 2*pi
    Ai(offset+1, p.v.x.i(4,j)) = -1;
    Ai(offset+1, p.v.sector.i(q,j)) = 2*pi;
    bi(offset+1) = -th0 + 2*pi;

    %   yaw(j) <= th1 + 2*pi - 2*pi*sector(q,j),...
    % yaw(j) + 2*pi*sector(q,j) <= th1 + 2*pi
    Ai(offset+2, p.v.x.i(4,j)) = 1;
    Ai(offset+2, p.v.sector.i(q,j)) = 2*pi;
    bi(offset+2) = th1 + 2*pi;

    %   u' * [cos_yaw(j); sin_yaw(j)] >= u' * [c0; s0] - 3 + 3*sector(q,j),...
    % 3*sector(q,j) - u' * [cos_yaw(j); sin_yaw(j)] <= -u' * [c0; s0] + 3
    Ai(offset+3, p.v.sector.i(q,j)) = 3;
    Ai(offset+3, p.v.cos_yaw.i(j)) = -u(1);
    Ai(offset+3, p.v.sin_yaw.i(j)) = -u(2);
    bi(offset+3) = -u' * [c0; s0] + 3;


    %   implies(sector(q,j), (v1' * [cos_yaw(j); sin_yaw(j)] - d0) / (d1 - d0) == (yaw(j) - th0) / (th1 - th0)),...
    % 1/(d1-d0) * (v1' * [cos_yaw(j); sin_yaw(j)] - d0) <= (yaw(j) - th0) / (th1-th0) + M*(1-sector(q,j))
    M = 2*pi / SECTOR_WIDTH;
    Ai(offset+4, p.v.cos_yaw.i(j)) = 1/(d1-d0)*v1(1);
    Ai(offset+4, p.v.sin_yaw.i(j)) = 1/(d1-d0)*v1(2);
    Ai(offset+4, p.v.x.i(4,j)) = -1/(th1-th0);
    Ai(offset+4, p.v.sector.i(q,j)) = M;
    bi(offset+4) = 1/(d1-d0)*d0 + -th0/(th1-th0) + M;

    % 1/(d1-d0) * (v1' * [cos_yaw(j); sin_yaw(j)] - d0) >= (yaw(j) - th0) / (th1-th0) - M*(1-sector(q,j))
    Ai(offset+5, p.v.cos_yaw.i(j)) = -1/(d1-d0) * v1(1);
    Ai(offset+5, p.v.sin_yaw.i(j)) = -1/(d1-d0) * v1(2);
    Ai(offset+5, p.v.x.i(4,j)) = 1/(th1-th0);
    Ai(offset+5, p.v.sector.i(q,j)) = M;
    bi(offset+5) = 1/(d1-d0)*(-d0) + th0/(th1-th0) + M;

    offset = offset + 5;

  end
end
assert(offset == expected_offset);
p = p.addLinearConstraints(Ai, bi, [], []);

Ai = zeros((nsteps-2)*(p.v.sector.size(1)-1), p.nv);
bi = zeros(size(Ai, 1), 1);
offset = 0;
expected_offset = size(Ai, 1);
for j = 3:nsteps
  if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.left
    for k = 1:p.v.sector.size(1) - 1
      % Constraints = [Constraints, sum(sector(k:k+1,j)) >= sector(k,j-1)];
      Ai(offset+1, p.v.sector.i(k,j-1)) = 1;
      Ai(offset+1, p.v.sector.i(k:k+1,j)) = -1;
      bi(offset+1) = 0;
      offset = offset + 1;
    end
  else
    for k = 2:p.v.sector.size(1)
      % Constraints = [Constraints, sum(sector(k-1:k,j)) >= sector(k,j-1)];
      Ai(offset+1, p.v.sector.i(k,j-1)) = 1;
      Ai(offset+1, p.v.sector.i(k-1:k,j)) = -1;
      bi(offset+1) = 0;
      offset = offset + 1;
    end
  end
end
assert(offset == expected_offset);
p = p.addLinearConstraints(Ai, bi, [], []);


% %% Enforce convex regions of terrain

% Enforce membership in safe regions
M = MAX_DISTANCE;
Ar = zeros((nsteps-2) * sum(cellfun(@(x) size(x, 1) + 2, {seed_plan.safe_regions.A})), p.nv);
br = zeros(size(Ar, 1), 1);
offset = 0;
expected_offset = size(Ar, 1);
for j = 3:nsteps
  for r = 1:length(seed_plan.safe_regions)
    %     Ar = [seed_plan.safe_regions(r).A(:,1:2), zeros(size(seed_plan.safe_regions(r).A, 1), 1), seed_plan.safe_regions(r).A(:,3)];
    %     Constraints = [Constraints, ...
    %        implies(region(r,j), Ar * x(:,j) <= seed_plan.safe_regions(r).b),...
    %        implies(region(r,j), seed_plan.safe_regions(r).normal' * x(1:3,j) == seed_plan.safe_regions(r).normal' * seed_plan.safe_regions(r).point)];
    A_region = seed_plan.safe_regions(r).A;
    A_region = [A_region(:,1:2), zeros(size(A_region, 1), 1), A_region(:,3)];
    A_region = [A_region;
                reshape(seed_plan.safe_regions(r).normal, 1, []), 0;
                -reshape(seed_plan.safe_regions(r).normal, 1, []), 0];
    b_region = [seed_plan.safe_regions(r).b;
                seed_plan.safe_regions(r).normal' * seed_plan.safe_regions(r).point;
                -seed_plan.safe_regions(r).normal' * seed_plan.safe_regions(r).point];

    Ai = zeros(size(A_region, 1), p.nv);
    Ai(:,p.v.x.i(:,j)) = A_region;
    Ai(:,p.v.region.i(r,j)) = M;
    bi = b_region + M;
    Ar(offset + (1:size(Ai, 1)), :) = Ai;
    br(offset + (1:size(Ai, 1)), :) = bi;
    offset = offset + size(Ai, 1);
  end
end
assert(offset == expected_offset);
p = p.addLinearConstraints(Ar, br, [], []);

% sum(region, 1) == 1
Aeq_i = zeros(nsteps, p.nv);
beq_i = zeros(size(Aeq_i, 1), 1);
offset = 0;
expected_offset = size(Aeq_i, 1);
for j = 1:nsteps
  Aeq_i(offset+1, p.v.region.i(:,j)) = 1;
  beq_i(offset+1) = 1;
  offset = offset + 1;
end
assert(offset == expected_offset);
p = p.addLinearConstraints([], [], Aeq_i, beq_i);

%% Add the objective on distance to the goal
% Objective = Objective + [0.1 * ones(1,nsteps-3), 10] * sum(goal_obj,1)';
p = p.setLinearCostEntries(p.v.goal_obj.i(:,1:end-1), 0.1);
p = p.setLinearCostEntries(p.v.goal_obj.i(:,end), 10);
p = p.addConesOrPolyConesByIndex([p.v.goal_obj.i(1,:); p.v.goal_slack.i], POLYCONE_APPROX_LEVEL);
Aeq_i = zeros(2*(nsteps-2), p.nv);
beq_i = zeros(size(Aeq_i, 1), 1);
offset_eq = 0;
expected_offset_eq = size(Aeq_i, 1);
Ai = zeros(4*(nsteps-2), p.nv);
bi = zeros(size(Ai, 1), 1);
offset_ineq = 0;
expected_offset_ineq = size(Ai, 1);
for j = 3:nsteps
  if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.right
    % Constraints = [Constraints,...
    %   polycone(x(1:2,j) - goal_pos.right(1:2), goal_obj(1,j-2), 16),...
    %   goal_obj(2,j-2) >= [1, 1] * abs(x(3:4,j) - goal_pos.right(POSE_INDICES(3:4))),...
    %   ];
    goal = goal_pos.right;
  elseif seed_plan.footsteps(j).frame_id == biped.foot_frame_id.left
    % Constraints = [Constraints,...
    %   polycone(x(1:2,j) - goal_pos.left(1:2), goal_obj(1,j-2), 16),...
    %   goal_obj(2,j-2) >= [1, 1] * abs(x(3:4,j) - goal_pos.left(POSE_INDICES(3:4))),...
    %   ];
    goal = goal_pos.left;
  else
    error('Invalid frame ID: %d', seed_plan.footsteps(j).frame_id);
  end

  Aeq_i(offset_eq+1, p.v.x.i(1,j)) = 1;
  Aeq_i(offset_eq+1, p.v.goal_slack.i(1,j-2)) = -1;
  beq_i(offset_eq+1) = goal(1);
  Aeq_i(offset_eq+2, p.v.x.i(2,j)) = 1;
  Aeq_i(offset_eq+2, p.v.goal_slack.i(2,j-2)) = -1;
  beq_i(offset_eq+2) = goal(2);
  offset_eq = offset_eq + 2;

  % The entry of -2 for goal_obj reflects the fact that we're actually splitting the weight evenly between z and yaw error
  Ai(offset_ineq+1, p.v.x.i(3,j)) = 1;
  Ai(offset_ineq+1, p.v.goal_obj.i(2,j-2)) = -2;
  bi(offset_ineq+1) = goal(POSE_INDICES(3));
  Ai(offset_ineq+2, p.v.x.i(3,j)) = -1;
  Ai(offset_ineq+2, p.v.goal_obj.i(2,j-2)) = -2;
  bi(offset_ineq+2) = -goal(POSE_INDICES(3));

  Ai(offset_ineq+3, p.v.x.i(4,j)) = 1;
  Ai(offset_ineq+3, p.v.goal_obj.i(3,j-2)) = -2;
  bi(offset_ineq+3) = goal(POSE_INDICES(4));
  Ai(offset_ineq+4, p.v.x.i(4,j)) = -1;
  Ai(offset_ineq+4, p.v.goal_obj.i(3,j-2)) = -2;
  bi(offset_ineq+4) = -goal(POSE_INDICES(4));

  offset_ineq = offset_ineq + 4;
end
p = p.addLinearConstraints(Ai, bi, Aeq_i, beq_i);
assert(offset_eq == expected_offset_eq);
assert(offset_ineq == expected_offset_ineq);

%% Add the objective on relative step displacements
% Objective = Objective + sum(rel_obj(:));
p = p.setLinearCostEntries(p.v.rel_obj.i, 1);
Aeq_i = zeros(4*(nsteps-2)-2, p.nv);
beq_i = zeros(size(Aeq_i, 1), 1);
Ai = zeros(5*(nsteps-2)-1, p.nv);
bi = zeros(size(Ai, 1), 1);
offset_eq = 0;
expected_offset_eq = size(Aeq_i, 1);
offset_ineq = 0;
expected_offset_ineq = size(Ai, 1);
p = p.addConesOrPolyConesByIndex([p.v.rel_obj.i(1,:), p.v.nom_step_slack.i; p.v.rel_slack.i(:,:,1), p.v.rel_slack.i(:,:,2)], POLYCONE_APPROX_LEVEL);
yaw_weight = 0.25;
for j = 3:nsteps
  if seed_plan.footsteps(j-1).frame_id == biped.foot_frame_id.right
    nom = [0; seed_plan.params.nom_step_width];
  else
    nom = [0; -seed_plan.params.nom_step_width];
  end
  % err = x(:,j) - [(x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * nom); x(3:4,j-1)];


  if j == nsteps
    % Constraints = [Constraints,...
    %   polycone(20 * err(1:2), rel_obj(1,j-2), 16),...
    %   rel_obj(2,j-2) >= 20 * [1, 1] * abs(err(3:4)),...
    %   ];
    weight = 20;
    Aeq_i(offset_eq+1, p.v.x.i(1,j)) = weight;
    Aeq_i(offset_eq+1, p.v.x.i(1,j-1)) = -weight;
    Aeq_i(offset_eq+1, p.v.cos_yaw.i(j-1)) = -weight * nom(1);
    Aeq_i(offset_eq+1, p.v.sin_yaw.i(j-1)) = weight * nom(2);
    Aeq_i(offset_eq+1, p.v.rel_slack.i(1,j-2,1)) = -1;

    Aeq_i(offset_eq+2, p.v.x.i(2,j)) = weight;
    Aeq_i(offset_eq+2, p.v.x.i(2,j-1)) = -weight;
    Aeq_i(offset_eq+2, p.v.sin_yaw.i(j-1)) = -weight * nom(1);
    Aeq_i(offset_eq+2, p.v.cos_yaw.i(j-1)) = -weight * nom(2);
    Aeq_i(offset_eq+2, p.v.rel_slack.i(2,j-2,1)) = -1;
    offset_eq = offset_eq + 2;

    Ai(offset_ineq+1, p.v.x.i(3,j)) = weight;
    Ai(offset_ineq+1, p.v.x.i(3,j-1)) = -weight;
    Ai(offset_ineq+1, p.v.rel_obj.i(2,j-2)) = -2;

    Ai(offset_ineq+2, p.v.x.i(3,j)) = -weight;
    Ai(offset_ineq+2, p.v.x.i(3,j-1)) = weight;
    Ai(offset_ineq+2, p.v.rel_obj.i(2,j-2)) = -2;

    Ai(offset_ineq+3, p.v.x.i(4,j)) = yaw_weight * weight;
    Ai(offset_ineq+3, p.v.x.i(4,j-1)) = -yaw_weight * weight;
    Ai(offset_ineq+3, p.v.rel_obj.i(3,j-2)) = -2;

    Ai(offset_ineq+4, p.v.x.i(4,j)) = -yaw_weight * weight;
    Ai(offset_ineq+4, p.v.x.i(4,j-1)) = yaw_weight * weight;
    Ai(offset_ineq+4, p.v.rel_obj.i(3,j-2)) = -2;

    offset_ineq = offset_ineq + 4;
  else
    scale = 0.1 * (nsteps - j) + 1;
    % Constraints = [Constraints,...
    %   polycone(0.5 * scale * err(1:2), rel_obj(1,j-2), 16),...
    %   polycone(2 * scale * err(1:2), rel_obj(1,j-2) + scale*((2 - 0.5) * seed_plan.params.nom_forward_step), 16),...
    %   rel_obj(2,j-2) >= [1, 1] * abs(err(3:4))];

    alpha1 = 0.5;
    alpha2 = 2;
    weight = alpha1 * scale;
    Aeq_i(offset_eq+1, p.v.x.i(1,j)) = weight;
    Aeq_i(offset_eq+1, p.v.x.i(1,j-1)) = -weight;
    Aeq_i(offset_eq+1, p.v.cos_yaw.i(j-1)) = -weight * nom(1);
    Aeq_i(offset_eq+1, p.v.sin_yaw.i(j-1)) = weight * nom(2);
    Aeq_i(offset_eq+1, p.v.rel_slack.i(1,j-2,1)) = -1;
    beq_i(offset_eq+1) = 0;

    Aeq_i(offset_eq+2, p.v.x.i(2,j)) = weight;
    Aeq_i(offset_eq+2, p.v.x.i(2,j-1)) = -weight;
    Aeq_i(offset_eq+2, p.v.sin_yaw.i(j-1)) = -weight * nom(1);
    Aeq_i(offset_eq+2, p.v.cos_yaw.i(j-1)) = -weight * nom(2);
    Aeq_i(offset_eq+2, p.v.rel_slack.i(2,j-2,1)) = -1;
    offset_eq = offset_eq + 2;

    weight = alpha2 * scale;
    Aeq_i(offset_eq+1, p.v.x.i(1,j)) = weight;
    Aeq_i(offset_eq+1, p.v.x.i(1,j-1)) = -weight;
    Aeq_i(offset_eq+1, p.v.cos_yaw.i(j-1)) = -weight * nom(1);
    Aeq_i(offset_eq+1, p.v.sin_yaw.i(j-1)) = weight * nom(2);
    Aeq_i(offset_eq+1, p.v.rel_slack.i(1,j-2,2)) = -1;

    Aeq_i(offset_eq+2, p.v.x.i(2,j)) = weight;
    Aeq_i(offset_eq+2, p.v.x.i(2,j-1)) = -weight;
    Aeq_i(offset_eq+2, p.v.sin_yaw.i(j-1)) = -weight * nom(1);
    Aeq_i(offset_eq+2, p.v.cos_yaw.i(j-1)) = -weight * nom(2);
    Aeq_i(offset_eq+2, p.v.rel_slack.i(2,j-2,2)) = -1;
    offset_eq = offset_eq + 2;

    % rel_obj(1,j-2) >= nom_step_slack(1,j-2) - scale*(2-0.5) * nom_forward_step
    Ai(offset_ineq+1, p.v.rel_obj.i(1,j-2)) = -1;
    Ai(offset_ineq+1, p.v.nom_step_slack.i(j-2)) = 1;
    bi(offset_ineq+1) = scale * (alpha2 - alpha1) * seed_plan.params.nom_forward_step;
    offset_ineq = offset_ineq + 1;

    weight = 1;

    Ai(offset_ineq+1, p.v.x.i(3,j)) = weight;
    Ai(offset_ineq+1, p.v.x.i(3,j-1)) = -weight;
    Ai(offset_ineq+1, p.v.rel_obj.i(2,j-2)) = -2;

    Ai(offset_ineq+2, p.v.x.i(3,j)) = -weight;
    Ai(offset_ineq+2, p.v.x.i(3,j-1)) = weight;
    Ai(offset_ineq+2, p.v.rel_obj.i(2,j-2)) = -2;

    Ai(offset_ineq+3, p.v.x.i(4,j)) = yaw_weight * weight;
    Ai(offset_ineq+3, p.v.x.i(4,j-1)) = -yaw_weight * weight;
    Ai(offset_ineq+3, p.v.rel_obj.i(3,j-2)) = -2;

    Ai(offset_ineq+4, p.v.x.i(4,j)) = -yaw_weight * weight;
    Ai(offset_ineq+4, p.v.x.i(4,j-1)) = yaw_weight * weight;
    Ai(offset_ineq+4, p.v.rel_obj.i(3,j-2)) = -2;

    offset_ineq = offset_ineq + 4;
  end
end
assert(offset_eq == expected_offset_eq)
assert(offset_ineq == expected_offset_ineq)
p = p.addLinearConstraints(Ai, bi, Aeq_i, beq_i);

%% Add the objective on movement of each foot
Aeq_i = zeros(2*(nsteps-2), p.nv);
beq_i = zeros(size(Aeq_i, 1), 1);
offset_eq = 0;
expected_offset_eq = size(Aeq_i, 1);
Ai = zeros(4*(nsteps-2), p.nv);
bi = zeros(size(Ai, 1), 1);
offset_ineq = 0;
expected_offset_ineq = size(Ai, 1);
% Objective = Objective + sum(foot_obj(:));
p = p.setLinearCostEntries(p.v.swing_obj.i, 1);
p = p.addConesOrPolyConesByIndex([p.v.swing_obj.i(1,:); p.v.swing_slack.i], POLYCONE_APPROX_LEVEL);
for j = 3:nsteps
  % err = x(:,j) - x(:,j-2);
  % Constraints = [Constraints,...
  %   polycone(err(1:2), foot_obj(1,j-2), 16),...
  %   foot_obj(2,j-2) >= [1, .5] * abs(err(3:4)),...
  %   ];
  Aeq_i(offset_eq+1, p.v.x.i(1,j)) = 1;
  Aeq_i(offset_eq+1, p.v.x.i(1,j-2)) = -1;
  Aeq_i(offset_eq+1, p.v.swing_slack.i(1,j-2)) = -1;

  Aeq_i(offset_eq+2, p.v.x.i(2,j)) = 1;
  Aeq_i(offset_eq+2, p.v.x.i(2,j-2)) = -1;
  Aeq_i(offset_eq+2, p.v.swing_slack.i(2,j-2)) = -1;

  offset_eq = offset_eq + 2;


  Ai(offset_ineq+1, p.v.x.i(3,j)) = 1;
  Ai(offset_ineq+1, p.v.x.i(3,j-2)) = -1;
  Ai(offset_ineq+1, p.v.swing_obj.i(2,j-2)) = -2;

  Ai(offset_ineq+2, p.v.x.i(3,j)) = -1;
  Ai(offset_ineq+2, p.v.x.i(3,j-2)) = 1;
  Ai(offset_ineq+2, p.v.swing_obj.i(2,j-2)) = -2;

  Ai(offset_ineq+3, p.v.x.i(4,j)) = yaw_weight;
  Ai(offset_ineq+3, p.v.x.i(4,j-2)) = -yaw_weight;
  Ai(offset_ineq+3, p.v.swing_obj.i(3,j-2)) = -2;

  Ai(offset_ineq+4, p.v.x.i(4,j)) = -yaw_weight;
  Ai(offset_ineq+4, p.v.x.i(4,j-2)) = yaw_weight;
  Ai(offset_ineq+4, p.v.swing_obj.i(3,j-2)) = -2;

  offset_ineq = offset_ineq + 4;

end
assert(offset_eq == expected_offset_eq)
assert(offset_ineq == expected_offset_ineq)
p = p.addLinearConstraints(Ai, bi, Aeq_i, beq_i);

params = struct();
params.mipgap = 1e-4;
params.outputflag = 1;
% params.ConcurrentMIP = 4;
% params.NodeMethod = 0;

fprintf(1, 'setup: %f\n', toc(t0));

% Solve the problem
t0 = tic();
[p, solvertime] = p.solveGurobi(params);
fprintf(1, 'solve: %f\n', toc(t0));

t0 = tic();
steps = p.v.x.value;
steps = [steps(1:3,:); zeros(2, size(steps, 2)); steps(4,:)];

plan = seed_plan;
for j = 1:nsteps
  plan.footsteps(j).pos = steps(:,j);
end

figure(7);
clf
hold on
plot(p.v.cos_yaw.value, p.v.sin_yaw.value, 'bo');
plot(cos(p.v.x.value(4,:)), sin(p.v.x.value(4,:)), 'ro');
th = linspace(0, 2*pi);
plot(cos(th), sin(th), 'k-');
drawnow()

region = p.v.region.value;

region_order = nan(1, size(region, 2));
for j = 1:length(region_order)
  i = find(region(:,j));
  if ~isempty(i)
    region_order(j) = i;
  end
end
assert(length(region_order) == size(region, 2));
plan.region_order = region_order;

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

v = p.v;
fprintf(1, 'extract: %f\n', toc(t0));


end

