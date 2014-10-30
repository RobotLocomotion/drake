function [plan, v] = footstepRelaxedMISOCPGurobi(biped, seed_plan, weights, goal_pos, v_seed)

checkDependency('gurobi');
seed_plan.sanity_check();
if nargin < 5
  v_seed = [];
end

SECTOR_WIDTH = pi/8;
POSE_INDICES = [1,2,3,6]; % which indices of xyzrpy we are searching over
MAX_DISTANCE = 30;

TRIM_THRESHOLD = [0.02; 0.02; 0.02; pi/16; pi/16; pi/16];

REACHABILITY_METHOD = 'ellipse';

if seed_plan.params.min_num_steps > 2
  biped.warning_manager.warnOnce('Drake:footstepRelaxedMISOCPGurobi:MinStepsNotImplemented', 'Min number of steps not implemented for this solver. You can call a different solver like @footstepAlternatingMIQP if this is important for you now.');
end

nsteps = length(seed_plan.footsteps);
seed_steps = [seed_plan.footsteps.pos];

min_yaw = pi * floor(seed_steps(6,1) / pi - 1);
max_yaw = pi * ceil(seed_steps(6,1) / pi + 1);
sector_boundaries = (min_yaw - SECTOR_WIDTH/2):SECTOR_WIDTH:(max_yaw + SECTOR_WIDTH/2);

% Build a struct to hold the sizes and indices of our decision variables
% This is a new approach that I'm experimenting with, which should offer
% a mix of the advantages of symbolic and matrix-based optimization
% frameworks. The idea is that we have a single Matlab struct (named just
% 'v' for convenience) and each variable in the optimization has a
% corresponding named field in v. For each variable, we have subfields as
% follows:
% type: 'B', 'I', or 'C' for binary, integer, or continuous variables
% size: a 2x1 vector describing the shape of the variable
% i: the indices corresponding to the variable, as a matrix of the same size as 
% the 'size' field above. 
% lb: lower bound, as a matrix of the same size as 'size'
% ub: upper bound, a matrix
% start:  the initial values as a matrix of the same size as 'size'
%
% After optimization, there will be an additional field added to v, called
% 'value', which will contain the final values after optimization.
% 
% The 'i' field of indices is useful because when
% we actually set up the problem in gurobi or another solver, all of the
% optimization variables are combined into one long vector. This index
% field lets us easily address parts of that vector. For example, to set
% the entry in a constraint matrix A corresponding to the jth row and kth column 
% of variable 'foo' to 1, we can do the following:
% A(1, v.foo.i(j,k)) = 1;
nv = 0;
v = struct();

function add_var(name, type_, size_, lb, ub, start_)
  v.(name) = struct();
  v.(name).type = type_;
  v.(name).size = size_;
  v.(name).i = reshape(nv + (1:prod(v.(name).size)), v.(name).size);
  nv = nv + v.(name).i(end);
  if isscalar(lb)
    lb = repmat(lb, v.(name).size);
  end
  if isscalar(ub)
    ub = repmat(ub, v.(name).size);
  end
  v.(name).lb = lb;
  v.(name).ub = ub;
  if nargin < 6
    start_ = [];
  end
  v.(name).start = nan(v.(name).size);
  v.(name).start(1:size(start_, 1), 1:size(start_, 2)) = start_;
end

x_lb = [-MAX_DISTANCE + repmat(seed_steps(1:3,1), 1, nsteps); 
          repmat(min_yaw, 1, nsteps)];
x_ub = [MAX_DISTANCE + repmat(seed_steps(1:3,1), 1, nsteps);
          repmat(max_yaw, 1, nsteps)];

x_start = seed_steps([1,2,3,6],:);
add_var('x', 'C', [4, nsteps], x_lb, x_ub, x_start);
if isempty(v_seed)
  add_var('cos_yaw', 'C', [1, nsteps], -1, 1);
  add_var('sin_yaw', 'C', [1, nsteps], -1, 1);
  add_var('region', 'B', [length(seed_plan.safe_regions), nsteps], 0, 1);
  add_var('sector', 'B', [length(sector_boundaries)-1, nsteps], 0, 1);
  add_var('reach_slack', 'C', [3, nsteps-2, 2], 0, MAX_DISTANCE);
  % add_var('trig_slack', 'C', [2, nsteps-2], -1, 1);
else
  add_var('cos_yaw', 'C', [1, nsteps], -1, 1, v_seed.cos_yaw.value);
  add_var('sin_yaw', 'C', [1, nsteps], -1, 1, v_seed.sin_yaw.value);
  add_var('region', 'B', [length(seed_plan.safe_regions), nsteps], 0, 1, v_seed.region.value);
  add_var('sector', 'B', [length(sector_boundaries)-1, nsteps], 0, 1, v_seed.sector.value);
  add_var('reach_slack', 'C', [3, nsteps-2, 2], 0, MAX_DISTANCE, v_seed.reach_slack.value);
  % add_var('trig_slack', 'C', [2, nsteps-2], -1, 1, v_seed.trig_slack.value);
end

c = zeros(nv, 1);
Q = sparse(nv, nv);
A = zeros(0, nv);
b = zeros(0, 1);
Aeq = zeros(0, nv);
beq = zeros(0, 1);
quadcon = struct('Qc', {}, 'q', {}, 'rhs', {});
objcon = 0;
cones = struct('index', {});

% Constraints = [x(:,1) == seed_steps(POSE_INDICES,1),...
%                x(:,2) == seed_steps(POSE_INDICES,2),...
v.x.lb(:,1:2) = seed_steps(POSE_INDICES, 1:2);
v.x.ub(:,1:2) = seed_steps(POSE_INDICES, 1:2);

%                cos_yaw(1:2) == cos(seed_steps(6,1:2)),...
v.cos_yaw.lb(1:2) = cos(seed_steps(6,1:2));
v.cos_yaw.ub(1:2) = cos(seed_steps(6,1:2));
%                sin_yaw(1:2) == sin(seed_steps(6,1:2)),...
v.sin_yaw.lb(1:2) = sin(seed_steps(6,1:2));
v.sin_yaw.ub(1:2) = sin(seed_steps(6,1:2));

%% Enforce reachability
nfoci = 2;
slack_i = reshape(v.reach_slack.i([3,1,2],:,:), 3, nfoci*(nsteps-2));
cones = [cones, struct('index', mat2cell(slack_i, 3, ones(1, size(slack_i,2))))];
if strcmp(REACHABILITY_METHOD, 'ellipse')
  Aeq_i = zeros(2*nfoci * (nsteps-2), nv);
  beq_i = zeros(size(Aeq_i,1), 1);
  Ai = zeros(nsteps-2, nv);
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
      Aeq_i(offset+1, v.x.i(1,j)) = 1;
      Aeq_i(offset+1, v.x.i(1,j-1)) = -1;
      Aeq_i(offset+1, v.cos_yaw.i(j-1)) = -foci(1,k);
      Aeq_i(offset+1, v.sin_yaw.i(j-1)) = foci(2,k);
      Aeq_i(offset+1, v.reach_slack.i(1,j-2,k)) = -1;
      beq_i(offset+1) = 0;

      Aeq_i(offset+2, v.x.i(2,j)) = 1;
      Aeq_i(offset+2, v.x.i(2,j-1)) = -1;
      Aeq_i(offset+2, v.sin_yaw.i(j-1)) = -foci(1,k);
      Aeq_i(offset+2, v.cos_yaw.i(j-1)) = -foci(2,k);
      Aeq_i(offset+2, v.reach_slack.i(2,j-2,k)) = -1;
      beq_i(offset+2) = 0;
      offset = offset + 2;

      Ai(j-2, v.reach_slack.i(3,j-2,k)) = 1;
    end
    % Constraints = [Constraints, sum(dists) <= l];
    bi(j-2) = l;
  end
  assert(offset == expected_offset, 'setup check failed');
  Aeq = [Aeq; Aeq_i];
  beq = [beq; beq_i];
  A = [A; Ai];
  b = [b; bi];
elseif strcmp(REACHABILITY_METHOD, 'circles')
  ncenters = 2;
  Aeq_i = zeros(3*ncenters*(nsteps-2), nv);
  beq_i = zeros(size(Aeq_i, 1), 1);
  offset = 0;
  expected_offset = size(Aeq_i, 1);
  slack_i = reshape(v.reach_slack.i([3,1,2],:,:), 3, ncenters*(nsteps-2));
  cones = [cones, struct('index', mat2cell(slack_i, 3, ones(1, size(slack_i,2))))];
  for j = 3:nsteps
    [centers, radii] = biped.getReachabilityCircles(seed_plan.params, seed_plan.footsteps(j-1).frame_id);
    assert(size(centers, 2) == ncenters, 'I have hard-coded the number of reachability circle centers');
    for k = 1:size(centers, 2)
      % Constraints = [Constraints, cone(x(1:2,j) - (x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * centers(:,k)), radii(k))];
      Aeq_i(offset+1, v.x.i(1,j)) = 1;
      Aeq_i(offset+1, v.x.i(1,j-1)) = -1;
      Aeq_i(offset+1, v.cos_yaw.i(j-1)) = -centers(1,k);
      Aeq_i(offset+1, v.sin_yaw.i(j-1)) = centers(2,k);
      Aeq_i(offset+1, v.reach_slack.i(1,j-2,k)) = -1;
      beq_i(offset+1) = 0;

      Aeq_i(offset+2, v.x.i(2,j)) = 1;
      Aeq_i(offset+2, v.x.i(2,j-1)) = -1;
      Aeq_i(offset+2, v.sin_yaw.i(j-1)) = -centers(1,k);
      Aeq_i(offset+2, v.cos_yaw.i(j-1)) = -centers(2,k);
      Aeq_i(offset+2, v.reach_slack.i(2,j-2,k)) = -1;
      beq_i(offset+2) = 0;

      Aeq_i(offset+3, v.reach_slack.i(3,j-2,k)) = 1;
      beq_i(offset+3) = radii(k);
      offset = offset+3;
    end
  end
  assert(offset == expected_offset, 'setup check failed');
  Aeq = [Aeq; Aeq_i];
  beq = [beq; beq_i];
else
  error('bad reachability method');
end

% Enforce z and yaw reachability
Ai = zeros((nsteps-2) * 4, nv);
bi = zeros(size(Ai, 1), 1);
offset = 0;
expected_offset = size(Ai, 1);
for j = 3:nsteps
  % Constraints = [Constraints,...
  %   -seed_plan.params.nom_downward_step <= x(3,j) - x(3,j-1) <= seed_plan.params.nom_upward_step,...
  %   ];
  Ai(offset+1, v.x.i(3,j)) = -1;
  Ai(offset+1, v.x.i(3,j-1)) = 1;
  bi(offset+1) = seed_plan.params.nom_downward_step;
  Ai(offset+2, v.x.i(3,j)) = 1;
  Ai(offset+2, v.x.i(3,j-1)) = -1;
  bi(offset+2) = seed_plan.params.nom_upward_step;
  offset = offset+2;

  if seed_plan.footsteps(j-1).frame_id == biped.foot_frame_id.right
    % Constraints = [Constraints,...
    %   -seed_plan.params.max_inward_angle <= x(4,j) - x(4,j-1) <= seed_plan.params.max_outward_angle];
    Ai(offset+1, v.x.i(4,j)) = -1;
    Ai(offset+1, v.x.i(4,j-1)) = 1;
    bi(offset+1) = seed_plan.params.max_inward_angle;
    Ai(offset+2, v.x.i(4,j)) = 1;
    Ai(offset+2, v.x.i(4,j-1)) = -1;
    bi(offset+2) = seed_plan.params.max_outward_angle;
    offset = offset+2;
  elseif seed_plan.footsteps(j-1).frame_id == biped.foot_frame_id.left
    % Constraints = [Constraints,...
    %   -seed_plan.params.max_inward_angle <= x(4,j-1) - x(4,j) <= seed_plan.params.max_outward_angle];
    Ai(offset+1, v.x.i(4,j)) = 1;
    Ai(offset+1, v.x.i(4,j-1)) = -1;
    bi(offset+1) = seed_plan.params.max_inward_angle;
    Ai(offset+2, v.x.i(4,j)) = -1;
    Ai(offset+2, v.x.i(4,j-1)) = 1;
    bi(offset+2) = seed_plan.params.max_outward_angle;
    offset = offset+2;
  else
    error('invalid frame ID: %d', seed_plan.footsteps(j-1).frame_id);
  end
end
A = [A; Ai];
b = [b; bi];
assert(offset == expected_offset);

%% Enforce rotation convex relaxation
offset = 0;
expected_offset = nsteps-2;
quadcon_i = struct('Qc', cell(1,nsteps-2), 'q', cell(1,nsteps-2), 'rhs', cell(1,nsteps-2));
for j = 3:nsteps
  Qc = sparse([v.cos_yaw.i(j), v.sin_yaw.i(j)], [v.cos_yaw.i(j), v.sin_yaw.i(j)], [1, 1], nv, nv);
  qc = zeros(nv, 1);
  beta = 1;
  quadcon_i(offset+1) = struct('Qc', Qc, 'q', qc, 'rhs', beta);
  offset = offset + 1;
  % Constraints = [Constraints,...
  %   polycone([cos_yaw(j); sin_yaw(j)], 1, 16)];
end
quadcon = [quadcon, quadcon_i];
assert(offset == expected_offset);

% offset = 0;
% expected_offset = nsteps-2;
% quadcon_i = struct('Qc', cell(1,nsteps-2), 'q', cell(1,nsteps-2), 'rhs', cell(1,nsteps-2));
% for j = 3:nsteps
%   % Constraints = [Constraints,...
%   %   polycone([cos_yaw(j) - cos_yaw(j-1); sin_yaw(j) - sin_yaw(j-1)], pi/8, 16)];
%   r = [v.cos_yaw.i(j), 
%   c = [v.cos_yaw.i(j)
%   s = [1,
%   Qc = sparse([
% end

%% Enforce rotation mixed-integer constraints
sector_boundaries = (min_yaw - SECTOR_WIDTH/2):SECTOR_WIDTH:(max_yaw + SECTOR_WIDTH/2);

% Constraints = [Constraints, sum(sector, 1) == 1];
Aeq_i = zeros(nsteps, nv);
beq_i = zeros(size(Aeq_i, 1), 1);
offset = 0;
expected_offset = size(Aeq_i, 1);
for j = 1:nsteps
  Aeq_i(offset+1, v.sector.i(:,j)) = 1;
  beq_i(offset+1) = 1;
  offset = offset + 1;
end
Aeq = [Aeq; Aeq_i];
beq = [beq; beq_i];
assert(offset == expected_offset);

Ai = zeros((length(sector_boundaries)-1)*(nsteps-2)*5, nv);
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
    Ai(offset+1, v.x.i(4,j)) = -1;
    Ai(offset+1, v.sector.i(q,j)) = 2*pi;
    bi(offset+1) = -th0 + 2*pi;

    %   yaw(j) <= th1 + 2*pi - 2*pi*sector(q,j),...
    % yaw(j) + 2*pi*sector(q,j) <= th1 + 2*pi
    Ai(offset+2, v.x.i(4,j)) = 1;
    Ai(offset+2, v.sector.i(q,j)) = 2*pi;
    bi(offset+2) = th1 + 2*pi;

    %   u' * [cos_yaw(j); sin_yaw(j)] >= u' * [c0; s0] - 3 + 3*sector(q,j),...
    % 3*sector(q,j) - u' * [cos_yaw(j); sin_yaw(j)] <= -u' * [c0; s0] + 3
    Ai(offset+3, v.sector.i(q,j)) = 3;
    Ai(offset+3, v.cos_yaw.i(j)) = -u(1);
    Ai(offset+3, v.sin_yaw.i(j)) = -u(2);
    bi(offset+3) = -u' * [c0; s0] + 3;


    %   implies(sector(q,j), (v1' * [cos_yaw(j); sin_yaw(j)] - d0) / (d1 - d0) == (yaw(j) - th0) / (th1 - th0)),...
    % 1/(d1-d0) * (v1' * [cos_yaw(j); sin_yaw(j)] - d0) <= (yaw(j) - th0) / (th1-th0) + M*(1-sector(q,j))
    M = 2*pi / SECTOR_WIDTH;
    Ai(offset+4, v.cos_yaw.i(j)) = 1/(d1-d0)*v1(1);
    Ai(offset+4, v.sin_yaw.i(j)) = 1/(d1-d0)*v1(2);
    Ai(offset+4, v.x.i(4,j)) = -1/(th1-th0);
    Ai(offset+4, v.sector.i(q,j)) = M;
    bi(offset+4) = 1/(d1-d0)*d0 + -th0/(th1-th0) + M;

    % 1/(d1-d0) * (v1' * [cos_yaw(j); sin_yaw(j)] - d0) >= (yaw(j) - th0) / (th1-th0) - M*(1-sector(q,j))
    Ai(offset+5, v.cos_yaw.i(j)) = -1/(d1-d0) * v1(1);
    Ai(offset+5, v.sin_yaw.i(j)) = -1/(d1-d0) * v1(2);
    Ai(offset+5, v.x.i(4,j)) = 1/(th1-th0);
    Ai(offset+5, v.sector.i(q,j)) = M;
    bi(offset+5) = 1/(d1-d0)*(-d0) + th0/(th1-th0) + M;

    offset = offset + 5;

  end
end
assert(offset == expected_offset);
A = [A; Ai];
b = [b; bi];

Ai = zeros((nsteps-2)*(v.sector.size(1)-1), nv);
bi = zeros(size(Ai, 1), 1);
offset = 0;
expected_offset = size(Ai, 1);
for j = 3:nsteps
  if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.left
    for k = 1:v.sector.size(1) - 1
      % Constraints = [Constraints, sum(sector(k:k+1,j)) >= sector(k,j-1)];
      Ai(offset+1, v.sector.i(k,j-1)) = 1;
      Ai(offset+1, v.sector.i(k:k+1,j)) = -1;
      bi(offset+1) = 0;
      offset = offset + 1;
    end
  else
    for k = 2:v.sector.size(1)
      % Constraints = [Constraints, sum(sector(k-1:k,j)) >= sector(k,j-1)];
      Ai(offset+1, v.sector.i(k,j-1)) = 1;
      Ai(offset+1, v.sector.i(k-1:k,j)) = -1;
      bi(offset+1) = 0;
      offset = offset + 1;
    end
  end
end
assert(offset == expected_offset);
A = [A; Ai];
b = [b; bi];


% %% Enforce convex regions of terrain

% Enforce membership in safe regions
M = MAX_DISTANCE;
Ar = zeros((nsteps-2) * sum(cellfun(@(x) size(x, 1) + 2, {seed_plan.safe_regions.A})), nv);
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

    Ai = zeros(size(A_region, 1), nv);
    Ai(:,v.x.i(:,j)) = A_region;
    Ai(:,v.region.i(r,j)) = M;
    bi = b_region + M;
    Ar(offset + (1:size(Ai, 1)), :) = Ai;
    br(offset + (1:size(Ai, 1)), :) = bi;
    offset = offset + size(Ai, 1);
  end
end
assert(offset == expected_offset);
A = [A; Ar];
b = [b; br];

% sum(region, 1) == 1
Aeq_i = zeros(nsteps, nv);
beq_i = zeros(size(Aeq_i, 1), 1);
offset = 0;
expected_offset = size(Aeq_i, 1);
for j = 1:nsteps
  Aeq_i(offset+1, v.region.i(:,j)) = 1;
  beq_i(offset+1) = 1;
  offset = offset + 1;
end
assert(offset == expected_offset);
Aeq = [Aeq; Aeq_i];
beq = [beq; beq_i];

% %% Add the objective on distance to the goal
% goal_obj = sdpvar(2, nsteps-2, 'full');
% Objective = Objective + [0.1 * ones(1,nsteps-3), 10] * sum(goal_obj,1)';
% for j = 3:nsteps
%   if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.right
%     Constraints = [Constraints,...
%       polycone(x(1:2,j) - goal_pos.right(1:2), goal_obj(1,j-2), 16),...
%       goal_obj(2,j-2) >= [1, 1] * abs(x(3:4,j) - goal_pos.right(POSE_INDICES(3:4))),...
%       ];
%   elseif seed_plan.footsteps(j).frame_id == biped.foot_frame_id.left
%     Constraints = [Constraints,...
%       polycone(x(1:2,j) - goal_pos.left(1:2), goal_obj(1,j-2), 16),...
%       goal_obj(2,j-2) >= [1, 1] * abs(x(3:4,j) - goal_pos.left(POSE_INDICES(3:4))),...
%       ];
%   else
%     error('Invalid frame ID: %d', seed_plan.footsteps(j).frame_id);
%   end
% end

% %% Add the objective on relative step displacements
% rel_obj = sdpvar(2, nsteps-2, 'full');
% Objective = Objective + sum(rel_obj(:));
% for j = 3:nsteps
%   if seed_plan.footsteps(j-1).frame_id == biped.foot_frame_id.right
%     nom = [0; seed_plan.params.nom_step_width];
%   else
%     nom = [0; -seed_plan.params.nom_step_width];
%   end
%   err = x(:,j) - [(x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * nom); x(3:4,j-1)];
%   if j == nsteps
%     Constraints = [Constraints,...
%       polycone(20 * err(1:2), rel_obj(1,j-2), 16),...
%       rel_obj(2,j-2) >= 20 * [1, 1] * abs(err(3:4)),...
%       ];
%   else
%     scale = 0.1 * (nsteps - j) + 1;
%     Constraints = [Constraints,...
%       polycone(0.5 * scale * err(1:2), rel_obj(1,j-2), 16),...
%       polycone(2 * scale * err(1:2), rel_obj(1,j-2) + scale*((2 - 0.5) * seed_plan.params.nom_forward_step), 16),...
%       rel_obj(2,j-2) >= [1, 1] * abs(err(3:4))];
%   end
% end

% %% Add the objective on movement of each foot
% foot_obj = sdpvar(2, nsteps-2, 'full');
% Objective = Objective + sum(foot_obj(:));
% for j = 3:nsteps
%   err = x(:,j) - x(:,j-2);
%   Constraints = [Constraints,...
%     polycone(err(1:2), foot_obj(1,j-2), 16),...
%     foot_obj(2,j-2) >= [1, .5] * abs(err(3:4)),...
%     ];
% end

% fprintf(1, 'setup: %f\n', toc(t0));
% t0 = tic;
% solvesdp(Constraints, Objective, sdpsettings('solver', 'gurobi'));
% fprintf(1, 'solve: %f\n', toc(t0));

% x = double(x);
% cos_yaw = double(cos_yaw);
% sin_yaw = double(sin_yaw);
% yaw = double(yaw)
% % sector = double(sector)


% figure(7);
% clf
% hold on
% plot(cos_yaw, sin_yaw, 'bo');
% plot(cos(yaw), sin(yaw), 'ro');
% th = linspace(0, 2*pi);
% plot(cos(th), sin(th), 'k-');
% drawnow()

% region = double(region);
% steps = zeros(6,nsteps);
% steps(POSE_INDICES,:) = x;

var_names = fieldnames(v);
clear model params
model.A = sparse([A; Aeq]);
model.rhs = [b; beq];
model.sense = [repmat('<', size(A,1), 1); repmat('=', size(Aeq, 1), 1)];
model.start = nan(nv, 1);
model.obj = c;
model.Q = Q;
if ~isempty(quadcon)
  model.quadcon = quadcon;
end
model.objcon = objcon;
if ~isempty(cones)
  model.cones = cones;
end

% Set up defaults so we can fill them in from v
model.vtype = repmat('C', nv, 1);
model.lb = -inf(nv, 1);
model.ub = inf(nv, 1);
for j = 1:length(var_names)
  name = var_names{j};
  i = reshape(v.(name).i, [], 1);
  model.vtype(i) = v.(name).type;
  model.lb(i) = reshape(v.(name).lb, [], 1);
  model.ub(i) = reshape(v.(name).ub, [], 1);
  model.start(i) = reshape(v.(name).start, [], 1);
end

params.mipgap = 1e-4;
params.outputflag = 1;
params.ConcurrentMIP = 4;
params.NodeMethod = 0;
% params.Cuts = 3;
% params.Heuristics = 0.2;
% params.Presolve = 2;
% params.MIQCPMethod = 1;
% params.MIPFocus = 2;

% Solve the problem
result = gurobi(model, params);

% Extract the solution
for j = 1:length(var_names)
  name = var_names{j};
  i = reshape(v.(name).i, [], 1);
  if v.(name).type == 'I' 
    v.(name).value = reshape(round(result.x(i)), v.(name).size);
  elseif v.(name).type == 'B'
    v.(name).value = reshape(logical(round(result.x(i))), v.(name).size);
  else
    v.(name).value = reshape(result.x(i), v.(name).size);
  end
end


steps = v.x.value;
steps = [steps(1:3,:); zeros(2, size(steps, 2)); steps(4,:)];

plan = seed_plan;
for j = 1:nsteps
  plan.footsteps(j).pos = steps(:,j);
end
% region_order = nan(1, size(region, 2));
% for j = 1:length(region_order)
%   i = find(round(region(:,j)));
%   if ~isempty(i)
%     region_order(j) = i;
%   end
% end
% assert(length(region_order) == size(region, 2));
% plan.region_order = region_order;

% for j = 1:nsteps
%   plan.footsteps(j).pos = steps(:,j);
% end

% plan.relative_step_offsets()

% % Remove unnecessary footsteps
% at_final_pose = false(1, nsteps);
% at_final_pose(end-1:end) = true;
% for j = 3:nsteps-2
%   if mod(nsteps-j, 2)
%     at_final_pose(j) = all(abs(plan.footsteps(j).pos - plan.footsteps(end-1).pos) <= TRIM_THRESHOLD);
%   else
%     at_final_pose(j) = all(abs(plan.footsteps(j).pos - plan.footsteps(end).pos) <= TRIM_THRESHOLD);
%   end
% end
% trim = at_final_pose % Cut off any steps that are at the final poses
% trim(find(trim, 2, 'first')) = false % Don't cut off the final poses themselves

% plan = plan.slice(~trim);



% plan.sanity_check();
% plan.relative_step_offsets()

end

