function [plan, v] = footstepMISOCP_grb(biped, seed_plan, weights, goal_pos, v_seed)

checkDependency('gurobi');
if nargin < 5
  v_seed = [];
end
seed_plan.sanity_check();
rangecheck(seed_plan.footsteps(1).pos(6), -pi, pi);
rangecheck(seed_plan.footsteps(2).pos(6), -pi, pi);

nsteps = length(seed_plan.footsteps);
foci = [[0; 0.15], [0; -0.7]];
ellipse_l = 0.55;

seed_steps = [seed_plan.footsteps.pos];

min_yaw = pi * floor(seed_steps(6,1) / pi - 1);
max_yaw = pi * ceil(seed_steps(6,1) / pi + 1);
cos_boundaries = reshape(bsxfun(@plus, [min_yaw:pi:max_yaw; min_yaw:pi:max_yaw], [-(pi/2-1); (pi/2-1)]), 1, []);
sin_boundaries = reshape(bsxfun(@plus, [min_yaw:pi:max_yaw; min_yaw:pi:max_yaw], [-1; 1]), 1, []);


% Build a struct to hold the sizes and indices of our decision variables
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
  % if isscalar(start_)
  %   start_ = repmat(start_, v.(name).size);
  % end
  v.(name).start = nan(v.(name).size);
  v.(name).start(1:size(start_, 1), 1:size(start_, 2)) = start_;
end

x_lb = [-100 + repmat(seed_steps(1:3,1), 1, nsteps); 
          repmat(min_yaw, 1, nsteps)];
x_ub = [100 + repmat(seed_steps(1:3,1), 1, nsteps);
          repmat(max_yaw, 1, nsteps)];
x_start = seed_steps([1,2,3,6],:);
add_var('x', 'C', [4, nsteps], x_lb, x_ub, x_start);
if isempty(v_seed)
  add_var('cos_yaw', 'C', [1, nsteps], -1, 1);
  add_var('sin_yaw', 'C', [1, nsteps], -1, 1);
  add_var('region', 'B', [length(seed_plan.safe_regions), nsteps], 0, 1);
  add_var('cos_sector', 'B', [length(cos_boundaries)-1, nsteps], 0, 1);
  add_var('sin_sector', 'B', [length(sin_boundaries)-1, nsteps], 0, 1);
  add_var('trim', 'B', [1, nsteps], 0, 1);
else
  add_var('cos_yaw', 'C', [1, nsteps], -1, 1, v_seed.cos_yaw.value);
  add_var('sin_yaw', 'C', [1, nsteps], -1, 1, v_seed.sin_yaw.value);
  add_var('region', 'B', [length(seed_plan.safe_regions), nsteps], 0, 1, v_seed.region.value);
  add_var('cos_sector', 'B', [length(cos_boundaries)-1, nsteps], 0, 1, v_seed.cos_sector.value);
  add_var('sin_sector', 'B', [length(sin_boundaries)-1, nsteps], 0, 1, v_seed.sin_sector.value);
  add_var('trim', 'B', [1, nsteps], 0, 1, v_seed.trim.value);
end

c = zeros(nv, 1);
Q = sparse(nv, nv);
A = zeros(0, nv);
b = zeros(0, 1);
Aeq = zeros(0, nv);
beq = zeros(0, 1);
quadcon = struct('Qc', {}, 'q', {}, 'rhs', {});
objcon = 0;

% x(:,1) == seed_steps([1,2,3,6],1),...
ai = zeros(4, nv);
ai(:,v.x.i(:,1)) = eye(4);
bi = seed_steps([1,2,3,6], 1);
Aeq = [Aeq; ai]; beq = [beq; bi];

% x(:,2) == seed_steps([1,2,3,6],2),...
ai = zeros(4, nv);
ai(:,v.x.i(:,2)) = eye(4);
bi = seed_steps([1,2,3,6], 2);
Aeq = [Aeq; ai]; beq = [beq; bi];

% sum(region, 1) + trim == 1,...
offset = 0;
Aeq_i = zeros(nsteps, nv);
beq_i = zeros(size(Aeq_i, 1), 1);
expected_offset = size(Aeq_i, 1);
for j = 1:nsteps
  % ai = zeros(1, nv);
  Aeq_i(offset+1, v.region.i(:,j)) = 1;
  Aeq_i(offset+1, v.trim.i(j)) = 1;
  beq_i(offset+1) = 1;
  offset = offset + 1;
end
assert(offset == expected_offset);
Aeq = [Aeq; Aeq_i];
beq = [beq; beq_i];

% sum(sin_sector, 1) == 1,...
for j = 1:nsteps
  ai = zeros(1, nv);
  ai(v.sin_sector.i(:,j)) = 1;
  bi = 1;
  Aeq = [Aeq; ai]; beq = [beq; bi];
end

% sum(cos_sector, 1) == 1,...
for j = 1:nsteps
  ai = zeros(1, nv);
  ai(v.cos_sector.i(:,j)) = 1;
  bi = 1;
  Aeq = [Aeq; ai]; beq = [beq; bi];
end

% trim(1:2) == 1
v.trim.lb(1:2) = 1;
v.trim.ub(1:2) = 1;

% trim(1:end-1) >= trim(2:end)
offset = 0;
A_i = zeros(nsteps-1, nv);
b_i = zeros(size(A_i, 1), 1);
expected_offset = size(A_i, 1);
for j = 2:nsteps
  A_i(offset+1, v.trim.i(j)) = 1;
  A_i(offset+1, v.trim.i(j-1)) = -1;
  offset = offset + 1;
end
A = [A; A_i];
b = [b; b_i];
assert(offset == expected_offset);

% sum(trim) <= nsteps - (min_num_steps - 2)];
min_num_steps = max([seed_plan.params.min_num_steps, 1]);
A_i = zeros(1, nv);
A_i(1, v.trim.i) = 1;
b_i = nsteps - min_num_steps;
A = [A; A_i];
b = [b; b_i];


offset = 0;
As = zeros(4*(length(cos_boundaries)-1)*nsteps, nv);
bs = zeros(size(As, 1), 1);
expected_offset = size(As, 1);
% Enforce approximation of cosine
for j = 1:nsteps
  for s = 1:length(cos_boundaries) - 1
    th0 = cos_boundaries(s);
    th1 = cos_boundaries(s+1);
    th = (th0 + th1)/2;
    cos_slope = -sin(th);
    cos_intercept = cos(th) - (cos_slope * th);

    % implies(cos_sector(s, j), th0 <= yaw(j) <= th1),...
    M = 4 * pi;
    As(offset+1:offset+2, v.cos_sector.i(s,j)) = M;
    As(offset+1, v.x.i(4,j)) = -1;
    bs(offset+1) = -th0 + M;
    As(offset+2, v.x.i(4,j)) = 1;
    bs(offset+2) = th1 + M;
    offset = offset + 2;

    % implies(cos_sector(s, j), cos_yaw(j) == cos_slope * yaw(j) + cos_intercept)];
    As(offset+1:offset+2, v.cos_sector.i(s,j)) = M;
    As(offset+1, v.cos_yaw.i(j)) = 1;
    As(offset+1, v.x.i(4,j)) = -cos_slope;
    bs(offset+1) = cos_intercept + M;
    As(offset+2, v.cos_yaw.i(j)) = -1;
    As(offset+2, v.x.i(4,j)) = cos_slope;
    bs(offset+2) = -cos_intercept + M;
    offset = offset + 2;
  end
end
assert(offset == expected_offset);
A = [A; As];
b = [b; bs];

offset = 0;
As = zeros(4*(length(sin_boundaries)-1)*nsteps, nv);
bs = zeros(size(As, 1), 1);
expected_offset = size(As, 1);
% Enforce approximation of sine
for j = 1:nsteps
  for s = 1:length(sin_boundaries) - 1
    th0 = sin_boundaries(s);
    th1 = sin_boundaries(s+1);
    th = (th0 + th1)/2;
    sin_slope = cos(th);
    sin_intercept = sin(th) - (sin_slope * th);

    % implies(sin_sector(s, j), th0 <= yaw(j) <= th1),...
    M = 4 * pi;
    As(offset+1:offset+2, v.sin_sector.i(s,j)) = M;
    As(offset+1, v.x.i(4,j)) = -1;
    bs(offset+1) = -th0 + M;
    As(offset+2, v.x.i(4,j)) = 1;
    bs(offset+2) = th1 + M;
    offset = offset + 2;

    % implies(sin_sector(s, j), sin_yaw(j) == sin_slope * yaw(j) + sin_intercept)];
    As(offset+1:offset+2, v.sin_sector.i(s,j)) = M;
    As(offset+1, v.sin_yaw.i(j)) = 1;
    As(offset+1, v.x.i(4,j)) = -sin_slope;
    bs(offset+1) = sin_intercept + M;
    As(offset+2, v.sin_yaw.i(j)) = -1;
    As(offset+2, v.x.i(4,j)) = sin_slope;
    bs(offset+2) = -sin_intercept + M;
    offset = offset + 2;
  end
end
assert(offset == expected_offset);
A = [A; As];
b = [b; bs];

% offset = 0;
% As = zeros(2 * v.sin_sector.size(1) * nsteps, nv);
% bs = zeros(size(As, 1), 1);
% % Enforce range between sin/cos sectors
% for j = 1:nsteps
%   for k = 1:v.sin_sector.size(1)
%     % sum(sin_sector(max(1,k-1):min(k+1,size(sin_sector,1)),j)) >= cos_sector(k,j),...
%     As(offset+1, v.sin_sector.i(max(1,k-1):min(k+1,v.sin_sector.size(1)),j)) = -1;
%     As(offset+1, v.cos_sector.i(k,j)) = 1;
%     % sum(cos_sector(max(1,k-1):min(k+1,size(cos_sector,1)),j)) >= sin_sector(k,j)];
%     As(offset+2, v.cos_sector.i(max(1,k-1):min(k+1,v.cos_sector.size(1)),j)) = -1;
%     As(offset+2, v.sin_sector.i(k,j)) = 1;
%     offset = offset + 2;
%   end
% end
% assert(offset == size(As, 1));
% A = [A; As];
% b = [b; bs];

disp('before reach')
size(A, 1)
offset = 0;
As = zeros((2 + 2*(v.cos_sector.size(1)) + 2*size(foci,2)) * (nsteps-2), nv);
bs = zeros(size(As, 1), 1);
expected_offset = size(As, 1);
% Reachability between steps
for j = 2:nsteps-1
  % Ensure that the foot doesn't yaw too much per step
  if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.right
    rel_foci = [foci(1,:); -foci(2,:)];
    yaw_range = [0, pi/8];
  else
    rel_foci = foci;
    yaw_range = [-pi/8, 0];
  end
  
  % Don't shift by more than one sin/cos sector per step
  for k = 1:v.cos_sector.size(1)
    As(offset+1, v.cos_sector.i(k, j)) = 1;
    As(offset+1, v.cos_sector.i(max(k-1, 1):min(k+1,v.cos_sector.size(1)), j+1)) = -1;
    As(offset+1, v.trim.i(j+1)) = -5;
    As(offset+2, v.sin_sector.i(k, j)) = 1;
    As(offset+2, v.sin_sector.i(max(k-1, 1):min(k+1,v.cos_sector.size(1)), j+1)) = -1;
    As(offset+2, v.trim.i(j+1)) = -5;
    offset = offset + 2;
  end

  % implies(~trim(j+1), yaw_range(1) <= yaw(j+1) - yaw(j) <= yaw_range(2));
  As(offset+1, v.x.i(4,j)) = 1;
  As(offset+1, v.x.i(4,j+1)) = -1;
  As(offset+1, v.trim.i(j+1)) = -pi;
  bs(offset+1) = -yaw_range(1);
  As(offset+2, v.x.i(4,j)) = -1;
  As(offset+2, v.x.i(4,j+1)) = 1;
  As(offset+2, v.trim.i(j+1)) = -pi;
  bs(offset+2) = yaw_range(2);
  offset = offset + 2;

  for k = 1:size(rel_foci, 2)
    % Constraints = [Constraints, ...
    %   cone(x(1:2,j) + [cos_yaw(j), -sin_yaw(j); sin_yaw(j), cos_yaw(j)] * rel_foci(:,k) - x(1:2,j+1), ellipse_l),...
    Qc = sparse(nv, nv);
    qc = zeros(nv, 1);
    rhs = ellipse_l^2;
    Qc(v.x.i(1:2,j+1),v.x.i(1:2,j+1)) = eye(2);
    Qc(v.x.i(1:2,j),v.x.i(1:2,j+1)) = -2*eye(2);
    Qc(v.x.i(1:2,j),v.x.i(1:2,j)) = eye(2);
    Qc(v.x.i(1,j+1),v.cos_yaw.i(j)) = -2*rel_foci(1,k);
    Qc(v.x.i(1,j+1),v.sin_yaw.i(j)) = 2*rel_foci(2,k);
    Qc(v.x.i(2,j+1),v.sin_yaw.i(j)) = -2*rel_foci(1,k);
    Qc(v.x.i(2,j+1),v.cos_yaw.i(j)) = -2*rel_foci(2,k);
    Qc(v.x.i(1,j),v.cos_yaw.i(j)) = 2*rel_foci(1,k);
    Qc(v.x.i(1,j),v.sin_yaw.i(j)) = -2*rel_foci(2,k);
    Qc(v.x.i(2,j),v.sin_yaw.i(j)) = 2*rel_foci(1,k);
    Qc(v.x.i(2,j),v.cos_yaw.i(j)) = 2*rel_foci(2,k);
    Qc(v.cos_yaw.i(j),v.cos_yaw.i(j)) = rel_foci(1,k)^2 + rel_foci(2,k)^2;
    Qc(v.sin_yaw.i(j),v.sin_yaw.i(j)) = rel_foci(1,k)^2 + rel_foci(2,k)^2;

    % Verify that our Qc form correctly measures the distance from the focus to the next footstep
    temp = zeros(nv, 1);
    tx1 = rand(2,1);
    tx2 = rand(2,1);
    tc = rand();
    ts = rand();
    d1 = norm(tx2 - (tx1 + [tc, -ts; ts, tc] * rel_foci(:,k)));
    temp(v.x.i(1:2,j)) = tx1;
    temp(v.x.i(1:2,j+1)) = tx2;
    temp(v.cos_yaw.i(j)) = tc;
    temp(v.sin_yaw.i(j)) = ts;
    d2 = sqrt(temp' * Qc * temp);
    valuecheck(d1, d2, 1e-4);

    quadcon(end+1) = struct('Qc', Qc, 'q', qc, 'rhs', rhs);

      % abs(x(3,j+1) - x(3,j)) <= seed_plan.params.nom_upward_step];
    As(offset+1, v.x.i(3,j+1)) = 1;
    As(offset+1, v.x.i(3,j)) = -1;
    bs(offset+1) = seed_plan.params.nom_upward_step;
    As(offset+2, v.x.i(3,j+1)) = -1;
    As(offset+2, v.x.i(3,j)) = 1;
    bs(offset+2) = seed_plan.params.nom_downward_step;
    offset = offset + 2;
  end
end
assert(offset == expected_offset);
A = [A; As];
b = [b; bs];
disp('after reach')
size(A, 1)

% Enforce membership in safe regions
M = 100;
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

% trim(j) fixes step j to the initial pose of that foot (so we can trim it out of the plan later)
A_i = zeros((8)*(nsteps - 2), nv);
b_i = zeros(size(A_i, 1), 1);
offset = 0;
expected_offset = size(A_i, 1);
M = 100;
for j = 3:nsteps
  if seed_plan.footsteps(j).frame_id == seed_plan.footsteps(1).frame_id
    % Constraints = [Constraints, implies(trim(j), x(:,j) == seed_steps([1,2,3,6],1))];
    k = 1;
  else
    % Constraints = [Constraints, implies(trim(j), x(:,j) == seed_steps([1,2,3,6], 2))];
    k = 2;
  end
  target = seed_steps([1,2,3,6], k);
  % x(:,j) <= target
  A_i(offset+(1:4), v.x.i(:,j)) = eye(4);
  A_i(offset+(1:4), v.trim.i(j)) = M;
  b_i(offset+(1:4)) = target + M;
  offset = offset + 4;

  % x(:,j) >= target ---> -x(:,j) <= -target
  A_i(offset+(1:4), v.x.i(:,j)) = -eye(4);
  A_i(offset+(1:4), v.trim.i(j)) = M;
  b_i(offset+(1:4)) = -target + M;
  offset = offset + 4;
end
assert(offset == expected_offset);
A = [A; A_i];
b = [b; b_i];


% Distance to goal objective
w_goal = diag(weights.goal([1,2,3,6]));
for j = nsteps-1:nsteps
  if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.right
    xg = reshape(goal_pos.right([1,2,3,6]), [], 1);
  else
    xg = reshape(goal_pos.left([1,2,3,6]), [], 1);
  end
  Q(v.x.i(:,j), v.x.i(:,j)) = w_goal;
  c(v.x.i(:,j)) = -2 * xg' * w_goal;
%   objcon = objcon + xg' * w_goal * xg;
end

% Step displacement objective
w_rel = diag(weights.relative([1,1,3,6]));
for j = 3:nsteps
  if j == nsteps
    w_rel = diag(weights.relative_final([1,1,3,6]));
  end
  Q(v.x.i(:,j), v.x.i(:,j)) = Q(v.x.i(:,j), v.x.i(:,j)) + w_rel;
  Q(v.x.i(:,j-1), v.x.i(:,j)) = Q(v.x.i(:,j-1), v.x.i(:,j)) - 2 * w_rel;
  Q(v.x.i(:,j-1), v.x.i(:,j-1)) = Q(v.x.i(:,j-1), v.x.i(:,j-1)) + w_rel;
end

% Trim objective
w_trim = 1 * w_rel(1) * (seed_plan.params.nom_forward_step^2);
for j = 3:nsteps
  c(v.trim.i(j)) = -w_trim;
end

var_names = fieldnames(v);
clear model params
model.A = sparse([A; Aeq]);
model.rhs = [b; beq];
model.sense = [repmat('<', size(A,1), 1); repmat('=', size(Aeq, 1), 1)];
model.start = nan(nv, 1);
model.obj = c;
model.Q = Q;
model.quadcon = quadcon;
model.objcon = objcon;

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

params.mipgap = 1e-3;
params.outputflag = 1;

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

% Sanity check the solution
for j = 1:nsteps
  for s = 1:length(cos_boundaries) - 1
    th0 = cos_boundaries(s);
    th1 = cos_boundaries(s+1);

    th = (th0 + th1)/2;
    cos_slope = -sin(th);
    cos_intercept = cos(th) - (cos_slope * th);
    if v.cos_sector.value(s,j)
      assert(v.x.value(4,j) >= th0-1e-3);
      assert(v.x.value(4,j) <= th1+1e-3);
      assert(abs(v.cos_yaw.value(j) - (cos_slope * v.x.value(4,j) + cos_intercept)) < 1e-3);
    end
  end
end
for j = 1:nsteps
  for s = 1:length(sin_boundaries) - 1
    th0 = sin_boundaries(s);
    th1 = sin_boundaries(s+1);

    th = (th0 + th1)/2;
    sin_slope = cos(th);
    sin_intercept = sin(th) - (sin_slope * th);
    if v.sin_sector.value(s,j)
      assert(v.x.value(4,j) >= th0-1e-3);
      assert(v.x.value(4,j) <= th1+1e-3);
      assert(abs(v.sin_yaw.value(j) - (sin_slope * v.x.value(4,j) + sin_intercept)) < 1e-3);
    end
  end
end

steps = v.x.value;
steps = [steps(1:3,:); zeros(2, size(steps, 2)); steps(4,:)];

plan = seed_plan;
for j = 1:nsteps
  plan.footsteps(j).pos = steps(:,j);
end

region_order = nan(1, v.region.size(2));
for j = 1:length(region_order)
  i = find(v.region.value(:,j));
  if ~isempty(i)
    region_order(j) = i;
  end
end
assert(length(region_order) == length(plan.footsteps));
plan.region_order = region_order;

% Remove unnecessary footsteps
% trim = round(trim);
v.trim.value(find(v.trim.value, 2, 'last')) = false;
plan = plan.slice(~v.trim.value);
% sin_yaw = sin_yaw(~trim);
% cos_yaw = cos_yaw(~trim);

plan.sanity_check();
end