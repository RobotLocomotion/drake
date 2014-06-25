function plan = footstepMIQPWRotation(biped, seed_plan, weights, goal_pos)
% Run the Mixed Integer Quadratic Program form of the footstep planning problem.
% This form can efficiently choose the assignment of individual foot positions to
% safe (obstacle-free) regions, but always keeps the yaw value of every foot
% fixed.
%
% The structure of the desired footstep plan is indicated by the seed plan.
% @param seed_plan a FootstepPlan object which specifies the structure of the
%                  desired footstep plan. This seed_plan must contain a
%                  list of Footsteps and the region_order of matching length,
%                  but undetermined footstep positions and region assignments
%                  can be NaN. The first two footstep poses must not be NaN,
%                  since they correspond to the current positions of the feet
% @retval plan a FootstepPlan matching the number of footsteps, body_idx, etc.
%              of the seed plan, but with the footstep positions and region_order
%              replaced by the results of the MIQP

seed_plan.sanity_check();
checkDependency('gurobi');
max_num_steps = seed_plan.params.max_num_steps + 2;
min_num_steps = max([seed_plan.params.min_num_steps + 2, 3]);

nsteps = length(seed_plan.footsteps); % ignore the first fixed footstep since it has no effect
nr = length(seed_plan.safe_regions);
nx = 4 * nsteps;
ns = (nsteps) * nr;
nt = nsteps;

yaw_slots = 17;
yaw_increment = pi/8;

nrot = yaw_slots * nsteps;
nvar = nx + ns + nt + nrot;
x_ndx = reshape(1:nx, 4, nsteps);
s_ndx = reshape(nx + (1:ns), nr, ns / nr);
t_ndx = reshape(nx + ns + (1:nt), 1, nsteps);
rot_ndx = reshape(nx + ns + nt + (1:nrot), [], nsteps);
goal_pos.center = mean([goal_pos.right, goal_pos.left],2);

x0 = nan(1, nvar);
R = cell(nsteps, 1);
seed_steps = seed_plan.step_matrix();
start_pos = seed_steps(:,2);
for j = 1:nsteps
  if ~any(isnan(seed_steps(:,j)))
    p0 = seed_steps(:,j);
    x0(x_ndx(:,j)) = p0([1,2,3,6]);
    if j > 2
      R{j} = [rotmat(-seed_steps(6,j-1)), zeros(2,2);
           zeros(2,2), eye(2)];
    end
  else
    p0 = seed_steps(:,mod(j-1, 2)+1);
    x0(x_ndx(:,j)) = p0([1,2,3,6]);
    R{j} = [rotmat(-p0(6)), zeros(2,2);
     zeros(2,2), eye(2)];
  end

  si = false(nr, 1);
  if ~isnan(seed_plan.region_order(j))
    si(seed_plan.region_order(j)) = true;
  else
    si(1) = true;
  end
  x0(s_ndx(:,j)) = si;
end

% Set the initial seed for the t_i, which indicate that step i is fixed to
% take the same pose as the final step (and thus can be trimmed off later)
for j = 1:nsteps-2
  if mod(nsteps-j, 2)
    last_step = nsteps-1;
  else
    last_step = nsteps;
  end
  if all(seed_steps(:,j) == seed_steps(:,last_step))
    x0(t_ndx(j)) = true;
  else
    x0(t_ndx(j)) = false;
  end
end
x0(t_ndx(end-1:end)) = true;

% nom_step = [seed_plan.params.nom_forward_step; seed_plan.params.nom_step_width; 0; 0]
% nom_step = [0; seed_plan.params.nom_step_width; 0; 0];
nom_step = zeros(4,1);
w_trim = weights.relative(1) * seed_plan.params.nom_forward_step^2;

A = [];
b = [];
Aeq = [];
beq = [];
Q = zeros(nvar, nvar);
c = zeros(nvar, 1);
lb = -inf(nvar, 1);
ub = inf(nvar, 1);

M = 100;
for j = 3:nsteps
  [A_reach, b_reach] = biped.getReachabilityPolytope(seed_plan.footsteps(j-1).body_idx, seed_plan.footsteps(j).body_idx, seed_plan.params);
  A_reach = A_reach(:,[1:3,6]);

  if j > 3
    for k = 1:yaw_slots
      yaw = x0(x_ndx(4,1)) + yaw_increment * (k - ceil(yaw_slots / 2));
      rmat = [rotmat(-yaw), zeros(2,2);
             zeros(2,2), eye(2)];
      Ai = zeros(size(A_reach, 1), nvar);
      rA_reach = A_reach * rmat;
      Ai(:,x_ndx(:,j)) = rA_reach;
      Ai(:,x_ndx(:,j-1)) = -rA_reach;
      Ai(:,rot_ndx(k,j-1)) = M;
      bi = b_reach + M;
      A = [A; Ai];
      b = [b; bi];
    end
  else
    yaw = x0(x_ndx(4,2));
    rmat = [rotmat(-yaw), zeros(2,2);
             zeros(2,2), eye(2)];
    Ai = zeros(size(A_reach, 1), nvar);
    rA_reach = A_reach * rmat;
    Ai(:,x_ndx(:,j)) = rA_reach;
    Ai(:,x_ndx(:,j-1)) = -rA_reach;
    bi = b_reach;
    A = [A; Ai];
    b = [b; bi];
  end
end

% Require that t(j) <= t(j+1)
At = zeros(nsteps-1, nvar);
At(:,t_ndx(1:end-1)) = diag(ones(nsteps-1, 1));
At(:,t_ndx(2:end)) = At(:,t_ndx(2:end)) + diag(-ones(nsteps-1, 1));
bt = zeros(size(At, 1), 1);
A = [A; At];
b = [b; bt];

% If t(j) is true, then require that step(i) == step(end) or step(end-1) as
% appropriate.
M = 100;
for j = 3:nsteps-2
  Ati = zeros(3, nvar);
  Ati(:,x_ndx(1:3,j)) = diag(ones(3, 1));
  if mod(nsteps - j, 2)
    Ati(:,x_ndx(1:3,end-1)) = diag(-ones(3,1));
  else
    Ati(:,x_ndx(1:3,end)) = diag(-ones(3,1));
  end
  Ati = [Ati; -Ati];

  Ati(:,t_ndx(j)) = M;
  bti = M + zeros(size(Ati, 1), 1);
  A = [A; Ati];
  b = [b; bti];
end

for j = 3:nsteps
  c(t_ndx(j)) = -w_trim;
end

w_goal = diag(weights.goal([1,2,3,6]));
if seed_plan.footsteps(end).body_idx == biped.foot_bodies_idx.right
  xg = reshape(goal_pos.right([1,2,3,6]), [], 1);
else
  xg = reshape(goal_pos.left([1,2,3,6]), [], 1);
end
for j = nsteps:nsteps
  Q(x_ndx(:,j), x_ndx(:,j)) = w_goal;
  c(x_ndx(:,j)) = -2 * xg' * w_goal;
end

w_rel = diag(weights.relative([1,2,3,6]));
for j = 3:nsteps
  if j == nsteps
    w_rel = diag(weights.relative_final([1,2,3,6]));
    nom_step(1) = 0;
  end
  Q(x_ndx(:,j), x_ndx(:,j)) = Q(x_ndx(:,j), x_ndx(:,j)) + R{j}' * w_rel * R{j};
  Q(x_ndx(:,j-1), x_ndx(:,j)) = Q(x_ndx(:,j-1), x_ndx(:,j)) - R{j}' * w_rel * R{j};
  Q(x_ndx(:,j), x_ndx(:,j-1)) = Q(x_ndx(:,j), x_ndx(:,j-1)) - R{j}' * w_rel * R{j};
  Q(x_ndx(:,j-1), x_ndx(:,j-1)) = Q(x_ndx(:,j-1), x_ndx(:,j-1)) + R{j}' * w_rel * R{j};

  if seed_plan.footsteps(j).body_idx == biped.foot_bodies_idx.right
    nom = diag([1,-1,1,-1]) *nom_step;
  else
    nom = nom_step;
  end
  c(x_ndx(:,j)) = c(x_ndx(:,j)) - (2 * nom' * w_rel * R{j})';
  c(x_ndx(:,j-1)) = c(x_ndx(:,j-1)) + (2 * nom' * w_rel * R{j})';
end


% The s_ndx variables are binary selectors on the safe terrain regions
% If x(s_ndx(r,j)) == 1, then step j must be in region r, and thus
% safe_regions(r).A * x(x_ndx(:,j)) <= safe_regions(r).b
%
% Additionally, each safe region has a point and vector defining a plane
% and any steps in that region must live in that plane.
M = 100;
Ar = zeros((nsteps-2) * sum(cellfun(@(x) size(x, 1), {seed_plan.safe_regions.A})), nvar);
br = zeros(size(Ar, 1), 1);
offset = 0;
for j = 3:nsteps
  for r = 1:nr
    A_region = seed_plan.safe_regions(r).A;
    A_region = [A_region(:,1:2), zeros(size(A_region, 1), 1), A_region(:,3)];
    A_region = [A_region;
                reshape(seed_plan.safe_regions(r).normal, 1, []), 0;
                -reshape(seed_plan.safe_regions(r).normal, 1, []), 0];
    b_region = [seed_plan.safe_regions(r).b;
                seed_plan.safe_regions(r).normal' * seed_plan.safe_regions(r).point;
                -seed_plan.safe_regions(r).normal' * seed_plan.safe_regions(r).point];

    Ai = zeros(size(A_region, 1), nvar);
    Ai(:,x_ndx(:,j)) = A_region;
    Ai(:,s_ndx(r,j)) = M;
    bi = b_region + M;
    Ar(offset + (1:size(Ai, 1)), :) = Ai;
    br(offset + (1:size(Ai, 1)), :) = bi;
    offset = offset + size(Ai, 1);
  end
end
assert(offset == size(Ar, 1));
A = [A; Ar];
b = [b; br];

% Each step must be in exactly one safe region
for j = 3:nsteps
  Aeqi = zeros(1, nvar);
  Aeqi(1, s_ndx(:,j)) = 1;
  beqi = 1;
  Aeq = [Aeq; Aeqi];
  beq = [beq; beqi];
end

% The rot_ndx variables are binary selectors on rotation bins.
% Require that if x(rot_ndx(k,j)) is 1, then x(x_ndx(4,j)) == x0(x_ndx(4,1) + pi/8 * (k - 5)
% x_4j + M * rot_kj <= M + x0_4 + pi/8 * (k-5)
% x_4j - M * rot_kj >= -M + x0_4 + pi/8 * (k-5)
%
% x_4j + M * rot_kj <= M + x0_4 + pi/8 * (k-5)
% -x_4j + M * rot_kj <= M - (x0_4 + pi/8 * (k-5))
A_rot = zeros((nsteps-2) * yaw_slots * 2, nvar);
b_rot = zeros(size(A_rot, 1), 1);
con_ndx = 1;
M = 4 * pi;
for j = 3:nsteps
  for k = 1:yaw_slots
    A_rot(con_ndx, rot_ndx(k,j)) = M;
    A_rot(con_ndx, x_ndx(4,j)) = 1;
    b_rot(con_ndx) = M + x0(x_ndx(4,1)) + yaw_increment * (k - ceil(yaw_slots / 2));
    con_ndx = con_ndx + 1;

    A_rot(con_ndx, rot_ndx(k,j)) = M;
    A_rot(con_ndx, x_ndx(4,j)) = -1;
    b_rot(con_ndx) = M -  (x0(x_ndx(4,1)) + yaw_increment * (k - ceil(yaw_slots / 2)));
    con_ndx = con_ndx + 1;
  end
end
A = [A; A_rot];
b = [b; b_rot];

% Exactly one rotation slot can be occupied per step
Aeq_rot = zeros(nsteps - 2, nvar);
beq_rot = zeros(size(Aeq_rot, 1), 1);
for j = 3:nsteps
  con_ndx = j - 2;
  Aeq_rot(con_ndx, rot_ndx(:,j)) = 1;
  beq_rot(con_ndx) = 1;
end
Aeq = [Aeq; Aeq_rot];
beq = [beq; beq_rot];

% Rotation can never change by more than one slot per step
A_rot = zeros((nsteps - 3) * yaw_slots, nvar);
b_rot = zeros(size(A_rot, 1), 1);
con_ndx = 1;
for j = 4:nsteps
  if seed_plan.footsteps(j).body_idx == biped.foot_bodies_idx.left
    for k = 1:yaw_slots
      A_rot(con_ndx, rot_ndx(k,j-1)) = 1;
      A_rot(con_ndx, rot_ndx(k,j)) = -1;
      if k < yaw_slots
        A_rot(con_ndx, rot_ndx(k+1,j)) = -1;
      end
      b_rot(con_ndx) = 0;
      con_ndx = con_ndx + 1;
    end
  else
    for k = 1:yaw_slots
      A_rot(con_ndx, rot_ndx(k,j-1)) = 1;
      A_rot(con_ndx, rot_ndx(k,j)) = -1;
      if k > 1
        A_rot(con_ndx, rot_ndx(k-1,j)) = -1;
      end
      b_rot(con_ndx) = 0;
    end
    con_ndx = con_ndx + 1;
  end
end
A = [A; A_rot];
b = [b; b_rot];

step1 = seed_plan.footsteps(1).pos.inFrame(seed_plan.footsteps(1).frames.center);
step2 = seed_plan.footsteps(2).pos.inFrame(seed_plan.footsteps(2).frames.center);
lb(x_ndx(:,1)) = step1([1,2,3,6]);
lb(x_ndx(:,2)) = step2([1,2,3,6]);
ub(x_ndx(:,1:2)) = lb(x_ndx(:,1:2));
% lb(x_ndx(4,:)) = x0(x_ndx(4,:)) - 0.05;
% ub(x_ndx(4,:)) = x0(x_ndx(4,:)) + 0.05;
lb(s_ndx(:,1:2)) = [1, 1; zeros(nr-1, 2)];
ub(s_ndx(:,1:2)) = lb(s_ndx(:,1:2));
ub(t_ndx(1:2)) = 0;
lb(t_ndx(1:2)) = 0;
ub(t_ndx(end)) = 1;
lb(t_ndx(end)) = 1;

clear model params
model.A = sparse([A; Aeq]);
model.obj = c;
model.sense = [repmat('<', size(A,1), 1); repmat('=', size(Aeq, 1), 1)];
model.rhs = [b; beq];
model.lb = lb;
model.ub = ub;
model.vtype = [repmat('C', nx, 1); repmat('B', ns, 1); repmat('B', nt, 1); repmat('B', nrot, 1)];
model.Q = sparse(Q);
model.start = x0;
params = struct();
% params.timelimit = 20;
% params.mipgap = 3e-4;
params.mipgap = 1e-2;
params.outputflag = 1;

result = gurobi(model, params);
if strcmp(result.status, 'INFEASIBLE') || strcmp(result.status, 'INF_OR_UNBD')
  warning('DRC:footstepMIQP:InfeasibleProblem', 'The footstep planning problem is infeasible. This often occurs when the robot cannot reach from its current pose into any of the safe regions');
  plan = seed_plan.slice(1:2);
  return
end
xstar = result.x;
steps = xstar(x_ndx);
steps = [steps(1:3,:); zeros(2, size(steps, 2)); steps(4,:)];
% diff(steps, 1, 2)

region_assignments = reshape(xstar(s_ndx), nr, nsteps);
[region_order, ~] = find(abs(region_assignments - 1) < 1e-2);
assert(length(region_order) == size(region_assignments, 2));

plan = seed_plan;
for j = 1:nsteps
  plan.footsteps(j).pos = Point(plan.footsteps(j).frames.center, steps(:,j));
end
plan.region_order = region_order;

trim = xstar(t_ndx);
final_steps = find(trim, 2);
if plan.footsteps(end).body_idx == biped.foot_bodies_idx.right
  dtheta = abs(angleDiff(plan.footsteps(end).pos(6), goal_pos.right(6)));
else
  dtheta = abs(angleDiff(plan.footsteps(end).pos(6), goal_pos.left(6)));
end
final_step_idx = min(nsteps, final_steps(end) + ceil(2 * dtheta / (pi/8)));

if plan.footsteps(1).body_idx == biped.foot_bodies_idx.right
  dtheta = abs(angleDiff(plan.footsteps(1).pos(6), goal_pos.right(6)));
else
  dtheta = abs(angleDiff(plan.footsteps(1).pos(6), goal_pos.left(6)));
end
% TODO: don't hardcode the turning rate here
min_num_steps = max(min_num_steps, ceil(2 * dtheta / (pi/8) + 2));

rot = xstar(rot_ndx);
for j = 3:nsteps
  assert(abs(sum(rot(:,j)) - 1) < 1e-2);
  for k = 1:yaw_slots
    if rot(k,j) > (1 - 1e-2);
      assert(abs(xstar(x_ndx(4,j)) - (x0(x_ndx(4,1)) + yaw_increment * (k - ceil(yaw_slots / 2)))) < 1e-2);
    end
  end
end


final_nsteps = min(max_num_steps, max(min_num_steps, final_step_idx));
plan = plan.slice(1:final_nsteps);

if 0
  right_foot_lead = plan.footsteps(1).body_idx == biped.foot_bodies_idx.right;
  if ~right_foot_lead
    r_ndx = 1:2:nsteps;
    l_ndx = 2:2:nsteps;
  else
    r_ndx = 2:2:nsteps;
    l_ndx = 1:2:nsteps;
  end
  figure(1);
  clf
  plot(steps(1,r_ndx), steps(2, r_ndx), 'bo')
  hold on
  plot(steps(1,l_ndx), steps(2,l_ndx), 'ro')
  plot(steps(1,:), steps(2,:), 'k:')
  for j = 1:length(seed_plan.safe_regions)
    V = iris.thirdParty.polytopes.lcon2vert(seed_plan.safe_regions(j).A(:,1:2), seed_plan.safe_regions(j).b);
    k = convhull(V(:,1), V(:,2));
    patch(V(k,1), V(k,2), 'k', 'FaceAlpha', 0.2);
  end
  axis equal
end