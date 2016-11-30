function [plan, solvertime] = footstepMIQP(biped, seed_plan, weights, goal_pos, ~)
% Run the Mixed Integer Quadratic Program form of the footstep planning problem.
% This form can efficiently choose the assignment of individual foot positions to
% safe (obstacle-free) regions, but always keeps the yaw value of every foot
% fixed.
% 
% Note: this planner will soon be replaced by footstepPlanner.fixedRotation(), which 
% should produce the same results but uses a more consistent internal structure.
%
% The structure of the desired footstep plan is indicated by the seed plan.
% @param seed_plan a FootstepPlan object which specifies the structure of the
%                  desired footstep plan. This seed_plan must contain a
%                  list of Footsteps and the region_order of matching length,
%                  but undetermined footstep positions and region assignments
%                  can be NaN. The first two footstep poses must not be NaN,
%                  since they correspond to the current positions of the feet
% @retval plan a FootstepPlan matching the number of footsteps, frame_id, etc.
%              of the seed plan, but with the footstep positions and region_order
%              replaced by the results of the MIQP

DEBUG = false;

checkDependency('gurobi');

seed_plan.sanity_check();
max_num_steps = seed_plan.params.max_num_steps + 2;
min_num_steps = max([seed_plan.params.min_num_steps + 2, 3]);

nsteps = length(seed_plan.footsteps); % ignore the first fixed footstep since it has no effect
nr = length(seed_plan.safe_regions);
nx = 4 * nsteps;
ns = (nsteps) * nr;
nt = nsteps;
nvar = nx + ns + nt;
x_ndx = reshape(1:nx, 4, nsteps);
s_ndx = reshape(nx + (1:ns), nr, ns / nr);
t_ndx = reshape(nx + ns + (1:nt), 1, nsteps);
goal_pos.center = mean([goal_pos.right, goal_pos.left],2);

x0 = nan(1, nvar);
R = cell(nsteps, 1);
seed_steps = [seed_plan.footsteps.pos];
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
    x0(x_ndx(1:3,j)) = p0(1:3);
    x0(x_ndx(4,j)) = seed_steps(6,2);
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

  if all(seed_steps(:,j) == seed_steps(:,j+2))
    x0(t_ndx(j)) = true;
  else
    x0(t_ndx(j)) = false;
  end
end
x0(t_ndx(end-1:end)) = true;

% nom_step = [seed_plan.params.nom_forward_step; seed_plan.params.nom_step_width; 0; 0]
nom_step = [seed_plan.params.nom_forward_step; seed_plan.params.nom_step_width; 0; 0];

A = [];
b = [];
Aeq = [];
beq = [];
objcon = 0;
Q = zeros(nvar, nvar);
c = zeros(nvar, 1);
lb = -inf(nvar, 1);
ub = inf(nvar, 1);

for j = 3:nsteps
  [A_reach, b_reach] = biped.getReachabilityPolytope(seed_plan.footsteps(j-1).frame_id, seed_plan.footsteps(j).frame_id, seed_plan.params);
  A_reach = A_reach(:,[1:3,6]);
  Ai = zeros(size(A_reach, 1), nvar);
  rA_reach = A_reach * R{j};
  Ai(:,x_ndx(:,j)) = rA_reach;
  Ai(:,x_ndx(:,j-1)) = -rA_reach;
  bi = b_reach;
  A = [A; Ai];
  b = [b; bi];
end

% Require that t(j) <= t(j+1)
At = zeros(nsteps-1, nvar);
At(:,t_ndx(1:end-1)) = eye(nsteps-1);
At(:,t_ndx(2:end)) = At(:,t_ndx(2:end)) + -eye(nsteps-1);
bt = zeros(size(At, 1), 1);
A = [A; At];
b = [b; bt];


w_trim = 1 * weights.relative(1) * seed_plan.params.nom_forward_step^2;
% If t(j) is true, then require that step(i) == step(i+2) 
M = 10;
for j = 3:nsteps-2
  Ati = zeros(3, nvar);
  Ati(:,x_ndx(1:3,j)) = eye(3);
  Ati(:,x_ndx(1:3,j+2)) = -eye(3);
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
for j = nsteps-1:nsteps
  if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.right
    xg = reshape(goal_pos.right([1,2,3,6]), [], 1);
  else
    xg = reshape(goal_pos.left([1,2,3,6]), [], 1);
  end
  Q(x_ndx(:,j), x_ndx(:,j)) = w_goal;
  c(x_ndx(:,j)) = -2 * xg' * w_goal;
  objcon = objcon + xg' * w_goal * xg;
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

  if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.right
    nom = diag([1,-1,1,-1]) *nom_step;
  else
    nom = nom_step;
  end
  c(x_ndx(:,j)) = c(x_ndx(:,j)) - (2 * nom' * w_rel * R{j})';
  c(x_ndx(:,j-1)) = c(x_ndx(:,j-1)) + (2 * nom' * w_rel * R{j})';
  objcon = objcon + nom' * R{j} * w_rel * R{j} * nom;
end

for j = 3:nsteps
  Aeqi = zeros(1, nvar);
  Aeqi(1, s_ndx(:,j)) = 1;
  beqi = 1;
  Aeq = [Aeq; Aeqi];
  beq = [beq; beqi];
end

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

step1 = seed_plan.footsteps(1).pos;
step2 = seed_plan.footsteps(2).pos;
lb(x_ndx(:,1)) = step1([1,2,3,6]);
lb(x_ndx(:,2)) = step2([1,2,3,6]);
ub(x_ndx(:,1:2)) = lb(x_ndx(:,1:2));
lb(x_ndx(4,3:end)) = x0(x_ndx(4,3:end)) - 0.01;
ub(x_ndx(4,3:end)) = x0(x_ndx(4,3:end)) + 0.01;
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
model.vtype = [repmat('C', nx, 1); repmat('B', ns, 1); repmat('B', nt, 1)];
model.Q = sparse(Q);
model.start = x0;
model.objcon = objcon;
params = struct();
params.timelimit = 5;
params.mipgap = 1e-4;
if DEBUG
  params.outputflag = 1;
else
  params.outputflag = 0;
end

result = gurobi(model, params);
if strcmp(result.status, 'INFEASIBLE') || strcmp(result.status, 'INF_OR_UNBD')
  error('Drake:MixedIntegerConvexProgram:InfeasibleProblem', 'The problem is infeasible');
end
solvertime = result.runtime;
xstar = result.x;
steps = xstar(x_ndx);
steps = [steps(1:3,:); zeros(2, size(steps, 2)); steps(4,:)];
% diff(steps, 1, 2)

region_assignments = reshape(xstar(s_ndx), nr, nsteps);
[region_order, ~] = find(abs(region_assignments - 1) < 1e-2);
assert(length(region_order) == size(region_assignments, 2));

plan = seed_plan;
for j = 1:nsteps
  plan.footsteps(j).pos = steps(:,j);
end
plan.region_order = region_order;

trim = xstar(t_ndx);
final_steps = find(trim, 2);
if plan.footsteps(end).frame_id == biped.foot_frame_id.right
  dtheta = abs(angleDiff(plan.footsteps(end).pos(6), goal_pos.right(6)));
else
  dtheta = abs(angleDiff(plan.footsteps(end).pos(6), goal_pos.left(6)));
end

max_yaw_rate = max([seed_plan.params.max_outward_angle, seed_plan.params.max_inward_angle]);

final_step_idx = min(nsteps, final_steps(end) + ceil(2 * dtheta / max_yaw_rate));

if plan.footsteps(1).frame_id == biped.foot_frame_id.right
  dtheta = abs(angleDiff(plan.footsteps(1).pos(6), goal_pos.right(6)));
else
  dtheta = abs(angleDiff(plan.footsteps(1).pos(6), goal_pos.left(6)));
end
min_num_steps = max(min_num_steps, ceil(2 * dtheta / max_yaw_rate + 2));


final_nsteps = min(max_num_steps, max(min_num_steps, final_step_idx));
plan = plan.slice(1:final_nsteps);
