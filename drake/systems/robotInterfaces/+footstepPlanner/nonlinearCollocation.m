function [plan, solvertime] = nonlinearCollocation(biped, seed_plan, weights, goal_pos, ~)
% A nonlinear footstep planner. This planner takes both the absolute pose of each footstep
% and the relative displacements between footsteps as decision variables, while
% constraining that they agree with one another. It optimizes the distance from the final step
% to the goal pose while minimizing relative footstep displacements and respecting reachability
% constraints.
% @param biped a Biped descendant (like Atlas.m)
% @param seed_plan a FootstepPlan to be optimized. The first two footsteps in the plan
%                  must be set to the current poses of the robot's feet, but the remainder
%                  can be NaN if desired. FootstepPlan.blank_plan() will produce a simple
%                  seed plan if desired.
% @param weights a struct of optimization weights, as returned by biped.getFootstepOptimizationWeights()
% @goal_pos a struct containing fields 'right' and 'left', indicating the desired positions of
%           the feet after walking

if checkDependency('snopt')
  USE_SNOPT = 1;
else
  USE_SNOPT = 0;
  warning('Drake:FootstepNLP:SNOPTNotFound', 'Snopt not found, using (slower) fmincon');
end
debug = false;
USE_MEX = 1;

seed_steps = seed_plan.footsteps;
region_order = seed_plan.region_order;
params = seed_plan.params;
safe_regions = seed_plan.safe_regions;
right_foot_lead = seed_steps(1).frame_id == biped.foot_frame_id.right;

if ~isfield(params, 'nom_step_width'); params.nom_step_width = 0.26; end

st0 = seed_steps(2).pos;
goal_pos.right(6) = st0(6) + angleDiff(st0(6), goal_pos.right(6));
goal_pos.left(6) = goal_pos.right(6) + angleDiff(goal_pos.right(6), goal_pos.left(6));
goal_pos.right(3) = st0(3);
goal_pos.left(3) = st0(3);
goal_pos.center = mean([goal_pos.right, goal_pos.left],2);

function [c, ceq, dc, dceq] = constraints(x)
  if USE_MEX == 0
    [c, ceq, dc, dceq] = footstepCollocationConstraints(x);
  elseif USE_MEX == 1
    [c, ceq, dc, dceq] = footstepCollocationConstraintsMex(x);
  else
    [c, ceq, dc, dceq] = footstepCollocationConstraints(x);
    [c_mex, ceq_mex, dc_mex, dceq_mex] = footstepCollocationConstraintsMex(x);
    if isempty(c)
      assert(isempty(c_mex));
      assert(isempty(dc));
      assert(isempty(dc_mex));
    else
      valuecheck(c, c_mex, 1e-8);
      valuecheck(dc, dc_mex, 1e-8);
    end
    valuecheck(ceq, ceq_mex, 1e-8);
    valuecheck(dceq, dceq_mex, 1e-8);
  end
end

nominal_dxy = [params.nom_forward_step; params.nom_step_width];
[cost_Q, cost_c] = footstepQuadraticCost(biped, seed_plan.slice(2:length(seed_plan.footsteps)), weights, goal_pos, nominal_dxy);


function [c, dc] = objfun(x)
%   [c, dc] = footstepCostFun(steps, steps_rel, weights, goal_pos, right_foot_lead, nominal_dxy);
  c = x' * cost_Q * x + cost_c' * x;
  dc = 2 * cost_Q * x + cost_c;
end

function [F,G] = collocation_userfun(x)
  [c, ceq, dc, dceq] = constraints(x);
  [cost, dCost] = objfun(x);
  F = [cost; reshape(c, [], 1); reshape(ceq, [], 1); zeros(size(A,1),1); zeros(size(Aeq,1),1)];
  G = [dCost, dc, dceq];
  G = reshape(G(iGndx), [], 1);
end

steps = [seed_plan.footsteps.pos];
steps = steps(:,2:end);
nsteps = size(steps,2);

nv = 12 * nsteps;

[A, b, Aeq, beq] = constructCollocationAb(biped, seed_plan, params);

lb = -inf(12,nsteps);
ub = inf(size(lb));
lb([4,5,10,11],:) = 0;
ub([4,5,10,11],:) = 0;
% Require that the first step be at the current stance foot pose
lb(1:6,1) = st0;
ub(1:6,1) = st0;
lb(7:12,1) = st0;
ub(7:12,1) = st0;
lb(12,:) = -pi;
ub(12,:) = pi;

x0 = encodeCollocationSteps(steps);

for j = 2:nsteps
  x_ndx = (j-1)*12+(1:6);
  if length(region_order) == 1
    region_ndx = 1;
  else
    region_ndx = j+1;
  end
  region = safe_regions(region_order(region_ndx));
  num_region_cons = length(region.b);
  expanded_A = zeros(num_region_cons, nv);
  expanded_A(1:length(region.b),x_ndx([1,2,6])) = region.A;
  A = [A; expanded_A];
  b = [b; region.b];

  expanded_Aeq = zeros(1, nv);
  expanded_Aeq(1, x_ndx([1,2,3])) = region.normal;
  Aeq = [Aeq; expanded_Aeq];
  beq = [beq; region.normal' * region.point];
end


if USE_SNOPT
  snseti ('Major Iteration limit', 250);
  if debug
    snseti ('Verify level', 3);
  else
    snseti ('Verify level', 0);
  end

  snseti ('Superbasics limit', 2000);
  n_obj = 1;
  n_proj_cons = 2*nsteps;
  iG = boolean(zeros(nv, n_obj + n_proj_cons));
  iA = boolean(zeros(size(iG,2)+size(A,1)+size(Aeq,1),nv));
  iG(:,1) = 1;

  x1_ndx = 1:6;
  dx_ndx = 7:12;
  con_ndx = (1:2)+1;
  iG(x1_ndx(1:2),con_ndx) = diag([1,1]);
  iG(dx_ndx(1:2),con_ndx) = diag([1,1]);

  for j = 2:nsteps
    con_ndx = (j-1)*2+(1:2) + n_obj;
    x1_ndx = (j-2)*12+(1:6);
    dx_ndx = (j-1)*12+(7:12);
    x2_ndx = (j-1)*12+(1:6);
    iG(x2_ndx(1:2),con_ndx) = diag([1,1]);
    iG(x1_ndx(1:2),con_ndx) = diag([1,1]);
    iG(x1_ndx(6),con_ndx) = [1,1];
    iG(dx_ndx(1:2),con_ndx) = [1 1; 1 1];
  end

  iGndx = find(iG);
  [jGvar, iGfun] = find(iG);

  iA(size(iG,2)+(1:size(A,1)),:)= A ~= 0;
  iA(size(iG,2)+size(A,1)+(1:size(Aeq,1)),:)= Aeq ~= 0;
  iAndx = find(iA);
  [iAfun, jAvar] = find(iA);
  A_sn = [zeros(size(iG,2),nv); A; Aeq];

  lb = reshape(lb, [],1);
  ub = reshape(ub, [],1);
  xlow = lb;
  xupp = ub;
  xmul = zeros(size(lb));
  xstate = zeros(size(lb));
  Flow = [-inf; zeros(n_proj_cons, 1); -inf(size(A,1),1); beq];
  Fupp = [inf; zeros(n_proj_cons, 1); b; beq];
  Fmul = zeros(size(Flow));
  Fstate = Fmul;
  ObjAdd = 0;
  ObjRow = 1;
  global SNOPT_USERFUN
  SNOPT_USERFUN = @collocation_userfun;
  t0 = tic();
  [xstar, fval, ~, ~, exitflag] = snsolve(x0,xlow,xupp,xmul,xstate,    ...
               Flow,Fupp,Fmul,Fstate,      ...
               ObjAdd,ObjRow,A_sn(iAndx),iAfun,jAvar,...
               iGfun,jGvar,'snoptUserfun');
  solvertime = toc(t0);
  if debug
    exitflag
  end
else
  t0 = tic();
  [xstar, fval, exitflag] = fmincon(@objfun, x0, sparse(A), b, ...
                  sparse(Aeq), beq, lb, ub, @constraints, ...
                  optimset('Algorithm', 'interior-point', ...
                  'DerivativeCheck', 'on', ...
                  'GradConstr', 'on', ...
                  'GradObj', 'on', 'OutputFcn',{}));
  solvertime = toc(t0);
end

% plotfun(xstar);

[output_steps, output_steps_rel] = decodeCollocationSteps(xstar);
output_cost = fval(1);

if exitflag < 10
  for j = 2:nsteps
    R = rotmat(output_steps(6,j-1));
    valuecheck(output_steps(:,j-1) + [R * output_steps_rel(1:2,j); output_steps_rel(3:6,j)], output_steps(:,j),1e-4);
  end
end
% nsteps

plan = seed_plan;
valuecheck(output_steps([1,2,6],1), plan.footsteps(2).pos([1,2,6]),1e-8);
for j = 2:nsteps
  plan.footsteps(j+1).pos = output_steps(:,j);
end

end


function x = encodeCollocationSteps(steps)
  nsteps = size(steps, 2);
  x = zeros(12,nsteps);
  x(1:6,:) = steps;
  x(7:12,1) = steps(:,1);
  for j = 2:nsteps
    R = rotmat(-steps(6,j-1));
    x(7:12,j) = [R * (steps(1:2,j) - steps(1:2,j-1));
                steps(3:6,j) - steps(3:6,j-1)];
  end
  x = reshape(x, [], 1);
end


function [steps, steps_rel] = decodeCollocationSteps(x)
  x = reshape(x, 12, []);
  steps = x(1:6,:);
  steps_rel = x(7:12,:);
end

function [c, ceq, dc, dceq] = footstepCollocationConstraints(x)
  [steps, rel_steps] = decodeCollocationSteps(x);
  nsteps = size(steps, 2);
  nv = length(x);
  nceq = 2 * nsteps;
  ceq = zeros(2, nsteps);
  dceq = zeros(nv, nceq);

  for j = 2:nsteps
    con_ndx = (j-1)*2+1:j*2;
    R = rotmat(steps(6,j-1));
    ct = R(1,1);
    st = -R(1,2);
    dxy = R * rel_steps(1:2,j);
    proj = steps(1:2,j-1) + dxy;
    ceq(:,j) = steps(1:2,j) - proj;
    x1_ndx = (j-2)*12+(1:6);
    dx_ndx = (j-1)*12+(7:12);
    x2_ndx = (j-1)*12+(1:6);
    dceq(x2_ndx(1:2),con_ndx) = diag(ones(2,1));
    dceq(x1_ndx(1:2),con_ndx) = -diag(ones(2,1));
    dx = rel_steps(1,j);
    dy = rel_steps(2,j);

    dceq(x1_ndx(6),con_ndx) = -[-dx*st - dy*ct, dx*ct - dy*st];

    dceq(dx_ndx(1),con_ndx) = -[ct,st];
    dceq(dx_ndx(2),con_ndx) = -[-st,ct];
  end
  ceq = reshape(ceq, nceq, 1);
  dceq = sparse(dceq);

  c = [];
  dc = [];
end

function [Q, c] = footstepQuadraticCost(biped, seed_plan, weights, goal_pos, nominal_dxy)
  % Build the Q matrix and c vector to be used as the cost function in footstepNLP.m
  % Q and c are defined such that the total cost of the NLP can be evaluated as
  % x'Qx + cx
  %
  % @param biped the Biped for which this footstep plan is being formed
  % @param seed_plan a FootstepPlan
  % @param weights optimization weights, as given by biped.getFootstepOptimizationWeights()
  % @param goal_pos a struct with fields 'right' and 'left' describing the desired poses of the feet
  % @param nominal_dxy a 2x1 vector expressing the nominal relative displacement of one foot in
  %                    the frame of the prior foot. The first element is the nominal forward step
  %                    in meters and the second element is the nominal step width in meters.

  nsteps = length(seed_plan.footsteps);
  nom_step = [reshape(nominal_dxy, [], 1); zeros(4,1)];
  nvar = 12 * nsteps;
  world_ndx = bsxfun(@plus, repmat((1:6)', 1, nsteps), 0:12:(12*(nsteps-1)));
  rel_ndx = bsxfun(@plus, repmat((7:12)', 1, nsteps), 0:12:(12*(nsteps-1)));

  Q = zeros(nvar, nvar);
  c = zeros(nvar, 1);

  w_goal = diag(weights.goal);
  for j = (nsteps - 1):nsteps
    if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.right
      xg = reshape(goal_pos.right, [], 1);
    else
      xg = reshape(goal_pos.left, [], 1);
    end
    Q(world_ndx(:,j), world_ndx(:,j)) = w_goal;
    c(world_ndx(:,j)) = -2 * xg' * w_goal;
  end

  w_rel = diag(weights.relative);
  for j = 2:nsteps
    if j == nsteps
      w_rel = diag(weights.relative_final);
      nom_step(1) = 0;
    end
    Q(rel_ndx(:,j), rel_ndx(:,j)) = Q(rel_ndx(:,j), rel_ndx(:,j)) + w_rel;

    if seed_plan.footsteps(j).frame_id == biped.foot_frame_id.right
      nom = diag([1,-1,1,1,1,-1]) *nom_step;
    else
      nom = nom_step;
    end
    c(rel_ndx(:,j)) = c(rel_ndx(:,j)) - (2 * nom' * w_rel)';
  end

  Q = sparse(Q);
  c = sparse(c);
end
