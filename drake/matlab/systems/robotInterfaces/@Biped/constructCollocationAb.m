function [A, b, Aeq, beq, step_map] = constructCollocationAb(biped, seed_plan, params)
  % Build the linear constraint matrices for the nonlinear footstep planner.
  % These constraint matrices enforce the relative footstep reachability constraints
  % and the constraints that the relative footstep displacments match the absolute
  % footstep displacements.
  % @retval step_map a struct with fields 'ineq' and 'eq'. Each field is a map from
  %                  a footstep index to the constraints relating specifically to
  %                  that footstep, which we use to view the infeasability of
  %                  individual footsteps later

  nsteps = length(seed_plan.footsteps) - 1;
  nv = 12 * nsteps;

  step_map.ineq = containers.Map('KeyType', 'int32', 'ValueType', 'any');
  step_map.eq = containers.Map('KeyType', 'int32', 'ValueType', 'any');

  A = [];
  b = [];
  offset = 0;
  for j = 2:nsteps
    [A_reach, b_reach] = biped.getReachabilityPolytope(seed_plan.footsteps(j).frame_id, seed_plan.footsteps(j+1).frame_id, params);
    A = [A; zeros(length(b_reach), nv)];
    b = [b; b_reach];
    con_ndx = offset + (1:length(b_reach));
    var_ndx = (j-1)*12+7:j*12;
    A(con_ndx, var_ndx) = A_reach;
    step_map.ineq(j) = con_ndx;
    offset = offset + length(b_reach);
  end

  Aeq = zeros(4*(nsteps),nv);
  beq = zeros(4*(nsteps),1);
  con_ndx = 1:4;
  x1_ndx = 1:6;
  dx_ndx = 7:12;
  Aeq(con_ndx, x1_ndx([1,2,3,6])) = -diag(ones(4,1));
  Aeq(con_ndx, dx_ndx([1,2,3,6])) = diag(ones(4,1));
  for j = 2:nsteps
    con_ndx = (j-1)*4+(1:4);
    x1_ndx = (j-2)*12+(1:6);
    x2_ndx = (j-1)*12+(1:6);
    dx_ndx = (j-1)*12+(7:12);
    Aeq(con_ndx, x1_ndx(3:6)) = -diag(ones(4,1));
    Aeq(con_ndx, x2_ndx(3:6)) = diag(ones(4,1));
    Aeq(con_ndx, dx_ndx(3:6)) = -diag(ones(4,1));
    step_map.eq(j) = con_ndx;
  end
end