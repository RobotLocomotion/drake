function evaluateNewSecondDerivativeMethod()
robot = createAtlas('rpy');

nq = robot.getNumPositions();
nv = robot.getNumVelocities();

n_bodies = length(robot.body);
bodyRange = [1, n_bodies];


old_time = 0;
new_time = 0;

for test = 1 : 50
  base = randi(bodyRange);
  end_effector = randi(bodyRange);
  expressed_in = randi(bodyRange);
  
  q = randn(nq, 1);
  
  tic
  options.use_mex = false;
  options.compute_gradients = true;
  kinsol = robot.doKinematics(q, [], [], [], options);
  [J, v_indices, dJ] = robot.geometricJacobian(kinsol, base, end_effector, expressed_in);
  J_full = zeros(6, nv);
  J_full(:, v_indices) = J;
  dJ_full_check = zeros(numel(J_full), nq);
  dJ_full_check = setSubMatrixGradient(dJ_full_check, dJ, 1:6, v_indices, size(J_full));
  old_time = old_time + toc;
  
  tic
  options.use_mex = false;
  options.compute_gradients = false;
  kinsol = robot.doKinematics(q, [], [], [], options);
  
  dJ_full = dJNew(robot, kinsol, base, end_effector, expressed_in);
  
  new_time = new_time + toc;
  
  valuecheck(dJ_full, dJ_full_check);
end

end

function dJ_full = dJNew(robot, kinsol, base, end_effector, expressed_in)
[~, path, signs] = findKinematicPath(robot, base, end_effector);

nv = robot.getNumVelocities();
cols = robot.getNumVelocities(); % TODO: make it possible to switch to q
J_size = [6, nv];
dJ_full = zeros(prod(J_size), cols);
for i = 1 : length(path)
  j = path(i);
  sign = signs(i);
  bodyJ = robot.body(j);
  qj = kinsol.q(bodyJ.position_num);
  Sj = sign * kinsol.J{j};
  [~, dSjdotDvj] = motionSubspace(bodyJ, qj);
  dSjdotDvj = sign * dSjdotDvj;
  % TODO: dSjdotDvj times appropriate vToqdot for gradient output
  AdHj = transformAdjoint(robot.relativeTransform(kinsol, expressed_in, j));
  AdH1 = transformAdjoint(robot.relativeTransform(kinsol, expressed_in, 1));
  [Jj, v_ind_ij] = robot.geometricJacobian(kinsol, expressed_in, j, 1);
  % TODO: q output for geometricJacobian
  
  for S_col = 1 : size(Sj, 2)
    col = bodyJ.velocity_num(S_col);
    
    block = AdHj * getSubMatrixGradient(dSjdotDvj, 1:6, S_col, size(Sj));
    %     block = block + getSubMatrixGradient(dJ_full, 1:6, col, size(J), bodyJ.velocity_num); % TODO: necessary?
    dJ_full = setSubMatrixGradient(dJ_full, block, 1:6, col, J_size, bodyJ.velocity_num);
    
%     block = -AdHj * crm(Sj(:, S_col)) * Jj;
    block = -AdH1 * crm(Sj(:, S_col)) * Jj;
    block = block + getSubMatrixGradient(dJ_full, 1:6, col, J_size, v_ind_ij);
    dJ_full = setSubMatrixGradient(dJ_full, block, 1:6, col, J_size, v_ind_ij);
  end
end
end
