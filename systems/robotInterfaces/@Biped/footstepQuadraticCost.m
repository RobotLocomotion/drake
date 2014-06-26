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
if seed_plan.footsteps(end).frame_id == biped.foot_frame_id.right
  xg = reshape(goal_pos.right, [], 1);
else
  xg = reshape(goal_pos.left, [], 1);
end
j = nsteps;
Q(world_ndx(:,j), world_ndx(:,j)) = w_goal;
c(world_ndx(:,j)) = -2 * xg' * w_goal;

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

