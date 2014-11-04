function [constraints, trim] = finalTrimConstraint(footsteps)

nsteps = size(footsteps, 2);
trim = sdpvar(1, nsteps);

constraints = trim(1:end-1) <= trim(2:end);

for j = 1:(nsteps-2)
  if mod(nsteps-j, 2)
    constraints = [constraints, implies(trim(j), footsteps(:,j) == footsteps(:,end-1))];
  else
    constraints = [constraints, implies(trim(j), footsteps(:,j) == footsteps(:,end))];
  end
end
