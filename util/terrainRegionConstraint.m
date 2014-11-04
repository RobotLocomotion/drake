function [constraints, region] = terrainRegionConstraint(safe_regions, footsteps)

region = sdpvar(length(safe_regions), size(footsteps, 2), 'full');
constraints = sum(region, 1) == 1;

for j = 1:size(footsteps,2)
  for r = 1:length(safe_regions)
    A = safe_regions(r).A;
    b = safe_regions(r).b;
    Ar_ineq = [A(:,1:2), zeros(size(A, 1), 1), A(:,3)];
    br_ineq = b;
    Ar_eq = [safe_regions(r).normal', 0];
    br_eq = safe_regions(r).normal' * safe_regions(r).point;
    constraints = [constraints,...
      implies(region(r,j), Ar_ineq * footsteps(:,j) <= br_ineq),...
      implies(region(r,j), Ar_eq * footsteps(:,j) == br_eq)];
  end
end




