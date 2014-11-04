function [constraints, sector] = sinCosUnitCircleLinearEquality(sin_theta, cos_theta, theta, theta_lb, theta_ub, slices)

nsteps = length(theta);

sector_width = 2*pi / slices;
angle_boundaries = (-pi-sector_width/2):(sector_width):(pi-sector_width/2);
sector = sdpvar(length(angle_boundaries) - 1, nsteps, 'full');

constraints = [sum(sector, 1) == 1,...
  theta_lb <= theta <= theta_ub,...
  -1 <= sin_theta <= 1,...
  -1 <= cos_theta <= 1,...
  0 <= sector <= 1,...
  polycone([cos_theta; sin_theta], 1, slices),...
  ];

for s = 1:length(angle_boundaries)-1
  th0 = angle_boundaries(s);
  th1 = angle_boundaries(s+1);
  th = (th0 + th1) / 2;
  ct = cos(th);
  st = sin(th);
  k = tan((th1 - th0)/2) / ((th1 - th0) / 2);
  for j = 1:nsteps
    constraints = [constraints,...
      implies(sector(s,j), th0 <= theta(j) <= th1),...
      implies(sector(s,j), [cos_theta(j); sin_theta(j)] == [ct; st] + (theta(j) - th) * k * [-st; ct])];
  end
end
