function [constraints, sin_sector, cos_sector] = sinCosPiecewiseLinear(sin_theta, cos_theta, theta, theta_lb, theta_ub)

cos_boundaries = reshape(bsxfun(@plus, [theta_lb:pi:theta_ub; theta_lb:pi:theta_ub], [-(pi/2-1); (pi/2-1)]), 1, []);
sin_boundaries = reshape(bsxfun(@plus, [theta_lb:pi:theta_ub; theta_lb:pi:theta_ub], [-1; 1]), 1, []);

cos_sector = sdpvar(length(cos_boundaries) - 1, length(theta), 'full');
sin_sector = sdpvar(length(sin_boundaries) - 1, length(theta), 'full');

constraints = [sum(cos_sector, 1) == 1, sum(sin_sector, 1) == 1,...
  theta_lb <= theta <= theta_ub,...
  -1 <= cos_theta <= 1,...
  -1 <= sin_theta <= 1,...
  polycone([cos_theta; sin_theta], norm([pi/4;pi/4]), 8),...
  ];

for j = 1:length(theta)
  for s = 1:length(cos_boundaries) - 1
    th0 = cos_boundaries(s);
    th1 = cos_boundaries(s+1);

    th = (th0 + th1)/2;
    cos_slope = -sin(th);
    cos_intercept = cos(th) - (cos_slope * th);
    constraints = [constraints,...
                   implies(cos_sector(s, j), th0 <= theta(j) <= th1),...
                   implies(cos_sector(s, j), cos_theta(j) == cos_slope * theta(j) + cos_intercept)];
  end
  for s = 1:length(sin_boundaries) - 1
    th0 = sin_boundaries(s);
    th1 = sin_boundaries(s+1);

    th = (th0 + th1)/2;
    sin_slope = cos(th);
    sin_intercept = sin(th) - (sin_slope * th);
    constraints = [constraints,...
                   implies(sin_sector(s, j), th0 <= theta(j) <= th1),...
                   implies(sin_sector(s, j), sin_theta(j) == sin_slope * theta(j) + sin_intercept)];
  end
  for k = 1:size(sin_sector, 1)
    constraints = [constraints,...
                   sum(sin_sector(max(1,k-1):min(k+1,size(sin_sector,1)),j)) >= cos_sector(k,j),...
                   sum(cos_sector(max(1,k-1):min(k+1,size(cos_sector,1)),j)) >= sin_sector(k,j)];
  end
end

