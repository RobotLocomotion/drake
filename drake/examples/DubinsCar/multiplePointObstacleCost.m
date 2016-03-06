function g = multiplePointObstacleCost(x, obstacle_positions)
  
  function c = singlePointObstacleCost(x_plant, x_obs)
  % For 2-dimensions, calculates the cost an obstacle imposes on
  % the plant.
  % @param x_plant Array of doubles of size (2, ns) representing
  % the horizontal and vertical coordinates of the plants for which
  % the cost is being evaluated.
  % @param x_obs Array of doubles of size (2, 1) representing the
  % horizontal and vertical coordinates of the obstacle.
    ns = size(x_plant,2); % number of discretized states
    distance_along_axes = x_plant-repmat(x_obs,1,ns);
    l2_norm = sqrt(sum(abs(distance_along_axes).^2,1));
    c = exp(-l2_norm.^2);
  end
  
  num_obstacles = size(obstacle_positions, 2);
  x_linear = x(1:2,:);
  g = 0;
  for n = 1:num_obstacles
    g = g + singlePointObstacleCost(x_linear, obstacle_positions(:,n));
  end

end

