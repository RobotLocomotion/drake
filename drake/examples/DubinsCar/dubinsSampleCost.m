function g = dubinsSampleCost(sys, x, u)
%DUBINSSAMPLECOST Sample cost function for discretization.
  
  g = 0.01*u^2 + multiplePointObstacleCost(x, [[0;0]]);

end

