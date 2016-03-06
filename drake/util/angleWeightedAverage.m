function mu = angleWeightedAverage(theta1,w1,theta2,w2)
% Computes a weighted average between two angles by averaging 
% points on the unit circle and taking the arctan of the result.
%    see: http://en.wikipedia.org/wiki/Mean_of_circular_quantities
% @param theta1 a scalar or column vector of angles (rad)
% @param w1 weights for theta1 - scalar or equal size column vec
% @param theta2 a scalar or column vector of angles (rad)
% @param w2 weight for theta2 - scalar or equal size column vec
% assumes w1 + w2 = 1;

x1 = [cos(theta1),sin(theta1)];
x2 = [cos(theta2),sin(theta2)];

x_mean = w1.*x1+w2.*x2;

mu = atan2(x_mean(:,2),x_mean(:,1));

end
