function mu = angleAverage(theta1,theta2)
% Computes the average between two angles by averaging points on the unit
% circle and taking the arctan of the result.
%    see: http://en.wikipedia.org/wiki/Mean_of_circular_quantities
% @param theta1 a scalar or column vector of angles (rad)
% @param theta2 a scalar or column vector of angles (rad)

x1 = [cos(theta1),sin(theta1)];
x2 = [cos(theta2),sin(theta2)];

x_mean = (x1+x2)./2;

mu = atan2(x_mean(:,2),x_mean(:,1));

end
