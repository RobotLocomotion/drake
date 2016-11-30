function [y,ydot,yddot] = evalCubicSplineSegment(t,coefs)
% evalCubicSplineSegment: evaluate a cubic polynomial spline given time and coefficients
% See Craig, J. Introduction to Robotics: Mechanics and Control, 2005, chapter 7
  y = coefs(:,:,1) * t^3 + coefs(:,:,2) * t^2 + coefs(:,:,3) * t + coefs(:,:,4);
  if nargout > 1
    ydot = 3 * coefs(:,:,1) * t^2 + 2 * coefs(:,:,2) * t + coefs(:,:,3);
  end
  if nargout > 2
    yddot = 6 * coefs(:,:,1) * t + 2 * coefs(:,:,2);
  end
end

