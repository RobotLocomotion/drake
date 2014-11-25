function [y,ydot,yddot] = evalCubicSplineSegment(t,a0,a1,a2,a3)
% evalCubicSplineSegment: evaluate a cubic polynomial spline given time and coefficients
% See Craig, J. Introduction to Robotics: Mechanics and Control, 2005, chapter 7
  y = a0 + a1*t + a2*t^2 + a3*t^3;
  if nargout > 1
    ydot = a1 + 2*a2*t + 3*a3*t^2;
  end
  if nargout > 2
    yddot = 2*a2 + 6*a3*t;
  end
end

