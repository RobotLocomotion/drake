 function [F,G] = toyusrfun(x)
%function [F,G] = toyusrfun(x)
% Defines the nonlinear part of the function and derivatives
% for the toy problem.

F  = [ 3*x(1)   +  (x(1)   + x(2) + x(3))^2 + 5*x(4);
       4*x(2)   + 2*x(3);
         x(1)   +   x(2)^2 + x(3)^2;
         x(2)^4 +   x(3)^4 + x(4) ];

% Define the derivatives.
% The coordinates aren't used but are included as a safety check.

G = [ 1,  1,  2*(x(1)+x(2)+x(3)) + 3;
      1,  2,  2*(x(1)+x(2)+x(3));
      1,  3,  2*(x(1)+x(2)+x(3));
      1,  4,  5;
      2,  2,  4;
      2,  3,  2;
      3,  1,  1;
      3,  2,  2*x(2);
      3,  3,  2*x(3);
      4,  2,  4*x(2)^2;
      4,  3,  4*x(3)^3;
      4,  4,  1 ];

G = G(:,3);
