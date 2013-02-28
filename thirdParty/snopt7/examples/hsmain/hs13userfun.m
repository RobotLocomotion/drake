 function [F,G] = hsuserfun(x)
%function [F,G] = hsuserfun(x)
% Defines the nonlinear part of the function and derivatives
% for HS 13.

F  = [   x(1)+x(2);
         x(1)^3-x(2) ];

% Define the derivatives.
% The coordinates aren't used but are included as a safety check.

G = [ 1,  1,   1;
      1,  2,   1;
      2,  1,   3*x(1)^2
      2,  2,  -1      ];

G = G(:,3);
