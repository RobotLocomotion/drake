 function [F,G] = hsmainusrfun(x)
%function [F,G] = hsmainusrfun(x)
% Defines the nonlinear part of the function and derivatives
% for the problem HS47.

% Nonlinear part of F.
% The values of the linear rows are set to 0 (they are not used)

F = [    0;
       x(2)*x(2) + x(3)*x(3)*x(3);
         0;
      -x(3)*x(3);
       x(1)*x(5);
      (x(1)-x(2))^2 + (x(2)-x(3))^3 + (x(3)-x(4))^4 + (x(4)-x(5))^4 ];

% Define the derivatives.
% The coordinates aren't needed but we include them as a safety check.

Obj = 6;

G = [ Obj,    1,   2*(x(1)-x(2));
      Obj,    2,   3*(x(2)-x(3))^2 - 2*(x(1)-x(2));
      Obj,    3,   4*(x(3)-x(4))^3 - 3*(x(2)-x(3))^2;
      Obj,    4,   4*(x(4)-x(5))^3 - 4*(x(3)-x(4))^3;
      Obj,    5,  -4*(x(4)-x(5))^3;
        2,    2,   2* x(2);
        2,    3,   3* x(3)^2;
        4,    3,  -2* x(3);
        5,    1,   x(5);
        5,    5,   x(1)  ];
G = G(:,3);
