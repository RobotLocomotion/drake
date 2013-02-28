 function [x,F,INFO] = snoptmain3()
%function [x,F,INFO] = snoptmain3()
% Defines the NLP problem and calls the mex interface for snopt.
% Some, but not all first derivatives are provided.

snoptmain3.spc = which('snoptmain3.spc');

snprint  ( 'snoptmain3.out' );
snsummary( 'snoptmain3.sum' );
snspec   (  snoptmain3.spc  );
snseti   ( 'Verify level     ', 3);
snseti   ( 'Derivative option', 0);

% Get the data defining the Hexagon problem.
[x,xlow,xupp,Flow,Fupp,A,iAfun,jAvar,iGfun,jGvar] = hexagon;

[x,F,INFO] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfun3', ...
		   A, iAfun, jAvar, iGfun, jGvar);

snsummary off;
snprint   off; % Closes the file and empties the print buffer

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 function [x,xlow,xupp,Flow,Fupp,A,iAfun,jAvar,iGfun,jGvar] = hexagon()
%function [x,xlow,xupp,Flow,Fupp,A,iAfun,jAvar,iGfun,jGvar] = hexagon()
%
% Defines the problem hexagon:
%   maximize F(1)  (the objective row)
%   subject to
%            xlow <=   x  <= xupp
%            Flow <= F(x) <= Fupp
%   where
%     F( 1)  =  x_2 x_6 - x_1 x_7 + x_3 x_7 + x_5 x_8 - x_4 x_9 - x_3 x_8
%     F( 2)  =    x_1^2 + x_6^2
%     F( 3)  =   (x_2   - x_1)^2  +  (x_7 - x_6)^2
%     F( 4)  =   (x_3   - x_1)^2  +   x_6^2
%     F( 5)  =   (x_1   - x_4)^2  +  (x_6 - x_8)^2
%     F( 6)  =   (x_1   - x_5)^2  +  (x_6 - x_9)^2
%     F( 7)  =    x_2^2 + x_7^2
%     F( 8)  =   (x_3   - x_2)^2  +   x_7^2
%     F( 9)  =   (x_4   - x_2)^2  +  (x_8 - x_7)^2
%     F(10)  =   (x_2   - x_5)^2  +  (x_7 - x_9)^2
%     F(11)  =   (x_4   - x_3)^2  +   x_8^2
%     F(12)  =   (x_5   - x_3)^2  +   x_9^2
%     F(13)  =    x_4^2 +  x_8^2
%     F(14)  =   (x_4   - x_5)^2 + (x_9 - x_8)^2
%     F(15)  =    x_5^2 + x_9^2
%     F(16)  =   -x_1   + x_2
%     F(17)  =          -x_2 + x_3
%     F(18)  =                 x_3 - x_4
%     F(19)  =                       x_4 - x_5

neF    = 19;
n      =  9;
Obj    =  1; % The default objective row

% The nonzeros of the Jacobian of F are provided in snoptuserfun3

G = [Obj,   1
     Obj,   2
     Obj,   3
     Obj,   4
     Obj,   5
     Obj,   6
     Obj,   7
     Obj,   8
       1,   9
       2,   1
       2,   6
       3,   1
       3,   2
       3,   6
       3,   7
       4,   1
       4,   3
       4,   6
       5,   1
       5,   4
       5,   6
       5,   8
       6,   1
       6,   5
       6,   6
       6,   9
       7,   2
       7,   7
       8,   2
       8,   3
       8,   7
       9,   2
       9,   4
       9,   7
       9,   8
      10,   2
      10,   5
      10,   7
      10,   9
      11,   3
      11,   4
      11,   8
      12,   3
      12,   5
      12,   9
      13,   4
      13,   8
      14,   4
      14,   5
      14,   8
      14,   9
      15,   5
      15,   9 ];

iGfun = G(:,1); jGvar = G(:,2);

% Constant Jacobian elements are defined once here.

A = [ 16,   1,      -1
      16,   2,       1
      17,   2,      -1
      17,   3,       1
      18,   3,       1
      18,   4,      -1
      19,   4,       1
      19,   5,      -1 ];

iAfun = A(:,1);  jAvar = A(:,2);  A = A(:,3);


% Ranges for F.
% The Objective row (row 1 by default) is free.

Flow = zeros(neF,1);    Fupp = ones (neF,1);

Flow( 1:15)  = -Inf;    Fupp(16:neF) =  Inf;

Flow(Obj)    = -Inf;    Fupp(Obj)    =  Inf;

% Ranges for x.

xlow = -Inf*ones(n,1);  xupp =  Inf*ones(n,1);

xlow(1) =  0;
xlow(3) = -1;
xlow(5) =  0;
xlow(6) =  0;
xlow(7) =  0;

xupp(3) =  1;
xupp(8) =  0;
xupp(9) =  0;

x   =  [ .1;
         .125;
         .666666;
         .142857;
         .111111;
         .2;
         .25;
        -.2;
        -.25 ];

snset('Maximize');
