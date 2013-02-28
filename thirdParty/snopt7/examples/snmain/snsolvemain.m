
 function [x,F,xmul,Fmul,INFO] = snsolvemain()
%function [x,F,xmul,Fmul,INFO] = snsolvemain()
%  (1) Defines the data for the problem Hexagon.
%  (2) Solves the problem using the full-blown
%      mex interface snsolve.

snprint  ('snsolvemain.out');
snsummary('snsolvemain.sum');
snsolvemain.spc  = which('snsolvemain.spc');
snspec   (snsolvemain.spc);
snseti   ('Major Iteration limit', 250);

[x,xlow,xupp,xmul,xstate,Flow,Fupp,Fmul,Fstate,ObjAdd,ObjRow, ...
            A,iAfun,jAvar,iGfun,jGvar] = hexagon;

snset    ('Maximize');

[x,F,xmul,Fmul,INFO]= snsolve( x,xlow,xupp,xmul,xstate,    ...
			       Flow,Fupp,Fmul,Fstate,      ...
			       ObjAdd,ObjRow,A,iAfun,jAvar,...
			       iGfun,jGvar,'snsolveuserfun');

snsummary off;
snprint   off; % Closes the file and empties the print buffer


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 function [x,xlow,xupp,xmul,xstate,Flow,Fupp,Fmul,Fstate,ObjAdd,ObjRow, ...
	   A,iAfun,jAvar,iGfun,jGvar] = hexagon()
%function [x,xlow,xupp,xmul,xstate,Flow,Fupp,Fmul,Fstate,ObjAdd,ObjRow, ...
%	   A,iAfun,jAvar,iGfun,jGvar] = hexagon()
%
% Defines the problem hexagon:
%   maximize F(1)  (the objective row)
%   subject to
%            xlow <=   x  <= xupp
%            Flow <= F(x) <= Fupp
%   where
%     F( 1) =   x_2 x_6 - x_1 x_7 + x_3 x_7 + x_5 x_8 - x_4 x_9 - x_3 x_8
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
%     F(16)  =   -x_1 + x_2
%     F(17)  =         -x_2 + x_3
%     F(18)  =                x_3 - x_4
%     F(19)  =                      x_4 - x_5


%
%     The pattern of the full Jacobian is as follows, where
%     A = constant element, G = nonlinear element.
%
%               Column
%             | 1   2    3    4    5   6   7   8   9
%             +--------------------------------------
%     Row  1  | G   G    G    G    G   G   G   G   G  Objective row
%          2  | G                      G
%          3  | G   G                  G   G
%          4  | G        G             G
%          5  | G             G        G       G
%          6  | G                  G   G           G
%          7  |     G                      G
%          8  |     G    G                 G
%          9  |     G         G            G   G
%         10  |     G              G       G       G
%         11  |          G    G                G
%         12  |          G         G               G
%         13  |               G                G
%         14  |               G    G           G   G
%         15  |                    G               G
%         16  | A   A
%         17  |     A    A
%         18  |          A    A
%         19  |               A    A
%

neF    = 19;
n      =  9;
Obj    =  1;
ObjRow =  1;  % The default objective row

% Linear Jacobian  elements  (i,j,Aij)

A = [ 16,  1,  -1;
      16,  2,   1;
      17,  2,  -1;
      17,  3,   1;
      18,  3,   1;
      18,  4,  -1;
      19,  4,   1;
      19,  5,  -1 ];

iAfun = A(:,1); jAvar = A(:,2); A = A(:,3);

% Coordinates of the nonlinear Jacobian elements
% For the complete coordinates, see snoptuserfun.m

G = [ Obj,   1;
      Obj,   2;
      Obj,   3;
      Obj,   4;
      Obj,   5;
      Obj,   6;
      Obj,   7;
      Obj,   8;
      Obj,   9;
        2,   1;
        2,   6;
        3,   1;
        3,   2;
        3,   6;
        3,   7;
        4,   1;
        4,   3;
        4,   6;
        5,   1;
        5,   4;
        5,   6;
        5,   8;
        6,   1;
        6,   5;
        6,   6;
        6,   9;
        7,   2;
        7,   7;
        8,   2;
        8,   3;
        8,   7;
        9,   2;
        9,   4;
        9,   7;
        9,   8;
       10,   2;
       10,   5;
       10,   7;
       10,   9;
       11,   3;
       11,   4;
       11,   8;
       12,   3;
       12,   5;
       12,   9;
       13,   4;
       13,   8;
       14,   4;
       14,   5;
       14,   8;
       14,   9;
       15,   5;
       15,   9 ];

iGfun = G(:,1); jGvar = G(:,2);

% Ranges for F.

Flow = zeros(neF,1);
Fupp = ones (neF,1);

Flow( 1:15)  = -Inf;
Fupp(16:neF) =  Inf;

% The Objective row is free.

Flow(Obj)  = -Inf;
Fupp(Obj)  =  Inf;

ObjAdd  = 0;

% Ranges for x.

xlow = -Inf*ones(n,1);
xupp =  Inf*ones(n,1);

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
xstate = zeros(n,1);
xmul   = zeros(n,1);

Fmul   = zeros(neF,1);
Fstate = zeros(neF,1);
